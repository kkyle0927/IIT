/**
 ******************************************************************************
 * @file    core_process.c
 * @author  HyundoKim
 * @brief   User Task 구동 및 XM API 데이터 동기화 엔진 구현부
 * @details
 * 이 모듈은 System Layer에 위치하지만, XM API(Interface)의
 * 'Backend Implementation' 역할을 수행합니다.
 * 따라서 상위 인터페이스인 xm_api_data.h에 의존하여
 * 데이터를 채워 넣습니다 (Dependency Inversion 적용).
 * @version 0.1
 * @date    Nov 17, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "core_process.h"

/* --- 1. Interfaces & APIs (계약서) --- */
#include "xm_api.h"     // 통합 파사드

/* --- 2. Device Drivers --- */
#include "cm_drv.h"             // Control Module 드라이버 (Rx/Tx + PnP 통합)
#include "imu_hub_drv.h"        // IMU Hub Driver (DOP V2)
#include "emg_hub_drv.h"        // EMG Hub Driver (DOP V2)
#include "fes_hub_drv.h"        // FES Hub Driver (DOP V2)
#include "mdaf-25-6850.h"       // GRF(FSR) Device (Auto-Sense + DataLake)
#include "mti-630.h"            // XSENS IMU Device (Auto-Sense + DataLake)
#include "xm_total_data.h"      // Total Data Packet 스냅샷 (System-Managed CDC)
#include "system_startup.h"      // System_SendSync_Ch1() — CiA 301 SYNC 전송
#include "ioif_agrb_tim.h"
#include "xm_api_usb.h"          // XM_USB_GetMode() — PRODUCTION 모드 가드

/* --- 3. RTOS & Utilities --- */
#include "FreeRTOS.h"
#include "task.h"

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

/**
 * @brief 전역 로봇 데이터 인스턴스 (End User가 'XM'으로 접근하는 실체)
 * @note  xm_api_data.h에 extern 선언되어 있음
 */
extern XmRobot_t XM; // 전역 인스턴스 정의


/* 사용자가 작성할 함수들 (링커가 찾을 수 있도록 extern 선언).
 * Control_Setup / Control_Loop 는 control_task.c (사용자 작성) 또는
 * user_compat.c (Backward-compat weak shim — User_* 로 forward) 가 제공. */
extern void Control_Setup(void);
extern void Control_Loop(void);

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

static UBaseType_t stack_remaining_words;
static uint32_t stack_remaining_bytes;

/* ─────────────────────────────────────────────────────────────────────────
 * [2026-05-14] Task Stack High-Water-Mark 모니터링 — 5s 주기 갱신.
 *
 * 목적:
 *   - v2.1.1 릴리즈 무한리셋 진단([[project_v211_cdc_reset_hang_diagnosis]])
 *     1순위 의심 = usbContolTask (1KB) stack overflow → 빈 hook → HardFault
 *     hang. 현 dev 에선 hook 가 freeze 로 잡지만, 실제 task 별 마진을 실측
 *     검증하기 위한 모니터링.
 *   - sensor-studio / phai-studio 시나리오 별 stack 소비 추적.
 *
 * 사용:
 *   - CubeProgrammer Live View 로 g_task_watermarks 주소 read.
 *   - watermark_words 가 minimum stack space (Words, 4B 단위). 작을수록 위험.
 *   - 5s 주기로 갱신 — 한 번 측정한 값은 task 의 lifetime 최저값.
 *
 * 주의:
 *   - xTaskGetHandle 는 FreeRTOSConfig.h 의 INCLUDE_xTaskGetHandle=1 필요.
 *   - task name 이 실제 osThreadAttr_t.name 과 정확히 일치해야 함.
 *   - StimulusTask 는 sensor-studio 연결 시에만 활성, 그 외엔 suspend → 측정 가능.
 *   - DefaultTask 는 main.c 에서 osThreadSuspend 됨 → 측정 의미 적음.
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    const char* name;
    uint32_t    stack_size_bytes;   /* osThreadAttr_t.stack_size 동기화 (참고용) */
    UBaseType_t watermark_words;    /* uxTaskGetStackHighWaterMark() — 작을수록 부족 */
    uint32_t    watermark_bytes;    /* watermark_words * 4 */
    uint8_t     handle_resolved;    /* xTaskGetHandle 성공 여부 (0=name 불일치 가능) */
} TaskWatermark_t;

/* 측정 대상 — module.h / main.c / xm_periph_stimulus.c / pnp_task.c /
 * data_logger.c / canfd_rx_handler.c 의 osThreadAttr_t.name 그대로. */
volatile TaskWatermark_t g_task_watermarks[] = {
    { "UserTask",       8192u * 4u, 0u, 0u, 0u },  /* main.c IOC */
    { "usbContolTask",  2048u,      0u, 0u, 0u },  /* module.h:111 (2026-05-14 상향) */
    { "PnP_Task",       2048u,      0u, 0u, 0u },  /* module.h:107 */
    { "StimulusTask",   4096u,      0u, 0u, 0u },  /* xm_periph_stimulus.c:51 */
    { "DataLoggerTask", 2048u * 4u, 0u, 0u, 0u },  /* module.h:134 (USB_SAVE = 8KB) */
    { "NRT_Proc",       2048u,      0u, 0u, 0u },  /* canfd_rx_handler.c */
    { "StartupTask",    512u  * 4u, 0u, 0u, 0u },  /* main.c IOC (2KB) */
};
#define G_TASK_WATERMARK_COUNT (sizeof(g_task_watermarks) / sizeof(g_task_watermarks[0]))

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _FetchAllInputs(void);
static void _FlushAllOutputs(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void StartUserTask(void *argument)
{
    /* 1. User Initialization (1회 실행) */
    // End User가 작성한 초기화 코드 실행 (TSM 생성, 초기값 설정 등)
    Control_Setup();

    /* 2. Timing Initialization */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 1ms 주기 고정 (1kHz)

    /* 3. Main Control Loop */
    for (;;)
    {
        /* [Timing Control] */
        // 정확한 1ms 주기를 보장하기 위해 vTaskDelayUntil 사용
        // 알고리즘 연산 시간이 1ms를 넘지 않도록 주의해야 함
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* [Step 1: Input] Data Gathering */
        // 모든 센서 데이터를 User가 보기 편한 구조체(XM.status)로 업데이트
        _FetchAllInputs();

        /* [Step 1.2: SYNC broadcast] _FetchAllInputs() 직후 SYNC 전송.
         * CM/SM이 다음 PDO를 준비하도록 트리거 → 다음 사이클 Read에서 수신.
         * Read↔SYNC가 같은 태스크에서 실행되어 phase drift 제거. */
        if (CM_Drv_IsConnected()) {
            System_SendSync_Ch1();          /* Ch1: XM↔CM (DOP V1) */
        }
        if (FesHub_Drv_IsConnected() || ImuHub_Drv_IsConnected() || EmgHub_Drv_IsConnected()) {
            System_SendSync_Ch2();          /* Ch2: XM↔SM (DOP V3, Phase-Lock) */
        }

        /* [Step 1.5: Total Data Snapshot] System-Managed CDC 데이터 수집 */
        // _FetchAllInputs() 직후, 모든 센서 raw 데이터를 패킷 버퍼에 스냅샷
        // USB CDC Total Data(Module ID 0x20)로 자동 전송됨
        XM_TotalData_Snapshot();

        /* [Step 2: Process] User Algorithm — PRODUCTION 모드에서 차단.
         *
         * 양산 라인 / SI 검증 중에는 sensor-studio 가 OD 0x7E00:N stimulus
         * 로 페리페럴(FDCAN/SPI/UART/PSRAM)을 직접 토글한다. 사용자가 작성한
         * Control_Loop 가 동일 페리페럴을 만지면 라우팅·register 상태 충돌이
         * 발생하므로, 모드가 PRODUCTION 인 동안 사용자 알고리즘 호출을 건너뛴다.
         *
         * core_process 의 _FetchAllInputs / SYNC / _FlushAllOutputs 등 시스템
         * 자체 R/W 는 평시처럼 유지 — 외부 디바이스가 PDO/UART 트래픽을
         * 정상적으로 받아야 PRODUCTION 모드에서도 의미 있는 SI 데이터가 흐른다.
         */
        if (XM_USB_GetMode() != XM_USB_MODE_PRODUCTION) {
            Control_Loop();
        }

        /* [Step 3: Output] Command Flushing */
        // User가 구조체(XM.command)에 쓴 값을 실제 하드웨어로 전송
        _FlushAllOutputs();

        /* [Step 4: USB Data Handling] Logging or Streaming */
        // Tx가 끝난 시점(데이터 확정)에서 로깅 및 모니터링 수행
        XM_USB_ProcessPeriodic();

        // --- 스택 모니터링 (디버깅용) ---
        static uint32_t last_check_time = 0;
        uint32_t current_time = IOIF_TIM_GetTick(); //

        // 5초(5000ms)에 한 번씩 스택 상태를 업데이트
        // 오버플로우 발생시 LED indicate?
        if (current_time - last_check_time > 5000) {
            last_check_time = current_time;

            // 1. 현재 태스크(NULL 전달)의 스택 High Water Mark를 확인합니다.
            //    (단위: Words, 4바이트)
            stack_remaining_words = uxTaskGetStackHighWaterMark(NULL);

            // 2. 바이트 단위로 변환
            stack_remaining_bytes = stack_remaining_words * 4;

            /* [2026-05-14] 7개 task watermark 일괄 측정 — g_task_watermarks 갱신.
             * CubeProgrammer Live View 로 read. INCLUDE_xTaskGetHandle=1 필요. */
            for (uint32_t i = 0; i < G_TASK_WATERMARK_COUNT; i++) {
                TaskHandle_t h = xTaskGetHandle(g_task_watermarks[i].name);
                if (h != NULL) {
                    UBaseType_t w = uxTaskGetStackHighWaterMark(h);
                    g_task_watermarks[i].watermark_words = w;
                    g_task_watermarks[i].watermark_bytes = (uint32_t)w * 4U;
                    g_task_watermarks[i].handle_resolved = 1U;
                } else {
                    /* handle 미확보 — task 미생성 (예: StimulusTask suspend 시점
                     * 에도 handle 은 valid) 또는 name 불일치. handle_resolved=0
                     * 으로 표시, watermark 는 이전 값 유지. */
                    g_task_watermarks[i].handle_resolved = 0U;
                }
            }

        }
    }
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/* ---- [IPO Step 1] Input Gathering (Rx Integration) ---- */
/**
 * @brief [IPO Step 1] Input Gathering
 * @details 하드웨어 계층(Link/Driver)의 최신 데이터를 API 계층(Facade)으로 복사합니다.
 * 데이터의 원자성(Atomicity)과 동기화(Synchronization)를 보장합니다.
 */
static void _FetchAllInputs(void)
{
    /* 1. Control Module (KIT H10) Data */
    // CM_RxData_t는 cm_drv.h에 정의된 구조체 (float 변환 완료된 상태)
    CM_RxData_t h10_data;

    // CM은 전원을 공급하므로 항상 물리적으로 연결되어 있다고 가정하지만,
    // 논리적 통신 상태(Operational)를 확인하는 것이 안전합니다.
    if (CM_Drv_IsConnected()) {
        // Mutex로 보호된 안전한 데이터 복사 수행
        // 실패 시 XM.status.h10은 이전 값 유지 (garbage 방지)
        if (CM_GetRxData(&h10_data)) {
            XM.status.h10.is_connected = true;

            // --- Info & State ---
            XM.status.h10.h10AssistModeLoopCnt  = h10_data.pdo.h10AssistModeLoopCnt;
            XM.status.h10.h10PostProcessingCnt  = h10_data.pdo.postProcessingCnt;
            XM.status.h10.h10Mode               = (XmH10Mode_t)h10_data.sdo.h10Mode;
            XM.status.h10.h10AssistLevel        = h10_data.sdo.h10AssistLevel;
            XM.status.h10.h10FSMcurrentState    = h10_data.pdo.h10FSMcurrentState;
            XM.status.h10.h10IsNeutralPosSet    = h10_data.sdo.h10NeutralPosSet;
            XM.status.h10.isPVectorRHDone       = h10_data.sdo.isPVectorRHDone;
            XM.status.h10.isPVectorLHDone       = h10_data.sdo.isPVectorLHDone;

            // --- Kinematics Data ---
            XM.status.h10.leftHipAngle     = h10_data.pdo.leftHipAngle;
            XM.status.h10.rightHipAngle    = h10_data.pdo.rightHipAngle;
            XM.status.h10.leftThighAngle   = h10_data.pdo.leftThighAngle;
            XM.status.h10.rightThighAngle  = h10_data.pdo.rightThighAngle;
            XM.status.h10.leftKneeAngle    = h10_data.pdo.leftKneeAngle;
            XM.status.h10.rightKneeAngle   = h10_data.pdo.rightKneeAngle;
            XM.status.h10.pelvicAngle      = h10_data.pdo.pelvicAngle;

            // --- Gait Data ---
            XM.status.h10.isLeftFootContact  = h10_data.pdo.isLeftFootContact;
            XM.status.h10.isRightFootContact = h10_data.pdo.isRightFootContact;
            XM.status.h10.forwardVelocity    = h10_data.pdo.forwardVelocity;

            // --- Motor Data ---
            XM.status.h10.leftHipTorque      = h10_data.pdo.leftHipTorque;
            XM.status.h10.rightHipTorque     = h10_data.pdo.rightHipTorque;
            XM.status.h10.leftHipMotorAngle  = h10_data.pdo.leftHipMotorAngle;
            XM.status.h10.rightHipMotorAngle = h10_data.pdo.rightHipMotorAngle;

            // --- IMU Data ---
            XM.status.h10.leftHipImuFrontalRoll    = h10_data.pdo.leftHipImuFrontalRoll;
            XM.status.h10.leftHipImuSagittalPitch  = h10_data.pdo.leftHipImuSagittalPitch;
            XM.status.h10.rightHipImuFrontalRoll   = h10_data.pdo.rightHipImuFrontalRoll;
            XM.status.h10.rightHipImuSagittalPitch = h10_data.pdo.rightHipImuSagittalPitch;
            XM.status.h10.leftHipImuTransverseYaw  = h10_data.pdo.leftHipImuTransverseYaw;
            XM.status.h10.rightHipImuTransverseYaw = h10_data.pdo.rightHipImuTransverseYaw;

            XM.status.h10.leftHipImuGlobalAccX = h10_data.pdo.leftHipImuGlobalAccX;
            XM.status.h10.leftHipImuGlobalAccY = h10_data.pdo.leftHipImuGlobalAccY;
            XM.status.h10.leftHipImuGlobalAccZ = h10_data.pdo.leftHipImuGlobalAccZ;

            XM.status.h10.leftHipImuGlobalGyrX = h10_data.pdo.leftHipImuGlobalGyrX;
            XM.status.h10.leftHipImuGlobalGyrY = h10_data.pdo.leftHipImuGlobalGyrY;
            XM.status.h10.leftHipImuGlobalGyrZ = h10_data.pdo.leftHipImuGlobalGyrZ;

            XM.status.h10.rightHipImuGlobalAccX = h10_data.pdo.rightHipImuGlobalAccX;
            XM.status.h10.rightHipImuGlobalAccY = h10_data.pdo.rightHipImuGlobalAccY;
            XM.status.h10.rightHipImuGlobalAccZ = h10_data.pdo.rightHipImuGlobalAccZ;

            XM.status.h10.rightHipImuGlobalGyrX = h10_data.pdo.rightHipImuGlobalGyrX;
            XM.status.h10.rightHipImuGlobalGyrY = h10_data.pdo.rightHipImuGlobalGyrY;
            XM.status.h10.rightHipImuGlobalGyrZ = h10_data.pdo.rightHipImuGlobalGyrZ;
        }
        // Mutex 실패 시: XM.status.h10은 이전 값 그대로 유지 (duplicate > garbage)
    } else {
        XM.status.h10.is_connected = false;
        // 연결 끊김 시 데이터 처리 정책 (0으로 초기화 또는 이전 값 유지)
    }

    /* 2. GRF Sensor Data Mapping (Mutex + Snapshot) (Optional) */
    MarvelDex_packet_t grf_L, grf_R;
    
    /* GRF Left (ch=0) */
    if (MarvelDex_IsOnline(MARVELDEX_CH_LEFT)) {
        MarvelDex_GetLatest(MARVELDEX_CH_LEFT, &grf_L);
        XM.status.grf.is_left_grf_connected = true;
                
        XM.status.grf.leftLastUpdateTick = grf_L.timestamp;
        XM.status.grf.leftSensorSpace = (XM_GRF_SPACE_e)grf_L.sensorSpace;
        XM.status.grf.leftRollingIndex = grf_L.rollingIndex;
        memcpy(XM.status.grf.leftSensorData, grf_L.sensorData, XM_GRF_CHANNEL_SIZE);
        XM.status.grf.leftBatteryLevel = grf_L.batteryLevel;
        XM.status.grf.leftStatusFlags  = grf_L.statusFlags;
    } else {
        XM.status.grf.is_left_grf_connected = false;
    }
    
    /* GRF Right (ch=1) */
    if (MarvelDex_IsOnline(MARVELDEX_CH_RIGHT)) {
        MarvelDex_GetLatest(MARVELDEX_CH_RIGHT, &grf_R);
        XM.status.grf.is_right_grf_connected = true;
                
        XM.status.grf.rightLastUpdateTick = grf_R.timestamp;
        XM.status.grf.rightSensorSpace = (XM_GRF_SPACE_e)grf_R.sensorSpace;
        XM.status.grf.rightRollingIndex = grf_R.rollingIndex;
        memcpy(XM.status.grf.rightSensorData, grf_R.sensorData, XM_GRF_CHANNEL_SIZE);
        XM.status.grf.rightBatteryLevel = grf_R.batteryLevel;
        XM.status.grf.rightStatusFlags  = grf_R.statusFlags;
    } else {
        XM.status.grf.is_right_grf_connected = false;
    }

    /* 3. External UART IMU Data (Xsens MTi-630, ch=0) — IMU Hub와 구분 */
    XsensMTi_packet_t ext_imu_packet;
    if (XsensMTi_IsOnline(0)) {
        XsensMTi_GetLatest(0, &ext_imu_packet);

        XM.status.ext_imu.is_connected   = true;
        XM.status.ext_imu.lastUpdateTick = ext_imu_packet.timestamp;

        // Quaternion
        XM.status.ext_imu.q_w = ext_imu_packet.q_w;
        XM.status.ext_imu.q_x = ext_imu_packet.q_x;
        XM.status.ext_imu.q_y = ext_imu_packet.q_y;
        XM.status.ext_imu.q_z = ext_imu_packet.q_z;

        // Acceleration
        XM.status.ext_imu.acc_x = ext_imu_packet.acc_x;
        XM.status.ext_imu.acc_y = ext_imu_packet.acc_y;
        XM.status.ext_imu.acc_z = ext_imu_packet.acc_z;

        // Gyroscope
        XM.status.ext_imu.gyr_x = ext_imu_packet.gyr_x;
        XM.status.ext_imu.gyr_y = ext_imu_packet.gyr_y;
        XM.status.ext_imu.gyr_z = ext_imu_packet.gyr_z;
    } else {
        XM.status.ext_imu.is_connected = false;
    }
    
    /* 4. IMU Hub Module Data — [Snapshot/HD/IMU_Resolution_For_Swiss] 단일 IMU float32 원본 */
    ImuHub_ImuDataF_t imu_hub_f32;
    uint32_t imu_hub_ts   = 0;
    uint8_t  imu_hub_mask = 0;
    if (ImuHub_Drv_IsConnected()) {  /* ✅ V3.0: Device Driver API 직접 호출 */
        XM.status.imu_hub.is_connected = true;
        /* IMU Hub float32 원본 데이터 (Mutex 보호 Snapshot) — 다운스케일/스케일 없음 */
        if (ImuHub_Drv_GetRxDataFloat(&imu_hub_f32, &imu_hub_ts, &imu_hub_mask)) {
            XM.status.imu_hub.lastUpdateTick = imu_hub_ts;
            XM.status.imu_hub.connected_mask = imu_hub_mask;

            /* 단일 IMU(첫 연결 채널) → sensor[0] 에 float32 원본 직접 대입 */
            XmImuHubSensor_t* dst = &XM.status.imu_hub.sensor[0];
            dst->q_w = imu_hub_f32.q[0];
            dst->q_x = imu_hub_f32.q[1];
            dst->q_y = imu_hub_f32.q[2];
            dst->q_z = imu_hub_f32.q[3];
            dst->acc_x = imu_hub_f32.a[0];
            dst->acc_y = imu_hub_f32.a[1];
            dst->acc_z = imu_hub_f32.a[2];
            dst->gyr_x = imu_hub_f32.g[0];
            dst->gyr_y = imu_hub_f32.g[1];
            dst->gyr_z = imu_hub_f32.g[2];
        }
    } else {
        XM.status.imu_hub.is_connected = false;
    }

    /* 5. EMG Hub Module Data (sEMG 1ch, DOP V2) */
    EmgHub_RxData_t emg_hub_data;
    if (EmgHub_Drv_IsConnected()) {
        XM.status.emg_hub.is_connected = true;
        if (EmgHub_Drv_GetRxData(&emg_hub_data)) {
            /* lastUpdateTick = Slave 제어 틱 (ctrl_tick_ms, 32-bit, OD 0x6050).
             * 구 14B 포맷 수신 시 _DecodeTpdo1 이 Metadata timestamp(24-bit) 로 fallback 설정. */
            XM.status.emg_hub.lastUpdateTick  = emg_hub_data.ctrl_tick_ms;
            XM.status.emg_hub.raw_adc         = emg_hub_data.raw_adc;
            XM.status.emg_hub.voltage_uv      = (float)emg_hub_data.voltage_uv_x10 / EMGHUB_SCALE_UV_X10;
            XM.status.emg_hub.rms_uv          = (float)emg_hub_data.rms_uv_x10 / EMGHUB_SCALE_UV_X10;
            XM.status.emg_hub.envelope_uv     = (float)emg_hub_data.envelope_uv_x10 / EMGHUB_SCALE_UV_X10;
            XM.status.emg_hub.mvc_percent     = emg_hub_data.mvc_percent;
            XM.status.emg_hub.is_active       = emg_hub_data.is_active;
            XM.status.emg_hub.status_flags    = emg_hub_data.status_flags;
        }
    } else {
        XM.status.emg_hub.is_connected = false;
    }

    /* 6. FES Hub Module Data (2ch FES feedback, DOP V3 ES-vector + KHJ telemetry) */
    FesHub_RxData_t fes_hub_data;
    if (FesHub_Drv_IsConnected()) {
        XM.status.fes_hub.is_connected = true;
        if (FesHub_Drv_GetRxData(&fes_hub_data)) {
            XM.status.fes_hub.lastUpdateTick = fes_hub_data.timestamp;

            /* Legacy 16B — 채널 상태 / 전류 / HV / Error */
            for (int i = 0; i < XM_FES_HUB_CH_COUNT; i++) {
                XM.status.fes_hub.ch_state[i]      = (uint8_t)fes_hub_data.ch_state[i];
                XM.status.fes_hub.ch_current_mA[i] = fes_hub_data.ch_current_mA[i];
                XM.status.fes_hub.ch_fault_code[i] = fes_hub_data.ch_fault_code[i];
            }
            XM.status.fes_hub.hv_voltage_V    = fes_hub_data.hv_voltage_V;
            XM.status.fes_hub.digipot_pos     = fes_hub_data.digipot_pos;
            XM.status.fes_hub.es_state_packed = fes_hub_data.es_state_packed;
            XM.status.fes_hub.error_register  = fes_hub_data.error_register;

            /* KHJ telemetry 21B — FSM / ISI / target / impedance / pulse / voltage */
            XM.status.fes_hub.fsm_state      = fes_hub_data.fsm_state;
            XM.status.fes_hub.fsm_state_prev = fes_hub_data.fsm_state_prev;
            XM.status.fes_hub.isi_packed     = fes_hub_data.isi_packed;
            for (int i = 0; i < XM_FES_HUB_CH_COUNT; i++) {
                XM.status.fes_hub.ch_es_error_lo[i]         = fes_hub_data.ch_es_error_lo[i];
                XM.status.fes_hub.ch_target_amplitude_mA[i] = fes_hub_data.ch_target_amplitude_mA[i];
                XM.status.fes_hub.ch_impedance[i]           = fes_hub_data.ch_impedance[i];
                XM.status.fes_hub.ch_pulse_cnt[i]           = fes_hub_data.ch_pulse_cnt[i];
                XM.status.fes_hub.ch_voltage_diff_V[i]      = fes_hub_data.ch_voltage_diff_V[i];
            }
        }
    } else {
        XM.status.fes_hub.is_connected = false;
    }

    /* 7. I/O (LED & Button) Update */
    // 버튼 디바운싱 및 LED 깜빡임 타이밍 계산 (Time-driven update)
    XM_IO_Update();
}

/* ---- [IPO Step 2] Algorithm Process (Process) ---- */
// IPO의 Process는 End User가 작성

/* ---- [IPO Step 3] Output Flushing (Tx Integration) ---- */
/**
 * @brief [IPO Step 3] Output Flushing
 * @details API 계층(Facade)에 기록된 제어 명령을 하드웨어 드라이버로 전달합니다.
 * 변경 사항이 있는 데이터만 선별적으로 Staging하여 버스 부하를 줄입니다.
 */
static void _FlushAllOutputs(void)
{
    /* ------------------------------------------------------
     * 1. Data Staging (값 업데이트)
     * ------------------------------------------------------ */
    // 사용자가 Process단계에서 SetAssistTorque를 호출해서 값이 바뀐 경우에만 드라이버 메모리 갱신
    if (XM.command._dirty_flags.torque_rh_updated) {
        CM_StageAuxTorque(SYS_NODE_ID_RH, XM.command.assist_torque_rh);
        XM.command._dirty_flags.torque_rh_updated = 0;
    }

    if (XM.command._dirty_flags.torque_lh_updated) {
        CM_StageAuxTorque(SYS_NODE_ID_LH, XM.command.assist_torque_lh);
        XM.command._dirty_flags.torque_lh_updated = 0;
    }

    /* ------------------------------------------------------
     * 2. Physical Transmission (Tx 정책)
     * ------------------------------------------------------ */
    // [정책] TORQUE 모드일 때만 주기적 전송 (Cyclic Transmission)
    if (XM.command.control_mode == XM_CTRL_TORQUE) {
        // 변경 사항이 없어도 이전에 Staging된 값(Zero-Order Hold)을 계속 전송함.
        // 이는 Watchdog에 걸리지 않고 "나 살아있어"라고 알리는 역할도 함.
        CM_FlushControlPDOs(); 
    }
    // MONITOR 모드일 때는 아무것도 전송하지 않음 (Silence)
}
