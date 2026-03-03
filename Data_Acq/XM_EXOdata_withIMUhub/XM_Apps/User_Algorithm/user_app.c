/**
 ******************************************************************************
 * @file    user_app.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Nov 18, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api.h"      // 통합 API 헤더

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

typedef enum {
    SmartAssist_OFF,
    SmartAssist_ON
} SmartAssist_t;

/**
 * @brief USB CDC로 전송할 통합 데이터 구조체 (H10 + IMU Hub)
 */
typedef struct __attribute__((packed)) {
    uint16_t sof;      // Start-of-frame: 0xAA55
    uint16_t len;      // 전체 패킷 길이 (sizeof(UsbStreamData_t))
    
    // 기본 정보
    uint32_t loopCnt;
    uint8_t  h10Mode;
    uint8_t  h10AssistLevel;
    uint8_t  imuHubConnected;  // IMU Hub 연결 상태 (0 or 1)
    uint8_t  sync_signal;      // XM_EXT_DIO_3 (PF5) GPIO 상태 (0=LOW, 1=HIGH)
    
    // H10 로봇 데이터
    float leftHipAngle, rightHipAngle;
    float leftThighAngle, rightThighAngle;
    float leftHipTorque, rightHipTorque;
    float leftHipMotorAngle, rightHipMotorAngle;
    
    // H10 내장 IMU (Left/Right Hip)
    float leftHipImuGlobalAccX, leftHipImuGlobalAccY, leftHipImuGlobalAccZ;
    float leftHipImuGlobalGyrX, leftHipImuGlobalGyrY, leftHipImuGlobalGyrZ;
    float rightHipImuGlobalAccX, rightHipImuGlobalAccY, rightHipImuGlobalAccZ;
    float rightHipImuGlobalGyrX, rightHipImuGlobalGyrY, rightHipImuGlobalGyrZ;
    
    // Trunk IMU (XSENS)
    float trunkImuLocalAccX, trunkImuLocalAccY, trunkImuLocalAccZ;
    float trunkImuLocalGyrX, trunkImuLocalGyrY, trunkImuLocalGyrZ;
    float trunkImuQuatW, trunkImuQuatX, trunkImuQuatY, trunkImuQuatZ;
    
    // IMU Hub Module - 6개 IMU 센서 (각 10개 float = 60 floats)
    struct {
        float q_w, q_x, q_y, q_z;      // Quaternion
        float acc_x, acc_y, acc_z;     // Acceleration (g)
        float gyr_x, gyr_y, gyr_z;     // Gyroscope (deg/s)
    } imuHub[6];
    
    uint16_t crc;      // CRC16 (sof ~ crc 직전까지)
} UsbStreamData_t;

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* --- 전역 변수 및 핸들 선언 --- */
static XmTsmHandle_t s_user_tsm;

/* --- USB 전송용 데이터 버퍼 --- */
static UsbStreamData_t s_usbData;  // XM_SendUsbData()가 내부 Ring Buffer로 복사
static uint32_t s_loopCount = 0;

/* --- SmartAssist 관련 변수 --- */
static SmartAssist_t SmartAssist = SmartAssist_OFF;
static XmBtnEvent_t XM_button1 = XM_BTN_NONE;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void DataCollection_Loop(void);
static void UpdateSmartAssistState(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* =================================================================
 * [필수 구현 1] 초기화 함수 예시 (User_Setup)
 * - 전원 인가 후 딱 한 번 실행됩니다.
 * - TSM 생성, 변수 초기화, LED/Button 초기 설정 등을 수행합니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
bool xsensIMUenableRes = false;

void User_Setup(void)
{
    // 1. TSM 생성 (단일 상태)
    s_user_tsm = XM_TSM_Create(XM_STATE_USER_START);
    
    // 2. 데이터 수집 상태 등록
    XmStateConfig_t conf = {
        .id = XM_STATE_USER_START,
        .on_loop = DataCollection_Loop
    };
    XM_TSM_AddState(s_user_tsm, &conf);
    
    // 3. 외부 IMU(Xsens MTi) 활성화 (PA0, PA1 → UART4 전환)
    if (XM_EnableExternalImu()) {
    	xsensIMUenableRes = true;
        // IMU 활성화 성공 - XM.status.imu.acc_x/gyr_x 등 사용 가능
    } else {
        // IMU 활성화 실패
    }
    
    // 3.5. Sync Signal GPIO 초기화 (PF5, XM_EXT_DIO_3)
    XM_SetPinMode(XM_EXT_DIO_3, XM_EXT_DIO_MODE_INPUT_PULLDOWN);
    
    // 4. 패킷 헤더 초기화
    s_usbData.sof = 0xAA55;
    s_usbData.len = sizeof(UsbStreamData_t);
    
    // 5. USB CDC 스트리밍 데이터 소스 등록 ("AGRB MON START" 명령용)
    XM_SetUsbStreamSource(&s_usbData, sizeof(UsbStreamData_t));
}

/* =================================================================
 * [필수 구현 2] 반복 루프 함수 (User_Loop)
 * - 2ms(500Hz) 주기로 계속 호출됩니다.
 * - 내부 IPO(Input-Process-Output) 모델이 적용되어 있습니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
void User_Loop(void)
{
    XM_TSM_Run(s_user_tsm);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief 데이터 수집 및 USB 전송용 버퍼 업데이트 (2ms마다 호출)
 */
static void DataCollection_Loop(void)
{
    // Update SmartAssist based on button input
    UpdateSmartAssistState();
    
    // 1. 기본 정보 업데이트
    s_loopCount++;
    s_usbData.loopCnt = s_loopCount;
    s_usbData.h10Mode = (uint8_t)XM.command.control_mode;
    s_usbData.h10AssistLevel = 0;
    s_usbData.imuHubConnected = XM.status.imu_hub.is_connected ? 1 : 0;
    s_usbData.sync_signal = (uint8_t)XM_DigitalRead(XM_EXT_DIO_3);  // Read GPIO state (0 or 1)
    
    // 2. H10 로봇 데이터
    s_usbData.leftHipAngle = XM.status.h10.leftHipAngle;
    s_usbData.rightHipAngle = XM.status.h10.rightHipAngle;
    s_usbData.leftThighAngle = XM.status.h10.leftThighAngle;
    s_usbData.rightThighAngle = XM.status.h10.rightThighAngle;
    s_usbData.leftHipTorque = XM.status.h10.leftHipTorque;
    s_usbData.rightHipTorque = XM.status.h10.rightHipTorque;
    s_usbData.leftHipMotorAngle = XM.status.h10.leftHipMotorAngle;
    s_usbData.rightHipMotorAngle = XM.status.h10.rightHipMotorAngle;
    
    // 3. H10 내장 IMU (Left/Right Hip)
    s_usbData.leftHipImuGlobalAccX = XM.status.h10.leftHipImuGlobalAccX;
    s_usbData.leftHipImuGlobalAccY = XM.status.h10.leftHipImuGlobalAccY;
    s_usbData.leftHipImuGlobalAccZ = XM.status.h10.leftHipImuGlobalAccZ;
    s_usbData.leftHipImuGlobalGyrX = XM.status.h10.leftHipImuGlobalGyrX;
    s_usbData.leftHipImuGlobalGyrY = XM.status.h10.leftHipImuGlobalGyrY;
    s_usbData.leftHipImuGlobalGyrZ = XM.status.h10.leftHipImuGlobalGyrZ;
    
    s_usbData.rightHipImuGlobalAccX = XM.status.h10.rightHipImuGlobalAccX;
    s_usbData.rightHipImuGlobalAccY = XM.status.h10.rightHipImuGlobalAccY;
    s_usbData.rightHipImuGlobalAccZ = XM.status.h10.rightHipImuGlobalAccZ;
    s_usbData.rightHipImuGlobalGyrX = XM.status.h10.rightHipImuGlobalGyrX;
    s_usbData.rightHipImuGlobalGyrY = XM.status.h10.rightHipImuGlobalGyrY;
    s_usbData.rightHipImuGlobalGyrZ = XM.status.h10.rightHipImuGlobalGyrZ;
    
    // 4. Trunk IMU (XSENS)
    s_usbData.trunkImuLocalAccX = XM.status.imu.acc_x;
    s_usbData.trunkImuLocalAccY = XM.status.imu.acc_y;
    s_usbData.trunkImuLocalAccZ = XM.status.imu.acc_z;
    s_usbData.trunkImuLocalGyrX = XM.status.imu.gyr_x;
    s_usbData.trunkImuLocalGyrY = XM.status.imu.gyr_y;
    s_usbData.trunkImuLocalGyrZ = XM.status.imu.gyr_z;
    s_usbData.trunkImuQuatW = XM.status.imu.q_w;
    s_usbData.trunkImuQuatX = XM.status.imu.q_x;
    s_usbData.trunkImuQuatY = XM.status.imu.q_y;
    s_usbData.trunkImuQuatZ = XM.status.imu.q_z;
    
    // 5. IMU Hub Module - 6개 IMU 센서 데이터
    for (int i = 0; i < 6; i++) {
        XmImuHubSensor_t* src = &XM.status.imu_hub.sensor[i];
        
        s_usbData.imuHub[i].q_w = src->q_w;
        s_usbData.imuHub[i].q_x = src->q_x;
        s_usbData.imuHub[i].q_y = src->q_y;
        s_usbData.imuHub[i].q_z = src->q_z;
        
        s_usbData.imuHub[i].acc_x = src->acc_x;
        s_usbData.imuHub[i].acc_y = src->acc_y;
        s_usbData.imuHub[i].acc_z = src->acc_z;
        
        s_usbData.imuHub[i].gyr_x = src->gyr_x;
        s_usbData.imuHub[i].gyr_y = src->gyr_y;
        s_usbData.imuHub[i].gyr_z = src->gyr_z;
    }
    
    // 6. CRC 계산
    s_usbData.crc = 0;  // TODO: CRC16 계산 필요시 구현
}

static void UpdateSmartAssistState(void)
{
//    // 1) H10 모드가 STANDBY이면 무조건 SmartAssist OFF
//    if (XM.status.h10.h10Mode == XM_H10_MODE_STANDBY) {
//        if (SmartAssist != SmartAssist_OFF) {
//            SmartAssist = SmartAssist_OFF;
//            XM_SetH10AssistExistingMode(false);
//        }
//
//        // STANDBY 상태에서는 BTN1 이벤트를 읽어서 그냥 버린다.
//        // (STANDBY에서 누른 클릭이 나중에 ACTIVE 모드에서 반영되지 않도록)
//        (void)XM_GetButtonEvent(XM_BTN_1);
//
//        return;
//    }

    // 2) 버튼 입력 처리 (BTN1)
    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    if (XM_button1 != XM_BTN_CLICK) {
        return;
    }

    // 3) SmartAssist 토글
    if (SmartAssist == SmartAssist_ON) {
        SmartAssist = SmartAssist_OFF;
        XM_SetH10AssistExistingMode(false);
    } else {
        SmartAssist = SmartAssist_ON;
        XM_SetH10AssistExistingMode(true);
    }
}

