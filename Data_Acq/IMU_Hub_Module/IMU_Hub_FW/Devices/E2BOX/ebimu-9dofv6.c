/**
 ******************************************************************************
 * @file    ebimu-9dofv6.c
 * @author  HyundoKim
 * @brief   [Devices Layer] E2BOX EBIMU-9DOFV6 센서 드라이버 구현부
 * @details
 * - Binary 프로토콜 바이트 단위 파싱 (ISR-safe)
 * - 센서 설정 (ConfigureOutput) - IOIF UART 사용
 * - IMU Hub Module / XM 공통 사용 가능 (Device + IOIF 복사 이식)
 * 
 * [의존성] Device → IOIF (00-core-architecture.mdc 준수)
 * - ioif_agrb_uart.h: UART Tx (센서 명령 전송)
 * - ioif_agrb_tim.h: 딜레이 (센서 부팅/응답 대기)
 * 
 * @version 3.0 (Multi-Instance Auto-Sense + DataLake)
 * @date    Feb 11, 2026
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ebimu-9dofv6.h"
#include "ioif_agrb_tim.h"   /* IOIF_TIM_Delay */
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

// EBIMU Binary Protocol Definitions
// EBIMU_HEADER_BYTE → ebimu-9dofv6.h (public, 파서 레지스트리 sync[] 에서 참조)
#define PAYLOAD_SIZE        (20)    // Q(8) + Gyr(6) + Acc(6) = 20 bytes

#define SCALE_QUAT  (0.0001f)
#define SCALE_GYRO  (0.1f)
#define SCALE_ACC   (0.01f)

/** @brief UART 센서 타임아웃 (ms) - 1초간 데이터 없으면 끊김 */
#define EBIMU_TIMEOUT_MS    1000

/** @brief 명령 전송 최대 재시도 횟수 */
#define EBIMU_CMD_MAX_RETRY 5

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/* 파싱 (Pure C, ISR-safe) */
static void _InitParser(EbimuParser_t* parser);
static bool _ParseByte(EbimuParser_t* parser, uint8_t byte, EBIMU_Data_t* output);
static bool _IsConnected(uint8_t imu_id, uint32_t last_rx_time, uint32_t current_time);

static float _ConvertDataFloat16(const uint8_t* t_byte);

/* 센서 설정 (IOIF 의존) */
static void _ConfigureOutput(IOIF_UARTx_t uart_id);
static void _Cmd_SetOutputFormat(IOIF_UARTx_t uart_id);
static void _Cmd_SetOutputRate_1kHz(IOIF_UARTx_t uart_id);
static void _Cmd_CalibGyro(IOIF_UARTx_t uart_id);
static void _Cmd_CalibAccelSimple(IOIF_UARTx_t uart_id);
static void _Cmd_Reset(IOIF_UARTx_t uart_id);
static void _Cmd_FactoryReset(IOIF_UARTx_t uart_id);
static void _TxCmd(IOIF_UARTx_t uart_id, const char* cmd);

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/** @brief [디버깅] 센서 설정 명령 전송 통계 */
static volatile uint32_t s_config_cmd_sent = 0;
static volatile uint32_t s_config_cmd_failed = 0;

/**
 *------------------------------------------------------------
 * PUBLIC VARIABLES
 *------------------------------------------------------------
 */

EBIMU_Driver_t ebimu9dofv6 = {
    /* 파싱 */
    .InitParser           = _InitParser,
    .ParseByte            = _ParseByte,
    .IsConnected          = _IsConnected,
    /* 센서 설정 */
    .ConfigureOutput      = _ConfigureOutput,
    .Cmd_SetOutputFormat   = _Cmd_SetOutputFormat,
    .Cmd_SetOutputRate_1kHz = _Cmd_SetOutputRate_1kHz,
    .Cmd_CalibGyro        = _Cmd_CalibGyro,
    .Cmd_CalibAccelSimple  = _Cmd_CalibAccelSimple,
    .Cmd_Reset            = _Cmd_Reset,
    .Cmd_FactoryReset     = _Cmd_FactoryReset,
};

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief [디버깅] 센서 설정 명령 전송 통계 가져오기
 */
void EBIMU_GetConfigStats(volatile uint32_t* sent, volatile uint32_t* failed)
{
    if (sent) *sent = s_config_cmd_sent;
    if (failed) *failed = s_config_cmd_failed;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - 파싱 (Pure C, ISR-safe)
 *------------------------------------------------------------
 */

static void _InitParser(EbimuParser_t* parser)
{
    if (parser) {
        memset(parser, 0, sizeof(EbimuParser_t));
        parser->state = EBIMU_STATE_WAIT_HEADER_1;
    }
}

/**
 * @brief 바이트 단위 파싱 (ISR Context)
 * @details Header: 0x55 0x55, Data: Q(4) -> Gyr(3) -> Acc(3) (int16_t array)
 * @note timestamp, imu_index는 caller가 설정해야 함
 */
static bool _ParseByte(EbimuParser_t* parser, uint8_t byte, EBIMU_Data_t* output)
{
    bool completed = false;

    switch (parser->state) {
        case EBIMU_STATE_WAIT_HEADER_1:
            if (byte == EBIMU_HEADER_BYTE) {
                parser->state = EBIMU_STATE_WAIT_HEADER_2;
            }
            break;
            
        case EBIMU_STATE_WAIT_HEADER_2:
            if (byte == EBIMU_HEADER_BYTE) {
                parser->buf_idx = 0;
                parser->payload_len = PAYLOAD_SIZE;
                parser->state = EBIMU_STATE_READ_PAYLOAD;
            } else {
                parser->state = EBIMU_STATE_WAIT_HEADER_1;
            }
            break;

        case EBIMU_STATE_READ_PAYLOAD:
            parser->buffer[parser->buf_idx++] = byte;

            if (parser->buf_idx >= parser->payload_len) {
                /* --- 패킷 완성! 데이터 변환 시작 --- */
                uint8_t* p = parser->buffer;

                // 1. Quaternion (Order: Z, Y, X, W based on E2BOX protocol)
                output->q_z = _ConvertDataFloat16(p) * SCALE_QUAT; p += 2;
                output->q_y = _ConvertDataFloat16(p) * SCALE_QUAT; p += 2;
                output->q_x = _ConvertDataFloat16(p) * SCALE_QUAT; p += 2;
                output->q_w = _ConvertDataFloat16(p) * SCALE_QUAT; p += 2;
                
                // 2. Gyroscope (X, Y, Z)
                output->gyr_x = _ConvertDataFloat16(p) * SCALE_GYRO; p += 2;
                output->gyr_y = _ConvertDataFloat16(p) * SCALE_GYRO; p += 2;
                output->gyr_z = _ConvertDataFloat16(p) * SCALE_GYRO; p += 2;

                // 3. Accelerometer (X, Y, Z)
                output->acc_x = _ConvertDataFloat16(p) * SCALE_ACC; p += 2;
                output->acc_y = _ConvertDataFloat16(p) * SCALE_ACC; p += 2;
                output->acc_z = _ConvertDataFloat16(p) * SCALE_ACC; p += 2;
                
                // 4. 상태 리셋 및 완료 플래그 설정
                parser->state = EBIMU_STATE_WAIT_HEADER_1;
                completed = true;
            }
            break;
    }
    return completed;
}

/**
 * @brief 연결 상태 확인 (타임아웃 기반)
 */
static bool _IsConnected(uint8_t imu_id, uint32_t last_rx_time, uint32_t current_time)
{
    (void)imu_id;
    
    uint32_t elapsed = current_time - last_rx_time;
    return (elapsed < EBIMU_TIMEOUT_MS);
}

/**
 * @brief Converts two 8-bit bytes (Big-Endian) into a float.
 */
static inline float _ConvertDataFloat16(const uint8_t* t_byte)
{
    uint16_t value_u16 = (uint16_t)t_byte[0] << 8 | (uint16_t)t_byte[1];
    int16_t value_s16 = (int16_t)value_u16;
    return (float)value_s16;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - 센서 설정 (IOIF 의존)
 *------------------------------------------------------------
 */

/**
 * @brief EBIMU 센서를 Binary 1kHz 모드로 설정
 * @param uart_id IOIF UART 인스턴스 ID
 * @details
 * [시퀀스]
 * 1. 센서 부팅 대기 (200ms)
 * 2. Binary Mode + Quat + Gyro + Acc 설정
 * 3. 1kHz Output Rate 설정
 */
static void _ConfigureOutput(IOIF_UARTx_t uart_id)
{
    /* 센서 부팅 대기 */
    IOIF_TIM_Delay(200);
    
    /* Binary Mode + Quat + Gyro + Acc */
    _Cmd_SetOutputFormat(uart_id);
    IOIF_TIM_Delay(50);
    
    /* 1kHz Output Rate */
    _Cmd_SetOutputRate_1kHz(uart_id);
    IOIF_TIM_Delay(50);
}

static void _Cmd_SetOutputFormat(IOIF_UARTx_t uart_id)
{
    _TxCmd(uart_id, "<sof2><soq1><sog1><soa1>");
}

static void _Cmd_SetOutputRate_1kHz(IOIF_UARTx_t uart_id)
{
    _TxCmd(uart_id, "<sor1>");
}

static void _Cmd_CalibGyro(IOIF_UARTx_t uart_id)
{
    _TxCmd(uart_id, "<cg>");
}

static void _Cmd_CalibAccelSimple(IOIF_UARTx_t uart_id)
{
    _TxCmd(uart_id, "<cas>");
}

static void _Cmd_Reset(IOIF_UARTx_t uart_id)
{
    _TxCmd(uart_id, "<rst>");
}

static void _Cmd_FactoryReset(IOIF_UARTx_t uart_id)
{
    _TxCmd(uart_id, "<frt>");
}

/**
 * @brief EBIMU ASCII 명령 전송 (재시도 포함)
 * @param uart_id IOIF UART 인스턴스 ID
 * @param cmd ASCII 명령 문자열
 */
static void _TxCmd(IOIF_UARTx_t uart_id, const char* cmd)
{
    uint32_t retry = 0;
    bool success = false;
    
    while (retry < EBIMU_CMD_MAX_RETRY) {
        AGRBStatusDef result = IOIF_UART_Write_Polling(uart_id, (uint8_t*)cmd, strlen(cmd));
        if (result == AGRBStatus_OK) {
            success = true;
            break;
        }
        
        IOIF_TIM_Delay(10);
        retry++;
    }
    
    if (success) {
        s_config_cmd_sent++;
    } else {
        s_config_cmd_failed++;
    }
    
    IOIF_TIM_Delay(20);
}

/* ================================================================
 * Auto-Sense + DataLake 구현 (Multi-Instance)
 * ================================================================
 * mti-630.c와 동일한 패턴으로 구현.
 * - DataLake: Mutex + Snapshot (Task-Task 표준 패턴)
 * - Auto-Sense: 데이터 타임아웃 기반 STOPPED ↔ OPERATIONAL 전환
 * - Multi-Instance: ch 파라미터로 인스턴스 구분
 *   XM: ch=0 (EBIMU_MAX_INSTANCES=1, 사용 시)
 *   IMU Hub: ch=0~5 (EBIMU_MAX_INSTANCES=6)
 *
 * 동기화 패턴 (cursorrule 13-comm-core-patterns):
 *   Task→Task, Latest Data Only → Mutex + Snapshot
 *   ❌ Double Buffer (불필요한 Lock-Free 복잡성)
 *   ❌ Queue (최신 값만 필요, 큐 관리 오버헤드)
 * ================================================================ */

#if defined(USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

/* ===== Auto-Sense 상태 ===== */
#define EBIMU_AUTO_SENSE_TIMEOUT_MS  (500U)

typedef enum {
    EBIMU_AUTO_STATE_STOPPED,
    EBIMU_AUTO_STATE_OPERATIONAL,
} EbimuAutoSenseState_t;

/* DataLake: 단일 버퍼 (인스턴스별 배열)
 * volatile: UART ISR(writer)과 Timer ISR(reader) 공유 → 컴파일러 캐시 방지.
 * [R6 Seqlock] BareMetal 에서 writer=UART RX ISR(NVIC prio0) > reader=Timer ISR(prio1)
 *   이라 reader 가 writer 에게 선점당해 구조체 대입(field-by-field, non-atomic)이
 *   torn-read 될 수 있다. seq 카운터로 보호: writer 가 seq 홀수(쓰기중)→짝수(완료),
 *   reader 는 전후 seq 불일치/홀수면 재시도. (RTOS 경로는 Mutex+Snapshot 유지.) */
static volatile EBIMU_Data_t  s_ebimu_datalake[EBIMU_MAX_INSTANCES];
#if !defined(USE_FREERTOS)
#define EBIMU_SEQLOCK_MAX_RETRY  4U   /**< writer 가 매우 짧아 0~1 회면 충분, cap=종료보장 */
static volatile uint32_t s_ebimu_seq[EBIMU_MAX_INSTANCES];  /**< seqlock: 홀수=쓰기중 */
#endif

#if defined(USE_FREERTOS)
static SemaphoreHandle_t  s_ebimu_mutex[EBIMU_MAX_INSTANCES];
#endif

/* Auto-Sense 상태 (인스턴스별 배열) */
static volatile EbimuAutoSenseState_t s_ebimu_auto_sense_state[EBIMU_MAX_INSTANCES];
static volatile uint32_t              s_ebimu_last_packet_tick[EBIMU_MAX_INSTANCES];

/* ===== Public Functions ===== */

void EbimuV6_StateInit(uint8_t ch)
{
    if (ch >= EBIMU_MAX_INSTANCES) return;

    memset((void*)&s_ebimu_datalake[ch], 0, sizeof(s_ebimu_datalake[ch]));
#if !defined(USE_FREERTOS)
    s_ebimu_seq[ch] = 0;
#endif
    s_ebimu_auto_sense_state[ch] = EBIMU_AUTO_STATE_STOPPED;
    s_ebimu_last_packet_tick[ch] = 0;

#if defined(USE_FREERTOS)
    if (s_ebimu_mutex[ch] == NULL) {
        s_ebimu_mutex[ch] = xSemaphoreCreateMutex();
    }
#endif
}

void EbimuV6_UpdateData(uint8_t ch, const EBIMU_Data_t* packet)
{
    if (ch >= EBIMU_MAX_INSTANCES) return;
    if (packet == NULL) return;

#if defined(USE_FREERTOS)
    /* Mutex + DataLake: Writer (UartRxTask) */
    xSemaphoreTake(s_ebimu_mutex[ch], portMAX_DELAY);
    s_ebimu_datalake[ch] = *packet;
    xSemaphoreGive(s_ebimu_mutex[ch]);
#else
    /* BareMetal seqlock writer (R6): seq 홀수→데이터→짝수. writer=UART RX ISR(prio0),
     * reader=Timer ISR(prio1, 하위)라 writer 는 reader 대비 원자적(선점당하지 않음). */
    s_ebimu_seq[ch]++;             /* 홀수 = 쓰기 시작 */
    __DMB();
    s_ebimu_datalake[ch] = *packet;
    __DMB();
    s_ebimu_seq[ch]++;             /* 짝수 = 쓰기 완료 */
#endif

    /* Auto-Sense: 데이터 수신 → OPERATIONAL */
    s_ebimu_last_packet_tick[ch] = IOIF_TIM_GetTick();
    s_ebimu_auto_sense_state[ch] = EBIMU_AUTO_STATE_OPERATIONAL;
}

bool EbimuV6_GetLatest(uint8_t ch, EBIMU_Data_t* out)
{
    if (ch >= EBIMU_MAX_INSTANCES) return false;
    if (out == NULL) return false;
    if (s_ebimu_auto_sense_state[ch] != EBIMU_AUTO_STATE_OPERATIONAL) return false;

#if defined(USE_FREERTOS)
    /* Mutex + Snapshot: Reader (Core Process) */
    xSemaphoreTake(s_ebimu_mutex[ch], portMAX_DELAY);
    *out = s_ebimu_datalake[ch];
    xSemaphoreGive(s_ebimu_mutex[ch]);
#else
    /* BareMetal seqlock reader (R6): writer(UART ISR prio0)가 선점해 torn-read 시
     * seq 전후 불일치/홀수 → 재시도. writer 가 매우 짧아 재시도 0~1 회.
     * cap 소진(사실상 불가) 시 false → 호출부(FetchAllImus)가 해당 ch 0 처리. */
    {
        uint32_t s1, s2;
        uint8_t  retry = 0;
        do {
            s1 = s_ebimu_seq[ch];
            __DMB();
            *out = s_ebimu_datalake[ch];
            __DMB();
            s2 = s_ebimu_seq[ch];
        } while ((s1 != s2 || (s1 & 1U)) && (++retry < EBIMU_SEQLOCK_MAX_RETRY));
        if (s1 != s2 || (s1 & 1U)) {
            return false;   /* 재시도 소진 → 이번 tick 갱신 스킵 */
        }
    }
#endif

    return true;
}

bool EbimuV6_IsOnline(uint8_t ch)
{
    if (ch >= EBIMU_MAX_INSTANCES) return false;
    return (s_ebimu_auto_sense_state[ch] == EBIMU_AUTO_STATE_OPERATIONAL);
}

void EbimuV6_RunPeriodic(uint8_t ch)
{
    if (ch >= EBIMU_MAX_INSTANCES) return;

    if (s_ebimu_auto_sense_state[ch] == EBIMU_AUTO_STATE_OPERATIONAL) {
        if (IOIF_TIM_GetTick() - s_ebimu_last_packet_tick[ch] > EBIMU_AUTO_SENSE_TIMEOUT_MS) {
            s_ebimu_auto_sense_state[ch] = EBIMU_AUTO_STATE_STOPPED;
        }
    }
}
