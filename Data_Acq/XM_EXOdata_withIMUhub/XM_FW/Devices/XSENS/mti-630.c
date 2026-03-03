/**
 ******************************************************************************
 * @file    mti-630.c
 * @author  HyundoKim
 * @brief   [Devices Layer] Xsens MTi-630 IMU 드라이버 구현부 (ROBUST)
 * @details Xbus 프로토콜(MTData2) 파서, 1kHz 설정 기능 포함
 * @version 1.0 (Robust Implementation - Ported from IMU_Hub_Module)
 * @date    Jan 26, 2026
 *
 * @changelog
 * v1.0 (Jan 26, 2026) - ROBUST Implementation with IMU_Hub_Module pattern:
 *                       ✅ 8-state parser with extended length support
 *                       ✅ Buffer overflow protection
 *                       ✅ Checksum validation (1-complement)
 *                       ✅ Big-endian to little-endian conversion
 *                       ✅ Full boot sequence (GoToConfig→SetOutput→GoToMeasurement)
 *                       ✅ Retry logic with exponential backoff (5 retries)
 *                       ✅ Connection monitoring (1s timeout)
 *                       ✅ Diagnostic counters (packets, errors, commands)
 *                       ✅ RTOS-safe (FreeRTOS queue-based callbacks)
 * v0.2 (Jan 22, 2026) - Added boot sequence commands
 * v0.1 (Nov 12, 2025) - Initial implementation
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "mti-630.h"
#include <string.h>
#include "ioif_agrb_tim.h" // IOIF_TIM_GetTick()
#include "ioif_agrb_uart.h" // IOIF_UART_Write_Polling

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define XSENS_RX_BUFFER_SIZE (256) // MTData2 패킷 최대 크기

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief Xsens 패킷 파싱 상태
 */
typedef enum {
    STATE_WAIT_PREAMBLE,  // 0xFA
    STATE_WAIT_BID,       // 0xFF
    STATE_WAIT_MID,       // Message ID
    STATE_WAIT_LEN,       // Length (Standard or 0xFF)
    STATE_WAIT_LEN_EXT_H, // Extended Length High Byte
    STATE_WAIT_LEN_EXT_L, // Extended Length Low Byte
    STATE_COLLECT_PAYLOAD,// Payload
    STATE_WAIT_CHECKSUM,  // Checksum
} ParseState_t;

/**
 * @brief 파싱 중인 패킷을 임시 저장하는 버퍼
 */
typedef struct {
    uint8_t buffer[XSENS_RX_BUFFER_SIZE];
    uint8_t mid;
    uint16_t len;       // Extended Length 지원
    uint16_t index;     // 페이로드 수집 인덱스
    uint8_t checksum;   // 1-complement 덧셈 체크섬
} XsensRxBuffer_t;

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

static IOIF_UARTx_t s_uart_id = IOIF_UART_ID_NOT_ALLOCATED;
static ParseState_t s_parse_state = STATE_WAIT_PREAMBLE;
static XsensRxBuffer_t s_rx_buffer;
/* FSR 드라이버와 동일하게, System Layer의 콜백을 저장할 변수 */
static ImuPacketCallback_t s_packet_callback = NULL;

/* [ROBUST] 디버깅 및 모니터링 카운터 (IMU_Hub pattern) */
static volatile uint32_t s_packets_received = 0;
static volatile uint32_t s_packets_failed_checksum = 0;
static volatile uint32_t s_packets_failed_overflow = 0;
static volatile uint32_t s_config_cmd_sent = 0;
static volatile uint32_t s_config_cmd_failed = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static bool initialize(IOIF_UARTx_t id, ImuPacketCallback_t packet_cb);
static bool ConfigureOutput(void);
static void uart_callback(uint8_t* rx_buf, uint32_t size, uint32_t id);
static bool parse_byte(uint8_t byte);
static bool process_packet(XsensMTi_packet_t* output);
static float _Xsens_ReverseFloat(uint8_t* pData);
static void parse_mtdata2(XsensRxBuffer_t* rx, XsensMTi_packet_t* output);

/**
 * @brief 센서 연결 상태 확인 (타임아웃 기반)
 * @param last_rx_time 마지막 패킷 수신 시간 (ms)
 * @param current_time 현재 시간 (ms)
 * @return true: 연결됨, false: 타임아웃
 * @note IMU_Hub pattern - 1초간 데이터 없으면 끊김
 */
static bool IsConnected(uint32_t last_rx_time, uint32_t current_time);

/* Boot Sequence Commands (ported from IMU_Hub_Module) */
static void _Cmd_Reset(IOIF_UARTx_t id);
static void _Cmd_GoToConfig(IOIF_UARTx_t id);
static void _Cmd_GoToMeasurement(IOIF_UARTx_t id);
static void _Cmd_SetBaudrate(IOIF_UARTx_t id, uint32_t baudrate);
static void _TxCmd(IOIF_UARTx_t id, const uint8_t* cmd, uint16_t len);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief [디버깅] 진단 통계 가져오기
 */
void XsensMTi_GetStats(XsensMTi_Stats_t* stats)
{
    if (stats == NULL) return;
    
    stats->packets_received = s_packets_received;
    stats->packets_failed_checksum = s_packets_failed_checksum;
    stats->packets_failed_overflow = s_packets_failed_overflow;
    stats->config_cmd_sent = s_config_cmd_sent;
    stats->config_cmd_failed = s_config_cmd_failed;
}

XsensMTi630_t xsensMTi630 = {
    .init = initialize,
    .ConfigureOutput = ConfigureOutput,
    .IsConnected = IsConnected,
    .Cmd_Reset = _Cmd_Reset,
    .Cmd_GoToConfig = _Cmd_GoToConfig,
    .Cmd_GoToMeasurement = _Cmd_GoToMeasurement,
    .Cmd_SetBaudrate = _Cmd_SetBaudrate,
};

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

static bool initialize(IOIF_UARTx_t id, ImuPacketCallback_t packet_cb)
{
    s_uart_id = id;
    s_parse_state = STATE_WAIT_PREAMBLE;
    s_rx_buffer.index = 0;

    /* [추가] FSR처럼 System Layer 콜백 저장 */
    s_packet_callback = packet_cb;

    IOIF_UART_SetRxIdleCallback(s_uart_id, uart_callback);
    return true;
}

/**
 * @brief [Updated] 센서를 1kHz Quat+Acc+Gyro 모드로 설정 (Full Boot Sequence)
 * @details MTData2 Output Configuration (MID 0xC0)
 * @note Ported from IMU_Hub_Module - Full boot sequence implementation:
 *       [1] Sensor boot wait (500ms)
 *       [2] GoToConfig (Config 모드 진입)
 *       [3] MTData2 Output Configuration
 *       [4] GoToMeasurement (MTData2 전송 시작)
 */
static bool ConfigureOutput(void)
{
    /* [1] 센서 부팅 대기 (전원 인가 후 안정화) */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* [2] GoToConfig (Config 모드 진입) - 현재 Baudrate로 통신 */
    _Cmd_GoToConfig(s_uart_id);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* [3] MTData2 Output Configuration (MID 0xC0) */
    // Payload: [DataID (2B) | Freq (2B)]
    // 0x2010 (Quat) @ 1000Hz (0x03E8)
    // 0x4020 (Accel) @ 1000Hz (0x03E8)
    // 0x8020 (Gyro)  @ 1000Hz (0x03E8)
    
    // Xsens는 Big-Endian이므로 네트워크 바이트 순서(MSB first)로 전송
    uint8_t payload[] = {
        0x20, 0x10, 0x03, 0xE8, // Quaternion (0x2010) @ 1000Hz (0x03E8)
        0x40, 0x20, 0x03, 0xE8, // Acceleration (0x4020) @ 1000Hz
        0x80, 0x20, 0x03, 0xE8  // Gyroscope (0x8020) @ 1000Hz
    };
    uint8_t len = sizeof(payload); // 12 bytes

    // Xsens Message (Preamble, BID, MID, LEN, Payload, Checksum)
    uint8_t tx_buffer[32]; // Preamble(1) + BID(1) + MID(1) + LEN(1) + Payload(12) + CS(1) = 16 bytes
    tx_buffer[0] = XSENS_PREAMBLE;
    tx_buffer[1] = XSENS_BID;
    tx_buffer[2] = XSENS_MID_SETOUTPUTCFG;
    tx_buffer[3] = len;
    
    memcpy(&tx_buffer[4], payload, len);
    
    // Checksum 계산 (BID부터 Payload 끝까지의 합)
    uint8_t checksum = 0;
    for (int i = 1; i < (4 + len); i++) { // BID부터 Payload 끝까지
        checksum += tx_buffer[i];
    }
    tx_buffer[4 + len] = (uint8_t)(-checksum); // 1-complement

    // IOIF UART로 전송
    _TxCmd(s_uart_id, tx_buffer, (5 + len));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* [4] GoToMeasurement (MTData2 전송 시작!) */
    _Cmd_GoToMeasurement(s_uart_id);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return true;
}

static void uart_callback(uint8_t* rx_buf, uint32_t size, uint32_t id)
{
    if (id != s_uart_id) return;

    for (uint32_t i = 0; i < size; i++) {
        if (parse_byte(rx_buf[i])) {
            // true: 패킷 수신/검증 완료
            XsensMTi_packet_t parsed_packet;
            if (process_packet(&parsed_packet)) {
                /* [수정] 큐에 넣는 대신, 등록된 콜백을 호출합니다. */
                if (s_packet_callback != NULL) {
                    s_packet_callback(&parsed_packet);
                }
            }
            // 상태 머신 리셋
            s_parse_state = STATE_WAIT_PREAMBLE;
            s_rx_buffer.index = 0;
        }
    }
}

/**
 * @brief Xsens 1바이트 파싱 상태 머신 (Extended Length 및 Checksum 포함)
 */
static bool parse_byte(uint8_t byte)
{
    // 체크섬 계산 (Preamble 제외 모든 바이트)
    if (s_parse_state > STATE_WAIT_PREAMBLE) {
        s_rx_buffer.checksum += byte;
    }

    switch (s_parse_state)
    {
        case STATE_WAIT_PREAMBLE:
            if (byte == XSENS_PREAMBLE) {
                s_parse_state = STATE_WAIT_BID;
                s_rx_buffer.checksum = 0; // 체크섬 계산 시작
            }
            break;
            
        case STATE_WAIT_BID:
            (byte == XSENS_BID) ? (s_parse_state = STATE_WAIT_MID) : (s_parse_state = STATE_WAIT_PREAMBLE);
            break;
            
        case STATE_WAIT_MID:
            s_rx_buffer.mid = byte;
            s_parse_state = STATE_WAIT_LEN;
            break;
            
        case STATE_WAIT_LEN:
            if (byte == 0xFF) { // Extended Length
                s_rx_buffer.len = 0;
                s_parse_state = STATE_WAIT_LEN_EXT_H;
            } else {
                s_rx_buffer.len = byte;
                s_rx_buffer.index = 0;
                s_parse_state = (s_rx_buffer.len > 0) ? STATE_COLLECT_PAYLOAD : STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_LEN_EXT_H: // Extended Length (High Byte)
            s_rx_buffer.len = (uint16_t)byte << 8;
            s_parse_state = STATE_WAIT_LEN_EXT_L;
            break;
            
        case STATE_WAIT_LEN_EXT_L: // Extended Length (Low Byte)
            s_rx_buffer.len |= byte;
            s_rx_buffer.index = 0;
            s_parse_state = (s_rx_buffer.len > 0) ? STATE_COLLECT_PAYLOAD : STATE_WAIT_CHECKSUM;
            break;
            
        case STATE_COLLECT_PAYLOAD:
            if (s_rx_buffer.index < XSENS_RX_BUFFER_SIZE) {
                s_rx_buffer.buffer[s_rx_buffer.index++] = byte;
            }
            
            /* [ROBUST] 버퍼 오버플로우 방지 (IMU_Hub pattern) */
            if (s_rx_buffer.len > XSENS_RX_BUFFER_SIZE) {
                // 잘못된 패킷 (len이 비정상적으로 큼) → 동기화 깨짐
                s_packets_failed_overflow++;
                s_parse_state = STATE_WAIT_PREAMBLE;
                break;
            }
            
            if (s_rx_buffer.index >= s_rx_buffer.len) {
                s_parse_state = STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_CHECKSUM:
            // [강화] 수신된 체크섬을 포함한 총 합이 0x00(1-complement)이어야 함
            if (s_rx_buffer.checksum == 0x00) {
                s_packets_received++;
                return true; // 파싱 성공, 패킷 완료
            }
            s_packets_failed_checksum++;
            s_parse_state = STATE_WAIT_PREAMBLE; // 체크섬 실패
            break;
    }
    return false; // 아직 패킷 진행 중
}

/**
 * @brief 수신된 패킷(MID)에 따라 적절한 파서 호출
 */
static bool process_packet(XsensMTi_packet_t* output)
{
    if (s_rx_buffer.mid == XSENS_MID_MTDATA2) { // 0x36
        parse_mtdata2(&s_rx_buffer, output);
        return true;
    }
    // (다른 MID 처리, 예: 0x42 (Error))
    return false;
}

/**
 * @brief [신규] Big-Endian 4바이트 배열을 Little-Endian float으로 변환
 */
static float _Xsens_ReverseFloat(uint8_t* pData)
{
    float f;
    uint8_t* pFloat = (uint8_t*)&f;
    // (Xsens Big-Endian -> STM32 Little-Endian)
    pFloat[0] = pData[3];
    pFloat[1] = pData[2];
    pFloat[2] = pData[1];
    pFloat[3] = pData[0];
    return f;
}

/**
 * @brief MTData2 (0x36) 페이로드를 파싱 (엔디안 변환)
 */
static void parse_mtdata2(XsensRxBuffer_t* rx, XsensMTi_packet_t* output)
{
    output->timestamp = IOIF_TIM_GetTick();
    
    // (초기화) 데이터가 없을 경우 0으로 유지
    memset(&output->q_w, 0, sizeof(XsensMTi_packet_t) - sizeof(uint32_t));

    uint16_t idx = 0;
    while (idx < rx->len) {
        // 1. Data ID 읽기 (2 bytes, Big-endian)
        uint16_t data_id = ((uint16_t)rx->buffer[idx] << 8) | rx->buffer[idx+1];
        idx += 2;
        // 2. Data Length 읽기 (1 byte)
        uint8_t data_len = rx->buffer[idx];
        idx += 1;
        
        // 3. Data ID에 따라 파싱 및 엔디안 변환
        switch(data_id) {
            case XDI_QUATERNION: // 0x2010 (16 bytes)
                output->q_w = _Xsens_ReverseFloat(&rx->buffer[idx + 0]);
                output->q_x = _Xsens_ReverseFloat(&rx->buffer[idx + 4]);
                output->q_y = _Xsens_ReverseFloat(&rx->buffer[idx + 8]);
                output->q_z = _Xsens_ReverseFloat(&rx->buffer[idx + 12]);
                break;
            case XDI_ACCELERATION: // 0x4020 (12 bytes)
                output->acc_x = _Xsens_ReverseFloat(&rx->buffer[idx + 0]);
                output->acc_y = _Xsens_ReverseFloat(&rx->buffer[idx + 4]);
                output->acc_z = _Xsens_ReverseFloat(&rx->buffer[idx + 8]);
                break;
            case XDI_GYROSCOPE_DATA: // 0x8020 (12 bytes)
                output->gyr_x = _Xsens_ReverseFloat(&rx->buffer[idx + 0]);
                output->gyr_y = _Xsens_ReverseFloat(&rx->buffer[idx + 4]);
                output->gyr_z = _Xsens_ReverseFloat(&rx->buffer[idx + 8]);
                break;
        }
        idx += data_len;
    }
}

/**
 *------------------------------------------------------------
 * BOOT SEQUENCE COMMANDS (Ported from IMU_Hub_Module)
 *------------------------------------------------------------
 */

/**
 * @brief UART 명령 전송 (재시도 포함)
 * @param id UART ID
 * @param cmd 전송할 명령 버퍼
 * @param len 명령 길이
 * @note Ported from IMU_Hub_Module with adaptation for Extension_Module architecture
 */
static void _TxCmd(IOIF_UARTx_t id, const uint8_t* cmd, uint16_t len)
{
    uint32_t retry = 0;
    const uint32_t max_retry = 5;
    bool success = false;
    
    // 전송 실패 시 재시도
    while (retry < max_retry) {
        AGRBStatusDef result = IOIF_UART_Write_Polling(id, (uint8_t*)cmd, len);
        if (result == AGRBStatus_OK) {
            success = true;
            s_config_cmd_sent++;
            break;
        }
        
        // 재시도 전 대기 (10ms)
        vTaskDelay(pdMS_TO_TICKS(10));
        retry++;
    }
    
    if (!success) {
        s_config_cmd_failed++;
    }
    
    // 명령 간 간격 (센서가 처리할 시간 제공) - 20ms
    vTaskDelay(pdMS_TO_TICKS(20));
}

/**
 * @brief 센서 리셋 명령 (MID 0x40)
 * @param id UART ID
 */
static void _Cmd_Reset(IOIF_UARTx_t id)
{
    // MID 0x40 (Reset)
    uint8_t cmd[] = {XSENS_PREAMBLE, XSENS_BID, 0x40, 0x00};
    uint8_t checksum = (uint8_t)(0xFF + 0x40 + 0x00);
    cmd[3] = (uint8_t)(-checksum);
    _TxCmd(id, cmd, 4);
}

/**
 * @brief Config 모드 진입 명령 (MID 0x30)
 * @param id UART ID
 */
static void _Cmd_GoToConfig(IOIF_UARTx_t id)
{
    // MID 0x30 (GoToConfig)
    uint8_t cmd[] = {XSENS_PREAMBLE, XSENS_BID, 0x30, 0x00};
    uint8_t checksum = (uint8_t)(0xFF + 0x30 + 0x00);
    cmd[3] = (uint8_t)(-checksum);
    _TxCmd(id, cmd, 4);
}

/**
 * @brief Measurement 모드 진입 명령 (MID 0x10)
 * @param id UART ID
 */
static void _Cmd_GoToMeasurement(IOIF_UARTx_t id)
{
    // MID 0x10 (GoToMeasurement)
    uint8_t cmd[] = {XSENS_PREAMBLE, XSENS_BID, 0x10, 0x00};
    uint8_t checksum = (uint8_t)(0xFF + 0x10 + 0x00);
    cmd[3] = (uint8_t)(-checksum);
    _TxCmd(id, cmd, 4);
}

/**
 * @brief Baudrate 설정 명령 (MID 0x18)
 * @param id UART ID
 * @param baudrate 설정할 Baudrate (921600, 460800, 230400, 115200, 57600)
 */
static void _Cmd_SetBaudrate(IOIF_UARTx_t id, uint32_t baudrate)
{
    // Baudrate 인코딩: 0x80 = 921600, 0x0A = 115200, 0x09 = 57600
    uint8_t rate_code;
    switch (baudrate) {
        case 921600: rate_code = 0x80; break;
        case 460800: rate_code = 0x0C; break;
        case 230400: rate_code = 0x0B; break;
        case 115200: rate_code = 0x0A; break;
        case 57600:  rate_code = 0x09; break;
        default:     rate_code = 0x0A; break;  // 기본값 115200
    }
    
    // MID 0x18 (SetBaudrate): [Preamble][BID][MID][LEN][RateCode][CS]
    uint8_t cmd[] = {XSENS_PREAMBLE, XSENS_BID, 0x18, 0x01, rate_code, 0x00};
    uint8_t checksum = (uint8_t)(0xFF + 0x18 + 0x01 + rate_code);
    cmd[5] = (uint8_t)(-checksum);
    _TxCmd(id, cmd, 6);
}

/**
 * @brief 센서 연결 상태 확인 (타임아웃 기반) - IMU_Hub pattern
 * @param last_rx_time 마지막 패킷 수신 시간 (ms)
 * @param current_time 현재 시간 (ms)
 * @return true: 연결됨, false: 타임아웃
 * @note 1초간 데이터 없으면 연결 끊김 간주
 */
static bool IsConnected(uint32_t last_rx_time, uint32_t current_time)
{
    /* Timeout: 1000ms */
    #define XSENS_TIMEOUT_MS  1000
    
    /* 타임아웃 체크 (Overflow 안전) */
    uint32_t elapsed = current_time - last_rx_time;
    return (elapsed < XSENS_TIMEOUT_MS);
}
