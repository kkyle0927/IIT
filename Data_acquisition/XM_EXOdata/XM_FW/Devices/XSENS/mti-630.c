/**
 ******************************************************************************
 * @file    mti-630.c
 * @author  HyundoKim
 * @brief   [Devices Layer] Xsens MTi-630 IMU 드라이버 구현부
 * @details Xbus 프로토콜(MTData2) 파서, 1kHz 설정 기능 포함
 * @version 0.1
 * @date    Nov 12, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "mti-630.h"
#include <string.h>
#include "ioif_agrb_tim.h" // IOIF_GetTick()
#include "ioif_agrb_uart.h" // ioif_uart_write

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
XsensMTi_packet_t parsed_packet;

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
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

XsensMTi630_t xsensMTi630 = {
    .init = initialize,
    .ConfigureOutput = ConfigureOutput,
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

    ioif_uart_update_rx_callback(s_uart_id, uart_callback);
    return true;
}

/**
 * @brief [신규] 1kHz로 Quat, Acc, Gyro를 요청하는 설정 메시지를 전송
 * @details MTi-630이 출력할 데이터를 변경하고자 한다면 내용을 변경해야 함.
 * XSENS 자체 내장 프로그램으로 IMU의 설정을 쉽게 변경할 수 있어서 ConfigureOutput을 사용하지 않음.
 */
static bool ConfigureOutput(void)
{
    // MTData2 Output Configuration (MID 0xC0)
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

    // IOIF UART로 전송 (Blocking)
    if (ioif_uart_write(s_uart_id, tx_buffer, (5 + len)) == AGRBStatus_OK) {
        return true;
    }
    return false;
}

static void uart_callback(uint8_t* rx_buf, uint32_t size, uint32_t id)
{
    if (id != s_uart_id) return;

    for (uint32_t i = 0; i < size; i++) {
        if (parse_byte(rx_buf[i])) {
            // true: 패킷 수신/검증 완료
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
            if (s_rx_buffer.index >= s_rx_buffer.len) {
                s_parse_state = STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_CHECKSUM:
            // [강화] 수신된 체크섬을 포함한 총 합이 0x00(1-complement)이어야 함
            if (s_rx_buffer.checksum == 0x00) {
                return true; // 파싱 성공, 패킷 완료
            }
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
    output->timestamp = IOIF_GetTick();
    
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
