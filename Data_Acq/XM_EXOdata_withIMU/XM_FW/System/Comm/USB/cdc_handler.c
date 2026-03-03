/**
 ******************************************************************************
 * @file    cdc_handler.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Nov 13, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "cdc_handler.h"
#include "usbd_cdc_if.h" // CDC_Transmit_FS, CDC_Register_Callbacks

#include "FreeRTOS.h"
#include "queue.h"

#include <string.h>
#include <stdatomic.h>

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
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// --- Non-Cacheable 영역에 링버퍼 할당 (DMA 충돌 방지 및 속도 향상) ---
__attribute__((section(IOIF_USB_CDC_SECTION))) 
static uint8_t s_cdcTxRingBuffer[CDC_TX_RING_BUFFER_SIZE];

__attribute__((section(IOIF_USB_CDC_SECTION))) 
static uint8_t s_cdcRxRingBuffer[CDC_RX_RING_BUFFER_SIZE];

// --- Tx Control (SPSC: Single Producer Single Consumer) ---
static volatile atomic_uint s_txHead = 0; // Producer(User)
static volatile uint32_t s_txTail = 0;    // Consumer(ISR)
static volatile atomic_bool s_isTxBusy = false; // 하드웨어 상태

// --- Rx Control ---
static volatile uint32_t s_rxHead = 0;    // Producer(ISR)
static volatile atomic_uint s_rxTail = 0; // Consumer(User)

// 모니터링 활성화 플래그 (PC와 연결 후 단순 시리얼 모니터링 진행 시 사용)
static volatile atomic_bool s_isStreamingActive = false;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _TryTransmit(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void CdcStream_Init(void)
{
    atomic_store(&s_txHead, 0);
    s_txTail = 0;
    s_rxHead = 0; 
    atomic_store(&s_rxTail, 0);
    atomic_store(&s_isTxBusy, false);
    atomic_store(&s_isStreamingActive, false); // 초기엔 끔
}

// [Tx] UserTask가 호출 (구조체 전송)
bool CdcStream_Send(const void* data, uint32_t len)
{
    if (len == 0 || len > CDC_TX_RING_BUFFER_SIZE) return false;

    uint32_t head = atomic_load(&s_txHead);
    uint32_t tail = s_txTail; // ISR 변수지만 volatile 읽기 안전

    // 공간 계산
    uint32_t free = (head >= tail) ? (CDC_TX_RING_BUFFER_SIZE - (head - tail) - 1) : ((tail - head) - 1);
    if (len > free) return false; // 버퍼 꽉 참

    // 데이터 복사 (Wrap-around)
    const uint8_t* pData = (const uint8_t*)data;
    uint32_t chunk1 = (head + len > CDC_TX_RING_BUFFER_SIZE) ? (CDC_TX_RING_BUFFER_SIZE - head) : len;
    memcpy(&s_cdcTxRingBuffer[head], pData, chunk1);
    if (len > chunk1) memcpy(&s_cdcTxRingBuffer[0], pData + chunk1, len - chunk1);

    // Head 갱신 (Memory Barrier 역할 포함)
    atomic_store(&s_txHead, (head + len) % CDC_TX_RING_BUFFER_SIZE);

    // 전송 트리거 (Lock-Free)
    // 이미 전송 중이면 ISR이 끝나고 이어서 보낼 것이므로 무시
    if (!atomic_load(&s_isTxBusy)) {
        _TryTransmit();
    }
    return true;
}

// [Rx] UserTask가 호출
uint32_t CdcStream_Read(void* buf, uint32_t max_len)
{
    uint32_t head = s_rxHead; // ISR 변수 읽기
    uint32_t tail = atomic_load(&s_rxTail);

    if (head == tail) return 0;

    uint32_t available = (head >= tail) ? (head - tail) : (CDC_RX_RING_BUFFER_SIZE - tail + head);
    uint32_t read_len = (available > max_len) ? max_len : available;

    uint8_t* pBuf = (uint8_t*)buf;
    uint32_t chunk1 = (tail + read_len > CDC_RX_RING_BUFFER_SIZE) ? (CDC_RX_RING_BUFFER_SIZE - tail) : read_len;
    
    memcpy(pBuf, &s_cdcRxRingBuffer[tail], chunk1);
    if (read_len > chunk1) memcpy(pBuf + chunk1, &s_cdcRxRingBuffer[0], read_len - chunk1);

    atomic_store(&s_rxTail, (tail + read_len) % CDC_RX_RING_BUFFER_SIZE);
    return read_len;
}

/**
 * @brief 모니터링 상태 반환 (Facade용)
 */
bool CdcStream_IsStreamingActive(void)
{
    return atomic_load(&s_isStreamingActive);
}

// ISR: 전송 완료 콜백
void CdcStream_OnTxComplete(void)
{
    atomic_store(&s_isTxBusy, false); // HW 바쁨 해제
    _TryTransmit(); // 남은 데이터 연쇄 전송
}

/**
 * @brief ISR: Rx 수신 콜백 + 명령어 감지기
 */
void CdcStream_OnRxReceived(uint8_t* data, uint32_t len)
{
    // 1. [Parser] 들어온 데이터가 '명령어'인지 먼저 확인
    // (주의: PC 터미널은 \r\n을 붙일 수 있으므로, 길이 체크 대신 strncmp 사용)
    if (len >= strlen(CDC_CMD_STREAMING_START)) {
        if (strncmp((char*)data, CDC_CMD_STREAMING_START, strlen(CDC_CMD_STREAMING_START)) == 0) {
            atomic_store(&s_isStreamingActive, true);
            // 명령 패킷은 링버퍼에 넣지 않고 여기서 소비(Discard)해도 됨
            // 하지만 Scenario 2와의 호환성을 위해 일단 링버퍼에 넣는 것도 방법임.
            // 여기서는 '명령어'는 소비하고 리턴하는 방식 채택 (깔끔함)
            return; 
        }
    }
    
    if (len >= strlen(CDC_CMD_STREAMING_STOP)) {
        if (strncmp((char*)data, CDC_CMD_STREAMING_STOP, strlen(CDC_CMD_STREAMING_STOP)) == 0) {
            atomic_store(&s_isStreamingActive, false);
            return;
        }
    }

    // 2. [기존 로직] 링버퍼에 데이터 저장 (Scenario 2 프로토콜 데이터용)
    uint32_t tail = atomic_load(&s_rxTail);
    uint32_t head = s_rxHead;

    uint32_t free = (tail > head) ? (tail - head - 1) : (CDC_RX_RING_BUFFER_SIZE - (head - tail) - 1);
    if (len > free) return; // 오버플로우 (데이터 버림)

    // 링버퍼 복사
    uint32_t chunk1 = (head + len > CDC_RX_RING_BUFFER_SIZE) ? (CDC_RX_RING_BUFFER_SIZE - head) : len;
    memcpy(&s_cdcRxRingBuffer[head], data, chunk1);
    if (len > chunk1) memcpy(&s_cdcRxRingBuffer[0], data + chunk1, len - chunk1);

    s_rxHead = (head + len) % CDC_RX_RING_BUFFER_SIZE; // ISR에서만 쓰므로 atomic 불필요
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

// [Internal] 실제 전송 시도 (UserTask 또는 ISR에서 호출)
static void _TryTransmit(void)
{
    // 1. Atomic Flag Test-and-Set (Critical Section 대체)
    // 이미 바쁘면 나감. (true->true: true반환, false->true: false반환)
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_isTxBusy, &expected, true)) {
        return; 
    }

    uint32_t head = atomic_load(&s_txHead);
    uint32_t tail = s_txTail;

    if (head == tail) {
        atomic_store(&s_isTxBusy, false); // 보낼 거 없음, Busy 해제
        return;
    }

    // 2. 연속된 데이터 길이 계산
    uint32_t len = (head > tail) ? (head - tail) : (CDC_TX_RING_BUFFER_SIZE - tail);
    if (len > APP_TX_DATA_SIZE) len = APP_TX_DATA_SIZE; // USB 패킷 제한

    // 3. 하드웨어 전송 (복사 없이 링버퍼 포인터 직접 전달!)
    // D2 영역이므로 캐시 문제 없음.
    if (CDC_Transmit_FS(&s_cdcTxRingBuffer[tail], (uint16_t)len) == USBD_OK) {
        s_txTail = (tail + len) % CDC_TX_RING_BUFFER_SIZE;
        // s_isTxBusy는 ISR(TxCplt)에서 해제됨
    } else {
        atomic_store(&s_isTxBusy, false); // 실패 시 즉시 해제
    }
}
