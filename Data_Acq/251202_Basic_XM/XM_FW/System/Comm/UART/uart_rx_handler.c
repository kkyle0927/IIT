/**
 ******************************************************************************
 * @file    uart_rx_handler.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Nov 5, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "uart_rx_handler.h"
#include <string.h>
#include <stdbool.h>

// RTOS 태스크 생성 및 속성 정의를 위해 헤더 추가
#include "FreeRTOS.h"
#include "module.h"     // 태스크 속성 매크로 (TASK_STACK_*, TASK_PRIO_*)
#include "cmsis_os2.h"  // for osThreadNew, osThreadId_t

// Devices Layer
#include "mdaf-25-6850.h"
#include "mti-630.h" // [신규] Xsens 드라이버

#include "grf_xm_link.h" // grf_xm_link의 데이터 업데이트 함수를 호출
#include "xsens_imu_xm_link.h" // IMU 데이터를 전달할 모듈

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

// RTOS 태스크 핸들 및 속성 정의
static osThreadId_t s_uartRxTaskHandle;
static const osThreadAttr_t s_uartRxTask_attributes = {
  .name = "UartRxTask", // 태스크 이름
  .stack_size = TASK_STACK_UART_RX,
  .priority = (osPriority_t) TASK_PRIO_UART_RX,
};

/* System Layer 큐 (ISR -> Task) */
static QueueHandle_t s_fsrQueue = NULL;
static QueueHandle_t s_imuQueue = NULL;

/* 큐 Set 핸들 (두 큐를 동시에 기다리기 위함) */
static QueueSetHandle_t s_uartQueueSet = NULL;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void StartUartRxTask(void* argument);
static void _OnFsrPacketReceived(const MarvelDex_packet_t* packet);
static void _OnImuPacketReceived(const XsensMTi_packet_t* packet);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void UartRxHandler_Init(IOIF_UARTx_t grf_left_id, IOIF_UARTx_t grf_right_id)
{
    // 1. System Layer 큐 생성
    s_fsrQueue = xQueueCreate(64, sizeof(MarvelDex_packet_t));
    s_imuQueue = xQueueCreate(64, sizeof(XsensMTi_packet_t)); // IMU용 미리 생성
    
    // 2. 큐 Set이 아직 생성되지 않았으면(NULL이면) 새로 생성
    // 큐 세트의 크기는 FSR(64) + IMU(64) = 128 설정
    if (s_uartQueueSet == NULL) {
        s_uartQueueSet = xQueueCreateSet(64 + 64); 

        xQueueAddToSet(s_fsrQueue, s_uartQueueSet);
        xQueueAddToSet(s_imuQueue, s_uartQueueSet); // IMU 큐도 미리 추가
    }

    // 3. GRF Device 드라이버 초기화 (큐 콜백 주입)
    if (marvelDexFSR.init(grf_left_id, grf_right_id, _OnFsrPacketReceived) == false) {
        // TODO: 치명적 오류 처리
    }

    // 3. 데이터 분배 태스크(소비자) 생성
    s_uartRxTaskHandle = osThreadNew(StartUartRxTask, NULL, &s_uartRxTask_attributes);
    if (s_uartRxTaskHandle == NULL) {
        // TODO: 치명적 오류 처리
    }
}

void Uart4Rx_XsensIMU_Init(IOIF_UARTx_t imu_id)
{
    // 1. Xsens Device 드라이버 초기화 (큐 콜백 주입)
    if (xsensMTi630.init(imu_id, _OnImuPacketReceived) == false) {
        // TODO: 오류
    }

    // 2. Xsens IMU가 부팅될 시간을 잠시 대기 (매우 중요)
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 대기

    // 3. Xsens IMU에 1kHz 출력 설정 명령 전송
    // xsensMTi630.ConfigureOutput(); xsense 자체 프로그램으로 사전 설정
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

// --------- idle event 기반 ---------- //
/**
 * @brief [수정] 1ms 폴링이 아닌, 큐(Queue) 이벤트 기반의 분배 태스크
 * @details mdaf-25-6850 또는 xsens_mti_630의 큐에 
 * 데이터가 들어올 때까지 무한 대기(Blocking)합니다.
 */
static void StartUartRxTask(void* argument)
{
    MarvelDex_packet_t fsr_packet;
    XsensMTi_packet_t  imu_packet;
    QueueSetMemberHandle_t xActivatedMember;

    for (;;) {
        // 1ms마다 깨어나는 대신, 큐 Set에 데이터가 들어올 때까지 무한 대기
        // 큐 세트 대기 (FSR 또는 IMU 데이터 도착 시 깨어남)
        xActivatedMember = xQueueSelectFromSet(s_uartQueueSet, portMAX_DELAY);

        /* 1. FSR(GRF) 데이터 처리 */
        if (xActivatedMember == s_fsrQueue) {
            // 큐가 빌 때까지 모든 데이터 처리
            while (xQueueReceive(s_fsrQueue, &fsr_packet, 0) == pdTRUE) {
                // Link Layer에 업데이트 요청 (Lock-Free Write)
                GRF_XM_Link_UpdateData(&fsr_packet);
            }
        }

        /* 2. IMU 데이터 처리 */
        else if (xActivatedMember == s_imuQueue) {
            // 큐가 빌 때까지 모든 데이터 처리
            while (xQueueReceive(s_imuQueue, &imu_packet, 0) == pdTRUE) {
                // Link Layer에 업데이트 요청 (Lock-Free Write)
                XsensIMU_XM_Link_UpdateData(&imu_packet);
            }
        }
    }
}

/**
 * @brief FSR Devices Layer가 호출할 콜백 (ISR 컨텍스트)
 */
static void _OnFsrPacketReceived(const MarvelDex_packet_t* packet)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(s_fsrQueue, packet, &xHigherPriorityTaskWoken) != pdTRUE) {
        // [디버깅] 여기에 브레이크 포인트: 큐가 꽉 찼다는 뜻
        // g_grf_drop_count++; // 이런 변수를 만들어 모니터링
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief IMU Devices Layer가 호출할 콜백 (ISR 컨텍스트)
 */
static void _OnImuPacketReceived(const XsensMTi_packet_t* packet)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(s_imuQueue, packet, &xHigherPriorityTaskWoken) != pdTRUE) {
        // [디버깅] 여기에 브레이크 포인트: 큐가 꽉 찼다는 뜻
        // g_grf_drop_count++; // 이런 변수를 만들어 모니터링
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
