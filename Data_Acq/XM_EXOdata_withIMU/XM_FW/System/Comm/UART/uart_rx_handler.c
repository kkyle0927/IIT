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
#include "MDAF-25-6850.h"  // Fixed: uppercase filename
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

/* [ROBUST] Diagnostic counters */
static volatile uint32_t s_imu_queue_drops = 0;
static volatile uint32_t s_xsens_boot_errors = 0;

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
        s_xsens_boot_errors++;
        // TODO: Trigger system alert (LED blink, log error, etc.)
        return;
    }

    // 2. [ROBUST] 센서 부팅 대기 및 Output 설정 (IMU_Hub pattern)
    // ConfigureOutput() 함수가 전체 boot sequence를 수행:
    //   - 500ms 센서 부팅 대기
    //   - GoToConfig
    //   - SetOutputConfig (1kHz: Quat+Acc+Gyro)
    //   - GoToMeasurement
    if (xsensMTi630.ConfigureOutput() == false) {
        s_xsens_boot_errors++;
        // TODO: 설정 실패 처리 (재시도 or 안전 모드)
        return;
    }
    
    // 3. 추가 안정화 대기 (데이터 스트리밍 시작 확인)
    vTaskDelay(pdMS_TO_TICKS(100));
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
    BaseType_t result = xQueueSendFromISR(s_imuQueue, packet, &xHigherPriorityTaskWoken);
    
    if (result != pdTRUE) {
        // [ROBUST] 큐가 꽉 찼을 때 드롭 카운터 증가
        s_imu_queue_drops++;
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *------------------------------------------------------------
 * DIAGNOSTIC API
 *------------------------------------------------------------
 */

/**
 * @brief [디버깅] Xsens IMU 큐 드롭 카운터 가져오기
 */
uint32_t Uart4Rx_GetImuDropCount(void)
{
    return s_imu_queue_drops;
}

/**
 * @brief [디버깅] Xsens 부팅 에러 카운터 가져오기
 */
uint32_t Uart4Rx_GetXsensBootErrors(void)
{
    return s_xsens_boot_errors;
}
