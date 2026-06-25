/**
 ******************************************************************************
 * @file    am_drv.c
 * @author  HyundoKim
 * @brief   Application Module (Jetson Orin) Device Driver — Implementation
 * @version 1.0
 * @date    Mar 2, 2026
 *
 * @details
 * cm_drv.c와 동일한 Mutex + Snapshot 패턴.
 *   - Writer: AGR_DOP UDP 콜백에서 호출 (tcpip_thread context)
 *   - Reader: Main Task 1kHz (core_process.c)
 *   - Mutex timeout: 1ms (제어 루프 지터 최소화)
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "am_drv.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

#define AM_MUTEX_TIMEOUT_MS     1U      /**< Mutex 대기 시간 (ms) */
#define AM_CONNECTION_TIMEOUT_MS 3000U  /**< 연결 타임아웃 (ms) */

#define AM_DATA_LOCK()   (s_am_data_mutex != NULL && \
                          xSemaphoreTake(s_am_data_mutex, pdMS_TO_TICKS(AM_MUTEX_TIMEOUT_MS)) == pdTRUE)
#define AM_DATA_UNLOCK() xSemaphoreGive(s_am_data_mutex)

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

static SemaphoreHandle_t s_am_data_mutex = NULL;
static AM_RxData_t       s_am_datalake;          /**< Mutex 보호 데이터 */
static uint32_t          s_last_rx_tick  = 0;     /**< 마지막 수신 시각 */
static uint8_t           s_am_node_id   = 0;      /**< AM Node ID */

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

int AM_Drv_Init(uint8_t am_node_id)
{
    s_am_node_id = am_node_id;

    /* Mutex 생성 (1회만) */
    if (s_am_data_mutex == NULL) {
        s_am_data_mutex = xSemaphoreCreateMutex();
        if (s_am_data_mutex == NULL) return -1;
    }

    /* Datalake 초기화 */
    memset(&s_am_datalake, 0, sizeof(AM_RxData_t));
    s_last_rx_tick = 0;

    return 0;
}

/* --- Writer: AGR_DOP UDP 콜백 (tcpip_thread) --- */

void AM_Drv_ProcessPdo(uint8_t pdo_type, uint8_t src_id,
                        const uint8_t* data, uint8_t len)
{
    (void)pdo_type; /* 향후 PDO 번호별 분기 확장 */

    if (data == NULL || len == 0) return;

    if (AM_DATA_LOCK()) {
        /* Raw PDO 저장 (향후 구조체 디코딩으로 확장) */
        uint8_t copy_len = (len > sizeof(s_am_datalake.raw_pdo))
                             ? sizeof(s_am_datalake.raw_pdo) : len;
        memcpy(s_am_datalake.raw_pdo, data, copy_len);
        s_am_datalake.raw_len = copy_len;
        s_am_datalake.timestamp = xTaskGetTickCount();

        s_last_rx_tick = s_am_datalake.timestamp;
        AM_DATA_UNLOCK();
    }
}

void AM_Drv_ProcessSdo(uint8_t msg_type, uint8_t src_id,
                        const uint8_t* data, uint8_t len)
{
    (void)msg_type; (void)src_id; (void)data; (void)len;

    /* SDO 처리는 향후 구현 (NRT 큐 기반 요청-응답) */
    /* 현재는 수신 시각만 업데이트하여 연결 상태 유지 */
    if (AM_DATA_LOCK()) {
        s_last_rx_tick = xTaskGetTickCount();
        AM_DATA_UNLOCK();
    }
}

/* --- Reader: Main Task 1kHz (core_process.c) --- */

bool AM_Drv_GetRxData(AM_RxData_t* out)
{
    if (out == NULL) return false;

    if (AM_DATA_LOCK()) {
        *out = s_am_datalake; /* Snapshot copy */
        AM_DATA_UNLOCK();
        return true;
    }
    return false; /* Mutex timeout — 이전 값 사용 */
}

bool AM_Drv_IsConnected(void)
{
    if (s_last_rx_tick == 0) return false;
    return (xTaskGetTickCount() - s_last_rx_tick) < pdMS_TO_TICKS(AM_CONNECTION_TIMEOUT_MS);
}
