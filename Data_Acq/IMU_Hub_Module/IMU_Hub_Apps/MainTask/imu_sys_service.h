/**
 ******************************************************************************
 * @file    imu_sys_service.h
 * @author  HyundoKim
 * @brief   [Application Layer] IMU Hub System Service - LED, PnP, DOP Serial Rx
 * @version 3.0 (단일 MainTask 로 fold — FDCAN Tx single-writer)
 * @date    2026-03-27
 *
 * @details
 * (구) 독립 TIM7 System Task 였으나, BareMetal FDCAN Tx single-writer 를 위해
 * 단일 MainTask(TIM16) ISR 안으로 fold 되었다. 더 이상 자체 Timer ISR 가 없는
 * "비실시간 시스템 서비스" — ImuHub_Task_RunLoop() 가 매 tick 호출한다.
 *
 * [호출 구조 — 단일 Timer ISR]
 * TIM16 ISR (1ms, NVIC preempt 1) — MainTask
 *   └── ImuHub_Task_RunLoop()
 *         ├── _FetchInputs()    (센서 수집)
 *         ├── TaskMngr_Run()    (사용자 알고리즘)
 *         ├── _FlushOutputs()   (TPDO direct) + XM_Drv_DrainTxDeferRing() (단일 Tx)
 *         └── ImuSysService_Run()  ← 본 서비스 (비RT, tick-gate)
 *               ├── DOP Serial Rx (CDC → COBS → SDO)  (매 tick)
 *               ├── DOP Periodic (Heartbeat 1s)        (매 tick)
 *               ├── IO_Manager_Update()                (매 tick)
 *               ├── _UpdateLedState()                  (100ms)
 *               ├── _UpdateDebugMonitors()             (100ms)
 *               └── XM_Drv_RunPeriodic()               (500ms, PnP HB Tx — 단일 writer)
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IMU_HUB_APPS_MAINTASK_IMU_SYS_SERVICE_H_
#define IMU_HUB_APPS_MAINTASK_IMU_SYS_SERVICE_H_

#include <stdint.h>

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/**
 * @brief System Service 초기화 (1회 실행)
 * @details system_startup.c의 _InitSystemServices() 또는 main.c 에서 호출.
 */
void ImuSysService_Init(void);

/**
 * @brief System Service 실행 (비RT) — MainTask(TIM16) 루프에서 매 tick 호출
 * @details 자체 Timer ISR 없음. ImuHub_Task_RunLoop() 가 호출. 내부 tick-gate 분기.
 *
 * [주기 분기]
 * - 매 tick:  DOP Serial Rx, DOP Periodic (Heartbeat), IO_Manager_Update()
 * - 100ms:   LED 상태 자동 매핑, Debug Monitor 업데이트, IMU Auto-Sense timeout
 * - 500ms:   XM_Drv_RunPeriodic() (NMT/PnP/Heartbeat — 단일 writer 컨텍스트)
 */
void ImuSysService_Run(void);

#endif /* IMU_HUB_APPS_MAINTASK_IMU_SYS_SERVICE_H_ */
