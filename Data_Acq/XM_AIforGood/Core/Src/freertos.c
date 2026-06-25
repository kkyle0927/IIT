/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Phase 2: FreeRTOS Heap — DTCMRAM (Zero-Wait-State, DMA 접근 불가 → 커널 객체 보호) */
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".dtcm_data"), aligned(8)));
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
/* [DIAG 2026-05-12 cross-port from 0428] Stack overflow 검출 진단.
 *  - 이전에는 빈 hook → silent corruption → 이후 freeze/hardfault 의 root cause.
 *  - 활성화 후: stack overflow 발생 즉시 infinite loop 에 진입.
 *    디버거 Suspend 하면 g_overflow_task_name 변수로 어느 task 인지 확인 가능.
 *  - 다음 단계: 식별된 task 의 stack size 를 늘리거나 stack 사용 줄이기. */
/* .noinit 배치 — POR 외 reset (NVIC_SystemReset, lockup auto-reset) 에서 보존.
 * 다음 boot 시 main() 가 g_overflow_detected 검사로 이전 boot 의 stack overflow
 * 흔적 확인 가능. POR 직후엔 random 값일 수 있으므로 detected==1 일 때만 신뢰. */
__attribute__((section(".noinit"), used))
volatile char     g_overflow_task_name[16];
__attribute__((section(".noinit"), used))
volatile uint32_t g_overflow_detected;

/* 직전 boot 시점의 RCC->RSR snapshot — main() 진입 직후 저장 + RMVF clear.
 * STM32H7 RSR 은 software 가 clear 안 하면 모든 boot 의 flag 누적. snapshot
 * + clear 로 정확히 "직전 reset 원인" 만 표시. POR 후엔 random, 이후엔 0xXX
 * 형식의 RSR 값. CubeProgrammer 로 read 가능 (주소 map 에서 확인). */
__attribute__((section(".noinit"), used))
volatile uint32_t g_last_rcc_rsr;

void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
    (void)xTask;

    if (pcTaskName != NULL) {
        for (int i = 0; i < 15 && pcTaskName[i] != '\0'; i++) {
            g_overflow_task_name[i] = pcTaskName[i];
        }
        g_overflow_task_name[15] = '\0';
    } else {
        g_overflow_task_name[0] = '?';
        g_overflow_task_name[1] = '\0';
    }
    g_overflow_detected = 1;

    /* Infinite loop — 디버거 Suspend 시 여기서 잡힘. PC + g_overflow_task_name 확인.
     * [Note] 전역 인터럽트는 차단하지 않는다 (hook 정책 + 다른 ISR가 디버거 attach
     *  안정성을 깨뜨릴 만큼 corrupted 영역을 건드릴 가능성 낮음).
     * [Phase 2 RM] LED/CDC 친화 메시지는 XM10 Risk Management 통합 단계에서 도입. */
    for (;;) {
        __NOP();
    }
}

/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

