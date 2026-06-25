/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_6_UART_RX_Pin GPIO_PIN_0
#define IMU_6_UART_RX_GPIO_Port GPIOC
#define IMU_6_UART_TX_Pin GPIO_PIN_1
#define IMU_6_UART_TX_GPIO_Port GPIOC
#define IMU_2_UART_TX_Pin GPIO_PIN_2
#define IMU_2_UART_TX_GPIO_Port GPIOA
#define IMU_2_UART_RX_Pin GPIO_PIN_3
#define IMU_2_UART_RX_GPIO_Port GPIOA
#define IMU_1_UART_TX_Pin GPIO_PIN_4
#define IMU_1_UART_TX_GPIO_Port GPIOC
#define IMU_1_UART_RX_Pin GPIO_PIN_5
#define IMU_1_UART_RX_GPIO_Port GPIOC
#define IMU_3_UART_TX_Pin GPIO_PIN_10
#define IMU_3_UART_TX_GPIO_Port GPIOB
#define IMU_3_UART_RX_Pin GPIO_PIN_11
#define IMU_3_UART_RX_GPIO_Port GPIOB
#define FDCAN_RX_Pin GPIO_PIN_12
#define FDCAN_RX_GPIO_Port GPIOB
#define FDCAN_TX_Pin GPIO_PIN_13
#define FDCAN_TX_GPIO_Port GPIOB
#define POWER_ON_LED_Pin GPIO_PIN_6
#define POWER_ON_LED_GPIO_Port GPIOC
#define IMU_HUB_LED_R_Pin GPIO_PIN_7
#define IMU_HUB_LED_R_GPIO_Port GPIOC
#define IMU_HUB_LED_G_Pin GPIO_PIN_8
#define IMU_HUB_LED_G_GPIO_Port GPIOC
#define IMU_HUB_LED_B_Pin GPIO_PIN_9
#define IMU_HUB_LED_B_GPIO_Port GPIOC
#define IMU_PWR_EN_Pin GPIO_PIN_8
#define IMU_PWR_EN_GPIO_Port GPIOA
#define IMU_4_UART_TX_Pin GPIO_PIN_10
#define IMU_4_UART_TX_GPIO_Port GPIOC
#define IMU_4_UART_RX_Pin GPIO_PIN_11
#define IMU_4_UART_RX_GPIO_Port GPIOC
#define IMU_5_UART_TX_Pin GPIO_PIN_12
#define IMU_5_UART_TX_GPIO_Port GPIOC
#define IMU_5_UART_RX_Pin GPIO_PIN_2
#define IMU_5_UART_RX_GPIO_Port GPIOD
#define FUNC_BTN_2_Pin GPIO_PIN_4
#define FUNC_BTN_2_GPIO_Port GPIOB
#define FUNC_BTN_1_Pin GPIO_PIN_5
#define FUNC_BTN_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


extern UART_HandleTypeDef hlpuart1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef hlpuart1;

extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;



extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;

extern PCD_HandleTypeDef hpcd_USB_FS;

extern FDCAN_HandleTypeDef hfdcan2;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
