/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "module.h"
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

/* USER CODE BEGIN Private defines */
/* ########################## L30_MD_REV06_ENABLED ############################## */
#ifdef L30_MD_REV06_ENABLED
#define SW_DEBUG__Pin GPIO_PIN_2
#define SW_DEBUG__GPIO_Port GPIOE
#define LED_TEST_Pin GPIO_PIN_3
#define LED_TEST_GPIO_Port GPIOE
#define LM_IN2_Pin GPIO_PIN_4
#define LM_IN2_GPIO_Port GPIOE
#define LM_IN1_Pin GPIO_PIN_5
#define LM_IN1_GPIO_Port GPIOE
#define LM_DOWN_Pin GPIO_PIN_6
#define LM_DOWN_GPIO_Port GPIOE
#define MCU_OSC32_IN_Pin GPIO_PIN_14
#define MCU_OSC32_IN_GPIO_Port GPIOC
#define MCU_OSC32_OUT_Pin GPIO_PIN_15
#define MCU_OSC32_OUT_GPIO_Port GPIOC
#define MCU_25M_OSC_IN_Pin GPIO_PIN_0
#define MCU_25M_OSC_IN_GPIO_Port GPIOH
#define MCU_25M_OSC_OUT_Pin GPIO_PIN_1
#define MCU_25M_OSC_OUT_GPIO_Port GPIOH
#define LM_SENSOR_ADC_Pin GPIO_PIN_0
#define LM_SENSOR_ADC_GPIO_Port GPIOC
#define MOTOR_THERMISTOR_Pin GPIO_PIN_1
#define MOTOR_THERMISTOR_GPIO_Port GPIOC
#define FSR_ADC_Pin GPIO_PIN_2
#define FSR_ADC_GPIO_Port GPIOC
#define LINEAR_ADC_Pin GPIO_PIN_3
#define LINEAR_ADC_GPIO_Port GPIOC
#define MOTOR_IncENC_A_Pin GPIO_PIN_0
#define MOTOR_IncENC_A_GPIO_Port GPIOA
#define MOTOR_IncENC_B_Pin GPIO_PIN_1
#define MOTOR_IncENC_B_GPIO_Port GPIOA
#define MOTOR_IncENC_Z_Pin GPIO_PIN_2
#define MOTOR_IncENC_Z_GPIO_Port GPIOA
#define MOTOR_IncENC_EN_Pin GPIO_PIN_3
#define MOTOR_IncENC_EN_GPIO_Port GPIOA
#define MOTOR_CURRENT_W_Pin GPIO_PIN_6
#define MOTOR_CURRENT_W_GPIO_Port GPIOA
#define BEMF_W_Pin GPIO_PIN_7
#define BEMF_W_GPIO_Port GPIOA
#define MOTOR_CURRENT_V_Pin GPIO_PIN_4
#define MOTOR_CURRENT_V_GPIO_Port GPIOC
#define BEMF_V_Pin GPIO_PIN_5
#define BEMF_V_GPIO_Port GPIOC
#define MOTOR_CURRENT_U_Pin GPIO_PIN_0
#define MOTOR_CURRENT_U_GPIO_Port GPIOB
#define BEMF_U_Pin GPIO_PIN_1
#define BEMF_U_GPIO_Port GPIOB
#define LM_UP_Pin GPIO_PIN_7
#define LM_UP_GPIO_Port GPIOE
#define DRV8350_PWMA_L_Pin GPIO_PIN_8
#define DRV8350_PWMA_L_GPIO_Port GPIOE
#define DRV8350_PWMA_H_Pin GPIO_PIN_9
#define DRV8350_PWMA_H_GPIO_Port GPIOE
#define DRV8350_PWMB_L_Pin GPIO_PIN_10
#define DRV8350_PWMB_L_GPIO_Port GPIOE
#define DRV8350_PWMB_H_Pin GPIO_PIN_11
#define DRV8350_PWMB_H_GPIO_Port GPIOE
#define DRV8350_PWMC_L_Pin GPIO_PIN_12
#define DRV8350_PWMC_L_GPIO_Port GPIOE
#define DRV8350_PWMC_H_Pin GPIO_PIN_13
#define DRV8350_PWMC_H_GPIO_Port GPIOE
#define DRV8350_EN_Pin GPIO_PIN_14
#define DRV8350_EN_GPIO_Port GPIOE
#define DRV8350__FAULT_Pin GPIO_PIN_15
#define DRV8350__FAULT_GPIO_Port GPIOE
#define LTC2944_I2C_SCL_Pin GPIO_PIN_10
#define LTC2944_I2C_SCL_GPIO_Port GPIOB
#define LTC2944_I2C_SDA_Pin GPIO_PIN_11
#define LTC2944_I2C_SDA_GPIO_Port GPIOB
#define NODE_ID_LOWEST_Pin GPIO_PIN_8
#define NODE_ID_LOWEST_GPIO_Port GPIOD
#define NODE_ID_LOW_Pin GPIO_PIN_9
#define NODE_ID_LOW_GPIO_Port GPIOD
#define NODE_ID_HIGH_Pin GPIO_PIN_10
#define NODE_ID_HIGH_GPIO_Port GPIOD
#define NODE_ID_HIGHEST_Pin GPIO_PIN_11
#define NODE_ID_HIGHEST_GPIO_Port GPIOD
#define NZR_LED_Pin GPIO_PIN_12
#define NZR_LED_GPIO_Port GPIOD
#define LED_BOOT_RED_Pin GPIO_PIN_13
#define LED_BOOT_RED_GPIO_Port GPIOD
#define HALL_SENSOR_H1_Pin GPIO_PIN_6
#define HALL_SENSOR_H1_GPIO_Port GPIOC
#define HALL_SENSOR_H1_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_H2_Pin GPIO_PIN_7
#define HALL_SENSOR_H2_GPIO_Port GPIOC
#define HALL_SENSOR_H2_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_H3_Pin GPIO_PIN_8
#define HALL_SENSOR_H3_GPIO_Port GPIOC
#define HALL_SENSOR_H3_EXTI_IRQn EXTI9_5_IRQn
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_VBUS_EXTI_IRQn EXTI9_5_IRQn
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define ANKLE_AbsENC_CLK_Pin GPIO_PIN_10
#define ANKLE_AbsENC_CLK_GPIO_Port GPIOC
#define ANKLE_AbsENC_DATA_Pin GPIO_PIN_11
#define ANKLE_AbsENC_DATA_GPIO_Port GPIOC
#define MCU_FDCAN_RX_Pin GPIO_PIN_0
#define MCU_FDCAN_RX_GPIO_Port GPIOD
#define MCU_FDCAN_TX_Pin GPIO_PIN_1
#define MCU_FDCAN_TX_GPIO_Port GPIOD
#define CAN_SILENT_Pin GPIO_PIN_2
#define CAN_SILENT_GPIO_Port GPIOD
#define IMU_RDY_Pin GPIO_PIN_3
#define IMU_RDY_GPIO_Port GPIOD
#define MOTOR_AbsENC_CLK_Pin GPIO_PIN_3
#define MOTOR_AbsENC_CLK_GPIO_Port GPIOB
#define MOTOR_AbsENC_DATA_Pin GPIO_PIN_4
#define MOTOR_AbsENC_DATA_GPIO_Port GPIOB
#define IMU_I2C_SCL_Pin GPIO_PIN_6
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_I2C_SDA_Pin GPIO_PIN_7
#define IMU_I2C_SDA_GPIO_Port GPIOB
#define TP5_Pin GPIO_PIN_0
#define TP5_GPIO_Port GPIOE
#define TP6_Pin GPIO_PIN_1
#define TP6_GPIO_Port GPIOE
#endif /* L30_MD_REV06_ENABLED */

/* ########################## L30_MD_REV07_ENABLED ############################## */
#ifdef L30_MD_REV07_ENABLED
#define SW_DEBUG__Pin GPIO_PIN_2
#define SW_DEBUG__GPIO_Port GPIOE
#define LED_TEST_Pin GPIO_PIN_3
#define LED_TEST_GPIO_Port GPIOE
#define LM_IN2_Pin GPIO_PIN_4
#define LM_IN2_GPIO_Port GPIOE
#define LM_IN1_Pin GPIO_PIN_5
#define LM_IN1_GPIO_Port GPIOE
#define LM_DOWN_Pin GPIO_PIN_6
#define LM_DOWN_GPIO_Port GPIOE
#define MCU_OSC32_IN_Pin GPIO_PIN_14
#define MCU_OSC32_IN_GPIO_Port GPIOC
#define MCU_OSC32_OUT_Pin GPIO_PIN_15
#define MCU_OSC32_OUT_GPIO_Port GPIOC
#define MCU_25M_OSC_IN_Pin GPIO_PIN_0
#define MCU_25M_OSC_IN_GPIO_Port GPIOH
#define MCU_25M_OSC_OUT_Pin GPIO_PIN_1
#define MCU_25M_OSC_OUT_GPIO_Port GPIOH
#define LM_SENSOR_ADC_Pin GPIO_PIN_0
#define LM_SENSOR_ADC_GPIO_Port GPIOC
#define MOTOR_THERMISTOR_Pin GPIO_PIN_1
#define MOTOR_THERMISTOR_GPIO_Port GPIOC
#define FSR_ADC_Pin GPIO_PIN_2
#define FSR_ADC_GPIO_Port GPIOC
#define LINEAR_ADC_Pin GPIO_PIN_3
#define LINEAR_ADC_GPIO_Port GPIOC
#define MOTOR_IncENC_A_Pin GPIO_PIN_0
#define MOTOR_IncENC_A_GPIO_Port GPIOA
#define MOTOR_IncENC_B_Pin GPIO_PIN_1
#define MOTOR_IncENC_B_GPIO_Port GPIOA
#define MOTOR_IncENC_Z_Pin GPIO_PIN_2
#define MOTOR_IncENC_Z_GPIO_Port GPIOA
#define MOTOR_IncENC_EN_Pin GPIO_PIN_3
#define MOTOR_IncENC_EN_GPIO_Port GPIOA
#define MOTOR_CURRENT_W_Pin GPIO_PIN_6
#define MOTOR_CURRENT_W_GPIO_Port GPIOA
#define BEMF_W_Pin GPIO_PIN_7
#define BEMF_W_GPIO_Port GPIOA
#define MOTOR_CURRENT_V_Pin GPIO_PIN_4
#define MOTOR_CURRENT_V_GPIO_Port GPIOC
#define BEMF_V_Pin GPIO_PIN_5
#define BEMF_V_GPIO_Port GPIOC
#define MOTOR_CURRENT_U_Pin GPIO_PIN_0
#define MOTOR_CURRENT_U_GPIO_Port GPIOB
#define BEMF_U_Pin GPIO_PIN_1
#define BEMF_U_GPIO_Port GPIOB
#define LM_UP_Pin GPIO_PIN_7
#define LM_UP_GPIO_Port GPIOE
#define DRV8350_PWMA_L_Pin GPIO_PIN_8
#define DRV8350_PWMA_L_GPIO_Port GPIOE
#define DRV8350_PWMA_H_Pin GPIO_PIN_9
#define DRV8350_PWMA_H_GPIO_Port GPIOE
#define DRV8350_PWMB_L_Pin GPIO_PIN_10
#define DRV8350_PWMB_L_GPIO_Port GPIOE
#define DRV8350_PWMB_H_Pin GPIO_PIN_11
#define DRV8350_PWMB_H_GPIO_Port GPIOE
#define DRV8350_PWMC_L_Pin GPIO_PIN_12
#define DRV8350_PWMC_L_GPIO_Port GPIOE
#define DRV8350_PWMC_H_Pin GPIO_PIN_13
#define DRV8350_PWMC_H_GPIO_Port GPIOE
#define DRV8350_EN_Pin GPIO_PIN_14
#define DRV8350_EN_GPIO_Port GPIOE
#define DRV8350__FAULT_Pin GPIO_PIN_15
#define DRV8350__FAULT_GPIO_Port GPIOE
#define LTC2944_I2C_SCL_Pin GPIO_PIN_10
#define LTC2944_I2C_SCL_GPIO_Port GPIOB
#define LTC2944_I2C_SDA_Pin GPIO_PIN_11
#define LTC2944_I2C_SDA_GPIO_Port GPIOB
#define NODE_ID_LOWEST_Pin GPIO_PIN_8
#define NODE_ID_LOWEST_GPIO_Port GPIOD
#define NODE_ID_LOW_Pin GPIO_PIN_9
#define NODE_ID_LOW_GPIO_Port GPIOD
#define NODE_ID_HIGH_Pin GPIO_PIN_10
#define NODE_ID_HIGH_GPIO_Port GPIOD
#define NODE_ID_HIGHEST_Pin GPIO_PIN_11
#define NODE_ID_HIGHEST_GPIO_Port GPIOD
#define NZR_LED_Pin GPIO_PIN_12
#define NZR_LED_GPIO_Port GPIOD
#define LED_BOOT_RED_Pin GPIO_PIN_13
#define LED_BOOT_RED_GPIO_Port GPIOD
#define HALL_SENSOR_H1_Pin GPIO_PIN_6
#define HALL_SENSOR_H1_GPIO_Port GPIOC
#define HALL_SENSOR_H1_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_H2_Pin GPIO_PIN_7
#define HALL_SENSOR_H2_GPIO_Port GPIOC
#define HALL_SENSOR_H2_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_H3_Pin GPIO_PIN_8
#define HALL_SENSOR_H3_GPIO_Port GPIOC
#define HALL_SENSOR_H3_EXTI_IRQn EXTI9_5_IRQn
#define _6X_I2C_SDA_Pin GPIO_PIN_9
#define _6X_I2C_SDA_GPIO_Port GPIOC
#define _6X_I2C_SCL_Pin GPIO_PIN_8
#define _6X_I2C_SCL_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_VBUS_EXTI_IRQn EXTI9_5_IRQn
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define USB_OTG_GPIO_Pin GPIO_PIN_15
#define USB_OTG_GPIO_GPIO_Port GPIOA
#define ANKLE_AbsENC_CLK_Pin GPIO_PIN_10
#define ANKLE_AbsENC_CLK_GPIO_Port GPIOC
#define ANKLE_AbsENC_DATA_Pin GPIO_PIN_11
#define ANKLE_AbsENC_DATA_GPIO_Port GPIOC
#define MCU_FDCAN_RX_Pin GPIO_PIN_0
#define MCU_FDCAN_RX_GPIO_Port GPIOD
#define MCU_FDCAN_TX_Pin GPIO_PIN_1
#define MCU_FDCAN_TX_GPIO_Port GPIOD
#define CAN_SILENT_Pin GPIO_PIN_2
#define CAN_SILENT_GPIO_Port GPIOD
#define IMU_RDY_Pin GPIO_PIN_3
#define IMU_RDY_GPIO_Port GPIOD
#define MOTOR_AbsENC_CLK_Pin GPIO_PIN_3
#define MOTOR_AbsENC_CLK_GPIO_Port GPIOB
#define MOTOR_AbsENC_DATA_Pin GPIO_PIN_4
#define MOTOR_AbsENC_DATA_GPIO_Port GPIOB
#define _3X_I2C_SCL_Pin GPIO_PIN_6
#define _3X_I2C_SCL_GPIO_Port GPIOB
#define _3X_I2C_SDA_Pin GPIO_PIN_7
#define _3X_I2C_SDA_GPIO_Port GPIOB
#define EXT_I2C_SCL_Pin GPIO_PIN_8
#define EXT_I2C_SCL_GPIO_Port GPIOB
#define EXT_I2C_SDA_Pin GPIO_PIN_9
#define EXT_I2C_SDA_GPIO_Port GPIOB
#define TP5_Pin GPIO_PIN_0
#define TP5_GPIO_Port GPIOE
#define TP6_Pin GPIO_PIN_1
#define TP6_GPIO_Port GPIOE
#endif /* L30_MD_REV07_ENABLED */

/* ########################## L30_MD_REV08_ENABLED ############################## */
#ifdef L30_MD_REV08_ENABLED
#define SW_DEBUG__Pin GPIO_PIN_2
#define SW_DEBUG__GPIO_Port GPIOE
#define LED_TEST_Pin GPIO_PIN_3
#define LED_TEST_GPIO_Port GPIOE
#define LM_IN2_Pin GPIO_PIN_4
#define LM_IN2_GPIO_Port GPIOE
#define LM_IN1_Pin GPIO_PIN_5
#define LM_IN1_GPIO_Port GPIOE
#define LM_DOWN_Pin GPIO_PIN_6
#define LM_DOWN_GPIO_Port GPIOE
#define MCU_OSC32_IN_Pin GPIO_PIN_14
#define MCU_OSC32_IN_GPIO_Port GPIOC
#define MCU_OSC32_OUT_Pin GPIO_PIN_15
#define MCU_OSC32_OUT_GPIO_Port GPIOC
#define MCU_25M_OSC_IN_Pin GPIO_PIN_0
#define MCU_25M_OSC_IN_GPIO_Port GPIOH
#define MCU_25M_OSC_OUT_Pin GPIO_PIN_1
#define MCU_25M_OSC_OUT_GPIO_Port GPIOH
#define LM_SENSOR_ADC_Pin GPIO_PIN_0
#define LM_SENSOR_ADC_GPIO_Port GPIOC
#define MOTOR_THERMISTOR_Pin GPIO_PIN_1
#define MOTOR_THERMISTOR_GPIO_Port GPIOC
#define FSR_ADC_Pin GPIO_PIN_2
#define FSR_ADC_GPIO_Port GPIOC
#define LINEAR_ADC_Pin GPIO_PIN_3
#define LINEAR_ADC_GPIO_Port GPIOC
#define MOTOR_IncENC_A_Pin GPIO_PIN_0
#define MOTOR_IncENC_A_GPIO_Port GPIOA
#define MOTOR_IncENC_B_Pin GPIO_PIN_1
#define MOTOR_IncENC_B_GPIO_Port GPIOA
#define MOTOR_IncENC_Z_Pin GPIO_PIN_2
#define MOTOR_IncENC_Z_GPIO_Port GPIOA
#define MOTOR_IncENC_EN_Pin GPIO_PIN_3
#define MOTOR_IncENC_EN_GPIO_Port GPIOA
#define MOTOR_CURRENT_W_Pin GPIO_PIN_6
#define MOTOR_CURRENT_W_GPIO_Port GPIOA
#define BEMF_W_Pin GPIO_PIN_7
#define BEMF_W_GPIO_Port GPIOA
#define MOTOR_CURRENT_V_Pin GPIO_PIN_4
#define MOTOR_CURRENT_V_GPIO_Port GPIOC
#define BEMF_V_Pin GPIO_PIN_5
#define BEMF_V_GPIO_Port GPIOC
#define MOTOR_CURRENT_U_Pin GPIO_PIN_0
#define MOTOR_CURRENT_U_GPIO_Port GPIOB
#define BEMF_U_Pin GPIO_PIN_1
#define BEMF_U_GPIO_Port GPIOB
#define LM_UP_Pin GPIO_PIN_7
#define LM_UP_GPIO_Port GPIOE
#define DRV8350_PWMA_L_Pin GPIO_PIN_8
#define DRV8350_PWMA_L_GPIO_Port GPIOE
#define DRV8350_PWMA_H_Pin GPIO_PIN_9
#define DRV8350_PWMA_H_GPIO_Port GPIOE
#define DRV8350_PWMB_L_Pin GPIO_PIN_10
#define DRV8350_PWMB_L_GPIO_Port GPIOE
#define DRV8350_PWMB_H_Pin GPIO_PIN_11
#define DRV8350_PWMB_H_GPIO_Port GPIOE
#define DRV8350_PWMC_L_Pin GPIO_PIN_12
#define DRV8350_PWMC_L_GPIO_Port GPIOE
#define DRV8350_PWMC_H_Pin GPIO_PIN_13
#define DRV8350_PWMC_H_GPIO_Port GPIOE
#define DRV8350_EN_Pin GPIO_PIN_14
#define DRV8350_EN_GPIO_Port GPIOE
#define DRV8350__FAULT_Pin GPIO_PIN_15
#define DRV8350__FAULT_GPIO_Port GPIOE
#define LTC2944_I2C_SCL_Pin GPIO_PIN_10
#define LTC2944_I2C_SCL_GPIO_Port GPIOB
#define LTC2944_I2C_SDA_Pin GPIO_PIN_11
#define LTC2944_I2C_SDA_GPIO_Port GPIOB
#define NODE_ID_LOWEST_Pin GPIO_PIN_8
#define NODE_ID_LOWEST_GPIO_Port GPIOD
#define NODE_ID_LOW_Pin GPIO_PIN_9
#define NODE_ID_LOW_GPIO_Port GPIOD
#define NODE_ID_HIGH_Pin GPIO_PIN_10
#define NODE_ID_HIGH_GPIO_Port GPIOD
#define NODE_ID_HIGHEST_Pin GPIO_PIN_11
#define NODE_ID_HIGHEST_GPIO_Port GPIOD
#define NZR_LED_Pin GPIO_PIN_12
#define NZR_LED_GPIO_Port GPIOD
#define LED_BOOT_RED_Pin GPIO_PIN_13
#define LED_BOOT_RED_GPIO_Port GPIOD
#define HALL_SENSOR_H1_Pin GPIO_PIN_6
#define HALL_SENSOR_H1_GPIO_Port GPIOC
#define HALL_SENSOR_H1_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_H2_Pin GPIO_PIN_7
#define HALL_SENSOR_H2_GPIO_Port GPIOC
#define HALL_SENSOR_H2_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_H3_Pin GPIO_PIN_8
#define HALL_SENSOR_H3_GPIO_Port GPIOC
#define HALL_SENSOR_H3_EXTI_IRQn EXTI9_5_IRQn
#define _6X_I2C_SDA_Pin GPIO_PIN_9
#define _6X_I2C_SDA_GPIO_Port GPIOC
#define _6X_I2C_SCL_Pin GPIO_PIN_8
#define _6X_I2C_SCL_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_VBUS_EXTI_IRQn EXTI9_5_IRQn
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define USB_OTG_GPIO_Pin GPIO_PIN_15
#define USB_OTG_GPIO_GPIO_Port GPIOA
#define ANKLE_AbsENC_CLK_Pin GPIO_PIN_10
#define ANKLE_AbsENC_CLK_GPIO_Port GPIOC
#define ANKLE_AbsENC_DATA_Pin GPIO_PIN_11
#define ANKLE_AbsENC_DATA_GPIO_Port GPIOC
#define MCU_FDCAN_RX_Pin GPIO_PIN_0
#define MCU_FDCAN_RX_GPIO_Port GPIOD
#define MCU_FDCAN_TX_Pin GPIO_PIN_1
#define MCU_FDCAN_TX_GPIO_Port GPIOD
#define CAN_SILENT_Pin GPIO_PIN_2
#define CAN_SILENT_GPIO_Port GPIOD
#define IMU_RDY_Pin GPIO_PIN_3
#define IMU_RDY_GPIO_Port GPIOD
#define MOTOR_AbsENC_CLK_Pin GPIO_PIN_3
#define MOTOR_AbsENC_CLK_GPIO_Port GPIOB
#define MOTOR_AbsENC_DATA_Pin GPIO_PIN_4
#define MOTOR_AbsENC_DATA_GPIO_Port GPIOB
#define _3X_I2C_SCL_Pin GPIO_PIN_6
#define _3X_I2C_SCL_GPIO_Port GPIOB
#define _3X_I2C_SDA_Pin GPIO_PIN_7
#define _3X_I2C_SDA_GPIO_Port GPIOB
#define EXT_I2C_SCL_Pin GPIO_PIN_8
#define EXT_I2C_SCL_GPIO_Port GPIOB
#define EXT_I2C_SDA_Pin GPIO_PIN_9
#define EXT_I2C_SDA_GPIO_Port GPIOB
#define TP5_Pin GPIO_PIN_0
#define TP5_GPIO_Port GPIOE
#define TP6_Pin GPIO_PIN_1
#define TP6_GPIO_Port GPIOE
#endif /* L30_MD_REV08_ENABLED */

/* ########################## SUIT_MD_ENABLED ############################## */
#ifdef SUIT_MD_ENABLED
#define MCU_32K_OSC_IN_Pin GPIO_PIN_14
#define MCU_32K_OSC_IN_GPIO_Port GPIOC
#define MCU_32K_OSC_OUT_Pin GPIO_PIN_15
#define MCU_32K_OSC_OUT_GPIO_Port GPIOC
#define IMU_6X_I2C2_SDA_Pin GPIO_PIN_0
#define IMU_6X_I2C2_SDA_GPIO_Port GPIOF
#define IMU_6X_I2C2_SCL_Pin GPIO_PIN_1
#define IMU_6X_I2C2_SCL_GPIO_Port GPIOF
#define ID_SET1_Pin GPIO_PIN_2
#define ID_SET1_GPIO_Port GPIOF
#define ID_SET2_Pin GPIO_PIN_3
#define ID_SET2_GPIO_Port GPIOF
#define ID_SET3_Pin GPIO_PIN_4
#define ID_SET3_GPIO_Port GPIOF
#define ID_SET4_Pin GPIO_PIN_5
#define ID_SET4_GPIO_Port GPIOF
#define GRF_3_ADC_Pin GPIO_PIN_7
#define GRF_3_ADC_GPIO_Port GPIOF
#define GRF_4_ADC_Pin GPIO_PIN_9
#define GRF_4_ADC_GPIO_Port GPIOF
#define MCU_25M_OSC_IN_Pin GPIO_PIN_0
#define MCU_25M_OSC_IN_GPIO_Port GPIOH
#define MCU_25M_OSC_OUT_Pin GPIO_PIN_1
#define MCU_25M_OSC_OUT_GPIO_Port GPIOH
#define M_TMR_ADC_Pin GPIO_PIN_1
#define M_TMR_ADC_GPIO_Port GPIOC
#define GRF_1_ADC_Pin GPIO_PIN_2
#define GRF_1_ADC_GPIO_Port GPIOC
#define GRF_2_ADC_Pin GPIO_PIN_3
#define GRF_2_ADC_GPIO_Port GPIOC
#define MOTOR_IncENC_A_Pin GPIO_PIN_0
#define MOTOR_IncENC_A_GPIO_Port GPIOA
#define MOTOR_IncENC_B_Pin GPIO_PIN_1
#define MOTOR_IncENC_B_GPIO_Port GPIOA
#define MOTOR_IncENC_Z_Pin GPIO_PIN_2
#define MOTOR_IncENC_Z_GPIO_Port GPIOA
#define MOTOR_IncENC_EN_Pin GPIO_PIN_3
#define MOTOR_IncENC_EN_GPIO_Port GPIOA
#define MOTOR_CURRENT_W_Pin GPIO_PIN_6
#define MOTOR_CURRENT_W_GPIO_Port GPIOA
#define BEMF_W_Pin GPIO_PIN_7
#define BEMF_W_GPIO_Port GPIOA
#define MOTOR_CURRENT_V_Pin GPIO_PIN_4
#define MOTOR_CURRENT_V_GPIO_Port GPIOC
#define BEMF_V_Pin GPIO_PIN_5
#define BEMF_V_GPIO_Port GPIOC
#define MOTOR_CURRENT_U_Pin GPIO_PIN_0
#define MOTOR_CURRENT_U_GPIO_Port GPIOB
#define BEMF_U_Pin GPIO_PIN_1
#define BEMF_U_GPIO_Port GPIOB
#define RT_CHECK_Pin GPIO_PIN_7
#define RT_CHECK_GPIO_Port GPIOE
#define DRV8350_PWMA_L_Pin GPIO_PIN_8
#define DRV8350_PWMA_L_GPIO_Port GPIOE
#define DRV8350_PWMA_H_Pin GPIO_PIN_9
#define DRV8350_PWMA_H_GPIO_Port GPIOE
#define DRV8350_PWMB_L_Pin GPIO_PIN_10
#define DRV8350_PWMB_L_GPIO_Port GPIOE
#define DRV8350_PWMB_H_Pin GPIO_PIN_11
#define DRV8350_PWMB_H_GPIO_Port GPIOE
#define DRV8350_PWMC_L_Pin GPIO_PIN_12
#define DRV8350_PWMC_L_GPIO_Port GPIOE
#define DRV8350_PWMC_H_Pin GPIO_PIN_13
#define DRV8350_PWMC_H_GPIO_Port GPIOE
#define DRV8350_EN_Pin GPIO_PIN_14
#define DRV8350_EN_GPIO_Port GPIOE
#define DRV8350__FAULT_Pin GPIO_PIN_15
#define DRV8350__FAULT_GPIO_Port GPIOE
#define TP8_Pin GPIO_PIN_12
#define TP8_GPIO_Port GPIOD
#define STATUS_LED_B_Pin GPIO_PIN_13
#define STATUS_LED_B_GPIO_Port GPIOD
#define USB_OTG_GPIO_Pin GPIO_PIN_8
#define USB_OTG_GPIO_GPIO_Port GPIOA
#define USB_OTG_FS_VBUS_Pin GPIO_PIN_9
#define USB_OTG_FS_VBUS_GPIO_Port GPIOA
#define USB_OTG_FS_VBUS_EXTI_IRQn EXTI9_5_IRQn
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define RMB30_AbsENC_CLK_Pin GPIO_PIN_10
#define RMB30_AbsENC_CLK_GPIO_Port GPIOC
#define RMB30_AbsENC_DATA_Pin GPIO_PIN_11
#define RMB30_AbsENC_DATA_GPIO_Port GPIOC
#define MCU_FDCAN_RX_Pin GPIO_PIN_0
#define MCU_FDCAN_RX_GPIO_Port GPIOD
#define MCU_FDCAN_TX_Pin GPIO_PIN_1
#define MCU_FDCAN_TX_GPIO_Port GPIOD
#define TP7_Pin GPIO_PIN_2
#define TP7_GPIO_Port GPIOD
#define IMU_3X_RDY_Pin GPIO_PIN_3
#define IMU_3X_RDY_GPIO_Port GPIOD
#define TP2_Pin GPIO_PIN_6
#define TP2_GPIO_Port GPIOD
#define TP4_Pin GPIO_PIN_7
#define TP4_GPIO_Port GPIOD
#define MOTOR_AbsENC_CLK_Pin GPIO_PIN_3
#define MOTOR_AbsENC_CLK_GPIO_Port GPIOB
#define MOTOR_AbsENC_DATA_Pin GPIO_PIN_4
#define MOTOR_AbsENC_DATA_GPIO_Port GPIOB
#define IMU_3X_I2C1_SCL_Pin GPIO_PIN_6
#define IMU_3X_I2C1_SCL_GPIO_Port GPIOB
#define IMU_3X_I2C1_SDA_Pin GPIO_PIN_7
#define IMU_3X_I2C1_SDA_GPIO_Port GPIOB
#define TP5_Pin GPIO_PIN_0
#define TP5_GPIO_Port GPIOE
#define TP6_Pin GPIO_PIN_1
#define TP6_GPIO_Port GPIOE
#define HWREV_2_Pin GPIO_PIN_0
#define HWREV_2_GPIO_Port GPIOG
#define HWREV_1_Pin GPIO_PIN_1
#define HWREV_1_GPIO_Port GPIOG
#define HWREV_0_Pin GPIO_PIN_7
#define HWREV_0_GPIO_Port GPIOE

#define MAG_SENSOR_IN_Pin GPIO_PIN_13 //ROBOWEAR
#define MAG_SENSOR_IN_Port GPIOC

#define LOAD_MT_DRV_EN	GPIO_PIN_2
#define LOAD_MT_DRV_DIR	GPIO_PIN_9

#endif /* SUIT_MD_ENABLED */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
