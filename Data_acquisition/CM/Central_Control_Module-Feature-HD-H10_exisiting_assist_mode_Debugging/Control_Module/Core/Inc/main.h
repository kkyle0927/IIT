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
void Init_Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/


/* USER CODE BEGIN Private defines */
#define BLE_BT_CONNECT_Pin GPIO_PIN_2
#define BLE_BT_CONNECT_GPIO_Port GPIOE
#define BLE_BT_CONNECT_EXTI_IRQn EXTI2_IRQn
#define BLE_BT_BLE_RDY_Pin GPIO_PIN_3
#define BLE_BT_BLE_RDY_GPIO_Port GPIOE
#define AUD_nSDMODE_Pin GPIO_PIN_6
#define AUD_nSDMODE_GPIO_Port GPIOE
#define I2C2_SDA_Pin GPIO_PIN_0
#define I2C2_SDA_GPIO_Port GPIOF
#define I2C2_SCL_Pin GPIO_PIN_1
#define I2C2_SCL_GPIO_Port GPIOF
#define SW_IC_INT_Pin GPIO_PIN_3
#define SW_IC_INT_GPIO_Port GPIOF
#define SW_IC_INT_EXTI_IRQn EXTI3_IRQn
#define RTC_SPI_MISO_Pin GPIO_PIN_2
#define RTC_SPI_MISO_GPIO_Port GPIOC
#define RTC_SPI_MOSI_Pin GPIO_PIN_3
#define RTC_SPI_MOSI_GPIO_Port GPIOC
#define MCU_USB_OC__DET_Pin GPIO_PIN_5
#define MCU_USB_OC__DET_GPIO_Port GPIOA
#define USB_CC1_ADC_Pin GPIO_PIN_11
#define USB_CC1_ADC_GPIO_Port GPIOF
#define USB_CC2_ADC_Pin GPIO_PIN_12
#define USB_CC2_ADC_GPIO_Port GPIOF
#define LTC2944_I2C_SCL_Pin GPIO_PIN_14
#define LTC2944_I2C_SCL_GPIO_Port GPIOF
#define LTC2944_I2C_SDA_Pin GPIO_PIN_15
#define LTC2944_I2C_SDA_GPIO_Port GPIOF
#define HWREV_2_Pin GPIO_PIN_0
#define HWREV_2_GPIO_Port GPIOG
#define HWREV_1_Pin GPIO_PIN_1
#define HWREV_1_GPIO_Port GPIOG
#define HWREV_0_Pin GPIO_PIN_7
#define HWREV_0_GPIO_Port GPIOE
#define ESP32_FW_UPDATE_Pin GPIO_PIN_8
#define ESP32_FW_UPDATE_GPIO_Port GPIOE
#define WIFI_SPI_RDY_Pin GPIO_PIN_9
#define WIFI_SPI_RDY_GPIO_Port GPIOE
#define WIFI_SPI_RDY_EXTI_IRQn EXTI9_5_IRQn
#define BLE_RESET_Pin GPIO_PIN_11
#define BLE_RESET_GPIO_Port GPIOE
#define RTC_SPI_SCK_Pin GPIO_PIN_10
#define RTC_SPI_SCK_GPIO_Port GPIOB
#define MC_FDCAN2_RX_Pin GPIO_PIN_12
#define MC_FDCAN2_RX_GPIO_Port GPIOB
#define MC_FDCAN2_TX_Pin GPIO_PIN_13
#define MC_FDCAN2_TX_GPIO_Port GPIOB
#define BLE_UART_TX_Pin GPIO_PIN_14
#define BLE_UART_TX_GPIO_Port GPIOB
#define BLE_UART_RX_Pin GPIO_PIN_15
#define BLE_UART_RX_GPIO_Port GPIOB
#define LED_DRV_nRESET_Pin GPIO_PIN_8
#define LED_DRV_nRESET_GPIO_Port GPIOD
#define LED_DRV_nOE_Pin GPIO_PIN_9
#define LED_DRV_nOE_GPIO_Port GPIOD
#define ASSIST_BTN_P_Pin GPIO_PIN_10
#define ASSIST_BTN_P_GPIO_Port GPIOD
#define ASSIST_BTN_P_EXTI_IRQn EXTI15_10_IRQn
#define ASSIST_BTN_N_Pin GPIO_PIN_12
#define ASSIST_BTN_N_GPIO_Port GPIOD
#define ASSIST_BTN_N_EXTI_IRQn EXTI15_10_IRQn
#define SDCARD_DET_Pin GPIO_PIN_13
#define SDCARD_DET_GPIO_Port GPIOD
#define SDCARD_nWP_Pin GPIO_PIN_14
#define SDCARD_nWP_GPIO_Port GPIOD
#define MCU_24V_MOTOR_ON_Pin GPIO_PIN_2
#define MCU_24V_MOTOR_ON_GPIO_Port GPIOG
#define BLE_3V3_PWR_EN_Pin GPIO_PIN_4
#define BLE_3V3_PWR_EN_GPIO_Port GPIOG
#define SDCARD_3V3_PWR_EN_Pin GPIO_PIN_5
#define SDCARD_3V3_PWR_EN_GPIO_Port GPIOG
#define MCU_SW_CLR_Pin GPIO_PIN_7
#define MCU_SW_CLR_GPIO_Port GPIOG
#define PB_IN_MCU_Pin GPIO_PIN_8
#define PB_IN_MCU_GPIO_Port GPIOG
#define USB_OTG_GPIO_Pin GPIO_PIN_8
#define USB_OTG_GPIO_GPIO_Port GPIOA
#define MCU_USB_FS_VBUS_Pin GPIO_PIN_9
#define MCU_USB_FS_VBUS_GPIO_Port GPIOA
#define MCU_USB_FS_ID_Pin GPIO_PIN_10
#define MCU_USB_FS_ID_GPIO_Port GPIOA
#define MCU_USB_FS_DM_Pin GPIO_PIN_11
#define MCU_USB_FS_DM_GPIO_Port GPIOA
#define MCU_USB_FS_DP_Pin GPIO_PIN_12
#define MCU_USB_FS_DP_GPIO_Port GPIOA
#define ESP32_SPI_NSS_Pin GPIO_PIN_15
#define ESP32_SPI_NSS_GPIO_Port GPIOA
#define MC_FDCAN1_RX_Pin GPIO_PIN_0
#define MC_FDCAN1_RX_GPIO_Port GPIOD
#define MC_FDCAN1_TX_Pin GPIO_PIN_1
#define MC_FDCAN1_TX_GPIO_Port GPIOD
#define RTC_nINT_Pin GPIO_PIN_4
#define RTC_nINT_GPIO_Port GPIOD
#define MCU_USB_PWR_ON_Pin GPIO_PIN_6
#define MCU_USB_PWR_ON_GPIO_Port GPIOD
#define LED_DRIVE_SPI_MOSI_Pin GPIO_PIN_7
#define LED_DRIVE_SPI_MOSI_GPIO_Port GPIOD
#define LED_DRIVE_SPI_MISO_Pin GPIO_PIN_9
#define LED_DRIVE_SPI_MISO_GPIO_Port GPIOG
#define LED_DRIVE_SPI_NSS_Pin GPIO_PIN_10
#define LED_DRIVE_SPI_NSS_GPIO_Port GPIOG
#define LED_DRIVE_SPI_SCK_Pin GPIO_PIN_11
#define LED_DRIVE_SPI_SCK_GPIO_Port GPIOG
#define MC_24V_PWR_EN_Pin GPIO_PIN_12
#define MC_24V_PWR_EN_GPIO_Port GPIOG
#define ESP32_SPI_SCK_Pin GPIO_PIN_3
#define ESP32_SPI_SCK_GPIO_Port GPIOB
#define ESP32_SPI_MISO_Pin GPIO_PIN_4
#define ESP32_SPI_MISO_GPIO_Port GPIOB
#define ESP32_SPI_MOSI_Pin GPIO_PIN_5
#define ESP32_SPI_MOSI_GPIO_Port GPIOB
#define IMU_3AXIS_I2C_SCL_Pin GPIO_PIN_6
#define IMU_3AXIS_I2C_SCL_GPIO_Port GPIOB
#define IMU_3AXIS_I2C_SDA_Pin GPIO_PIN_7
#define IMU_3AXIS_I2C_SDA_GPIO_Port GPIOB
#define IMU_3AXIS_I2C_RDY_Pin GPIO_PIN_8
#define IMU_3AXIS_I2C_RDY_GPIO_Port GPIOB
#define RTC_SPI_NSS_Pin GPIO_PIN_9
#define RTC_SPI_NSS_GPIO_Port GPIOB
#define BLE_SYS_CHECK_Pin GPIO_PIN_0
#define BLE_SYS_CHECK_GPIO_Port GPIOE
#define WIFI_CONNECT_Pin GPIO_PIN_1
#define WIFI_CONNECT_GPIO_Port GPIOE
#define WIFI_CONNECT_EXTI_IRQn EXTI1_IRQn
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
