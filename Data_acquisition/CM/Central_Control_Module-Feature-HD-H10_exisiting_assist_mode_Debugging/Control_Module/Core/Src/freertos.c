/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "module.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS_ble_comm_hdlr.h"
#include "AS_debug_ctrl.h"
#include "AS_ext_dev_ctrl.h"
#include "AS_gait_ctrl.h"
#include "AS_imu_ctrl.h"
#include "AS_system_ctrl.h"
#include "AS_whole_body_ctrl.h"
#include "AS_data_ctrl.h"
#include "AS_Audio_Ctrl.h"
#include "AS_RiskMngt_Hdlr.h"
#include "AS_wifi_comm_hdlr.h"

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

/* USER CODE END Variables */
/* Definitions for App_Startup */
osThreadId_t App_StartupHandle;
const osThreadAttr_t App_Startup_attributes = {
  .name = "App_Startup",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

#ifdef _USE_DEBUG_CLI
/* Definitions for Debug_Ctrl */
 osThreadId_t Debug_CtrlHandle;
 const osThreadAttr_t Debug_Ctrl_attributes = {
   .name = "Debug_Ctrl",
   .stack_size = 1024 * 4,
   .priority = (osPriority_t) osPriorityNormal,
   //.priority = (osPriority_t) osPriorityLow,
 };
#else
/* Definitions for Risk_Mng */
osThreadId_t Risk_MngHandle;
const osThreadAttr_t Risk_Mng_attributes = {
  .name = "Risk_Mng",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};

/* Definitions for System_Ctrl */
osThreadId_t System_CtrlHandle;
const osThreadAttr_t System_Ctrl_attributes = {
  .name = "System_Ctrl",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};

/* Definitions for Whole_Body_Ctrl */
osThreadId_t Whole_Body_CtrlHandle;
const osThreadAttr_t Whole_Body_Ctrl_attributes = {
  .name = "Whole_Body_Ctrl",
  .stack_size = 4096 * 6,
  .priority = (osPriority_t) osPriorityRealtime6,
};

/* Definitions for Gait_Ctrl */
osThreadId_t Gait_CtrlHandle;
const osThreadAttr_t Gait_Ctrl_attributes = {
  .name = "Gait_Ctrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};

/* Definitions for Data_Ctrl */
osThreadId_t Data_CtrlHandle;
const osThreadAttr_t Data_Ctrl_attributes = {
  .name = "Data_Ctrl",
  //.stack_size = 1024 * 1,
  .stack_size = 1024 * 3,
  .priority = (osPriority_t) osPriorityHigh6,
};

/* Definitions for Audio_Ctrl */
osThreadId_t Audio_CtrlHandle;
const osThreadAttr_t Audio_Ctrl_attributes = {
  .name = "Audio_Ctrl",
  .stack_size = 512 * 1,
  .priority = (osPriority_t) osPriorityNormal,
  //.priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for BLE_Comm_Hdlr */
osThreadId_t BLE_Comm_HdlrHandle;
const osThreadAttr_t BLE_Comm_Hdlr_attributes = {
  .name = "BLE_Comm_Hdlr",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t WiFi_Comm_HdlrHandle;
const osThreadAttr_t WiFi_Comm_Hdlr_attributes = {
  .name = "WiFi_Comm_Hdlr",
  .stack_size = 1024 * 1,
  //.priority = (osPriority_t) osPriorityNormal,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for Ext_Dev_Ctrl */
osThreadId_t Ext_Dev_CtrlHandle;
const osThreadAttr_t Ext_Dev_Ctrl_attributes = {
	.name = "Ext_Dev_Ctrl",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for IMU_Ctrl */
//osThreadId_t IMU_CtrlHandle;
//const osThreadAttr_t IMU_Ctrl_attributes = {
//  .name = "IMU_Ctrl",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};

#endif /*_USE_DEBUG_CLI*/

/* Definitions for BinSem_I2C1 */
osSemaphoreId_t BinSem_I2C1Handle;
const osSemaphoreAttr_t BinSem_I2C1_attributes = {
  .name = "BinSem_I2C1"
};
/* Definitions for BinSem_I2C2 */
osSemaphoreId_t BinSem_I2C2Handle;
const osSemaphoreAttr_t BinSem_I2C2_attributes = {
  .name = "BinSem_I2C2"
};
/* Definitions for BinSem_I2C3 */
osSemaphoreId_t BinSem_I2C3Handle;
const osSemaphoreAttr_t BinSem_I2C3_attributes = {
  .name = "BinSem_I2C3"
};
/* Definitions for BinSem_I2C4 */
osSemaphoreId_t BinSem_I2C4Handle;
const osSemaphoreAttr_t BinSem_I2C4_attributes = {
  .name = "BinSem_I2C4"
};
/* Definitions for BinSem_SPI1 */
osSemaphoreId_t BinSem_PlaySoundHandle;
const osSemaphoreAttr_t BinSem_PlaySound_attributes = {
  .name = "BinSem_PlaySound"
};
/* Definitions for BinSem_SPI2 */
osSemaphoreId_t BinSem_SPI2Handle;
const osSemaphoreAttr_t BinSem_SPI2_attributes = {
  .name = "BinSem_SPI2"
};
/* Definitions for BinSem_SPI3 */
osSemaphoreId_t BinSem_SPI3Handle;
const osSemaphoreAttr_t BinSem_SPI3_attributes = {
  .name = "BinSem_SPI3"
};
/* Definitions for BinSem_UART1 */
osSemaphoreId_t BinSem_UART1Handle;
const osSemaphoreAttr_t BinSem_UART1_attributes = {
  .name = "BinSem_UART1"
};
///* Definitions for BinSem_SDMMC1 */
osSemaphoreId_t BinSem_PlayBeepHandle;
const osSemaphoreAttr_t BinSem_PlayBeep_attributes = {
  .name = "BinSem_PlayBeep"
};
///* Definitions for BinSem_SAI1 */
//osSemaphoreId_t BinSem_SAI1Handle;
//const osSemaphoreAttr_t BinSem_SAI1_attributes = {
//  .name = "BinSem_SAI1"
//};
/* Definitions for BinSem_FDCAN1 */
osSemaphoreId_t BinSem_FDCAN1Handle;
const osSemaphoreAttr_t BinSem_FDCAN1_attributes = {
  .name = "BinSem_FDCAN1"
};
/* Definitions for BinSem_FDCAN2 */
osSemaphoreId_t BinSem_FDCAN2Handle;
const osSemaphoreAttr_t BinSem_FDCAN2_attributes = {
  .name = "BinSem_FDCAN2"
};
/* Definitions for BinSem_SPI4 */
osSemaphoreId_t BinSem_SPI1Handle;
const osSemaphoreAttr_t BinSem_SPI1_attributes = {
  .name = "BinSem_SPI1"
};
/* Definitions for BinSem_SPI5 */
osSemaphoreId_t BinSem_SPI5Handle;
const osSemaphoreAttr_t BinSem_SPI5_attributes = {
  .name = "BinSem_SPI5"
};
/* Definitions for BinSem_SPI6 */
osSemaphoreId_t BinSem_SPI6Handle;
const osSemaphoreAttr_t BinSem_SPI6_attributes = {
  .name = "BinSem_SPI6"
};

osSemaphoreId_t sdio_sync_semaphore;
const osSemaphoreAttr_t sdio_sync_semaphoreHandle_attributes = {
  .name = "SDIO_Sync"
};

/* Definitions for SDIOmutex */
osMutexId_t SDIOmutexHandle;
const osMutexAttr_t SDIOmutex_attributes = {
  .name = "SDIOmutex"
};
/* Definitions for SoundMutex */
osMutexId_t SoundMutexHandle;
const osMutexAttr_t SoundMutex_attributes = {
  .name = "SoundMutex"
};

/* Definitions for SD Buffering Mutex */
osMutexId_t mutex_sd_buffering;
const osMutexAttr_t mutex_sd_buffering_attributes = {
  .name = "Mutex_SD_Buffering"
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartupAPP(void *argument);
void StartSystemCtrl(void *argument);
void StartDebugCtrl(void *argument);
void StartExtDevCtrl(void *argument);
void StartGaitCtrl(void *argument);
void StartIMUCtrl(void *argument);
void StartWholeBodyCtrl(void *argument);
void StartBLEComm(void *argument);
void StartDataCtrl(void *argument);
void StartAudioCtrl(void *argument);
void StartRiskMngtCtrl(void *argument);
void StartWiFiComm(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  
	/* Create the mutex(es) */
	/* creation of SDIOmutex */
	SDIOmutexHandle = osMutexNew(&SDIOmutex_attributes);

	/* creation of SoundMutex */
	SoundMutexHandle = osMutexNew(&SoundMutex_attributes);

  /* creation of SD Buffering Mutex */
	mutex_sd_buffering = osMutexNew(&mutex_sd_buffering_attributes);

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinSem_I2C1 */
  BinSem_I2C1Handle = osSemaphoreNew(1, 1, &BinSem_I2C1_attributes);

  /* creation of BinSem_I2C2 */
  BinSem_I2C2Handle = osSemaphoreNew(1, 1, &BinSem_I2C2_attributes);

  /* creation of BinSem_I2C3 */
  BinSem_I2C3Handle = osSemaphoreNew(1, 1, &BinSem_I2C3_attributes);

  /* creation of BinSem_I2C4 */
  BinSem_I2C4Handle = osSemaphoreNew(1, 1, &BinSem_I2C4_attributes);

  /* creation of BinSem_PlaySound */
  BinSem_PlaySoundHandle = osSemaphoreNew(1, 0, &BinSem_PlaySound_attributes);

  /* creation of BinSem_SPI2 */
  BinSem_SPI2Handle = osSemaphoreNew(1, 1, &BinSem_SPI2_attributes);

  /* creation of BinSem_SPI3 */
  BinSem_SPI3Handle = osSemaphoreNew(1, 1, &BinSem_SPI3_attributes);

  /* creation of BinSem_UART7 */
  BinSem_UART1Handle = osSemaphoreNew(1, 1, &BinSem_UART1_attributes);

//  /* creation of BinSem_SDMMC1 */
  BinSem_PlayBeepHandle = osSemaphoreNew(1, 0, &BinSem_PlayBeep_attributes);
//
//  /* creation of BinSem_SAI1 */
//  BinSem_SAI1Handle = osSemaphoreNew(1, 1, &BinSem_SAI1_attributes);

  /* creation of BinSem_FDCAN1 */
  BinSem_FDCAN1Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN1_attributes);

  /* creation of BinSem_FDCAN2 */
  BinSem_FDCAN2Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN2_attributes);

  /* creation of BinSem_SPI1 */
  BinSem_SPI1Handle = osSemaphoreNew(1, 1, &BinSem_SPI1_attributes);

  /* creation of BinSem_SPI5 */
  BinSem_SPI5Handle = osSemaphoreNew(1, 1, &BinSem_SPI5_attributes);

  /* creation of BinSem_SPI6 */
  BinSem_SPI6Handle = osSemaphoreNew(1, 1, &BinSem_SPI6_attributes);

  sdio_sync_semaphore = osSemaphoreNew(1, 0, &sdio_sync_semaphoreHandle_attributes);			// sync semaphore create

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* creation of App_Startup */
  App_StartupHandle = osThreadNew(StartupAPP, NULL, &App_Startup_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

void StartupAPP(void *argument)
{
	/* Peripheral Initialize*/
	IOIF_SYS_CtrlInit();

	if(IOIF_LED24chDriverInit(IOIF_SPI1) != IOIF_LED_STATUS_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_LED] = true;
	}

	if(IOIF_BatMonitor_Init()!= IOIF_BATMONITOR_STATUS_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_BAT] = true;
	}

	if(IOIF_Magneto_Init()!= IOIF_MAGNETO_STATUS_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_MAGNETO] = true;
	}

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_5, IOIF_GPIO_PIN_SET); //Enable SD Card Power On
	osDelay(10);
	if(IOIF_FATFS_SD_Init(IOIF_SD1, (uint8_t*) "0:") != IOIF_MSD_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_SDCARD] = true;
	}

	if(IOIF_SAI_WavAudioFSInit((uint8_t*) "1:", IOIF_SAI_WAVPLAY_SD) != IOIF_SAI_WAV_STATUS_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_SDCARD] = true;
	}

	if (IOIF_ESP32_Init() != true)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_ESP32] = true;
	}

	if (IOIF_InitFDCAN1(NODE_ID_CM) != IOIF_FDCAN_STATUS_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_FDCAN_CH1] = true;
	}
	if (IOIF_InitFDCAN2(NODE_ID_CM) != IOIF_FDCAN_STATUS_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_FDCAN_CH2] = true;
	}

#if defined(_USE_DEBUG_CLI) || defined(USB_LOG_ENABLED)
	IOIF_InitUSB(IOIF_USBD_CDC, IOIF_USB_TIM_NULL);
#else
	if(IOIF_InitUSB(IOIF_USBD_MSC, IOIF_USB_TIM_NULL) != IOIF_USB_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_USB] = true;
	}
#endif

	if(IOIF_MCP79510_Init() != IOIF_MCP79510_STATE_OK)
	{
		init_fault[INIT_FAULT_OCCUR] = true;
		init_fault[INIT_FAULT_RTC] = true;
	}

	/* Creation of Threads */
#ifdef _USE_DEBUG_CLI
	/* creation of Debug_Ctrl */
	Debug_CtrlHandle = osThreadNew(StartDebugCtrl, NULL, &Debug_Ctrl_attributes);
#else /*_USE_DEBUG_CLI*/
	Risk_MngHandle 			= osThreadNew(StartRiskMngtCtrl, NULL, &Risk_Mng_attributes);
	System_CtrlHandle 		= osThreadNew(StartSystemCtrl, NULL, &System_Ctrl_attributes);
	Whole_Body_CtrlHandle	= osThreadNew(StartWholeBodyCtrl, NULL, &Whole_Body_Ctrl_attributes);
	Gait_CtrlHandle 		= osThreadNew(StartGaitCtrl, NULL, &Gait_Ctrl_attributes);
	Data_CtrlHandle 		= osThreadNew(StartDataCtrl, NULL, &Data_Ctrl_attributes);
	Audio_CtrlHandle 		= osThreadNew(StartAudioCtrl, NULL, &Audio_Ctrl_attributes);
	BLE_Comm_HdlrHandle 	= osThreadNew(StartBLEComm, NULL, &BLE_Comm_Hdlr_attributes);
	WiFi_Comm_HdlrHandle 	= osThreadNew(StartWiFiComm, NULL, &WiFi_Comm_Hdlr_attributes);
	Ext_Dev_CtrlHandle 		= osThreadNew(StartExtDevCtrl, NULL, &Ext_Dev_Ctrl_attributes);
	//  IMU_CtrlHandle = osThreadNew(StartIMUCtrl, NULL, &IMU_Ctrl_attributes)
#endif /*_USE_DEBUG_CLI*/

	/*Task Delete */
	vTaskDelete(NULL); // NULL은 현재 태스크를 의미
}


TickType_t xRiskMngtCtrlLastWakeTime;
const TickType_t xRiskMngtTimeIncrement = pdMS_TO_TICKS(10); // 1ms interval for RiskMng

void StartRiskMngtCtrl(void *argument)
{
  /* USER CODE BEGIN StartSystemCtrl */
	InitRiskMngtHdlr();

  // Initialize xWholeBodyCtrlLastWakeTime with the current FreeRTOS time
	xRiskMngtCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  RunRiskMngtHdlr();
    // Wait for the next task
	  vTaskDelayUntil(&xRiskMngtCtrlLastWakeTime, xRiskMngtTimeIncrement);
  }
  /* USER CODE END StartSystemCtrl */
}

/* USER CODE BEGIN Header_StartSystemCtrl */
TickType_t xSystemCtrlLastWakeTime;
const TickType_t xSystemCtrlTimeIncrement = pdMS_TO_TICKS(10); // 10ms interval for SystemCtrl, 바꾸지 말 것

/**
  * @brief  Function implementing the System_Ctrl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSystemCtrl */

void StartSystemCtrl(void *argument)
{
  /* USER CODE BEGIN StartSystemCtrl */
  InitSysCtrl();

  // Initialize xWholeBodyCtrlLastWakeTime with the current FreeRTOS time
  xSystemCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    RunSysCtrl();
    // Wait for the next task
	  vTaskDelayUntil(&xSystemCtrlLastWakeTime, xSystemCtrlTimeIncrement);
  }
  /* USER CODE END StartSystemCtrl */
}

/* USER CODE BEGIN Header_StartDebugCtrl */
TickType_t xDebugCtrlLastWakeTime;
const TickType_t xDebugCtrlTimeIncrement = pdMS_TO_TICKS(1); // 1ms interval for DebugCtrl, 바꾸지 말 것
/**
* @brief Function implementing the Debug_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugCtrl */
void StartDebugCtrl(void *argument)
{
  /* USER CODE BEGIN StartDebugCtrl */
   InitDebugTask();

  // Initialize xDebugCtrlLastWakeTime with the current FreeRTOS time
  xDebugCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
     RunDebugTask();
    // Wait for the next task
	  vTaskDelayUntil(&xDebugCtrlLastWakeTime, xDebugCtrlTimeIncrement);
  }
  /* USER CODE END StartDebugCtrl */
}

/* USER CODE BEGIN Header_StartExtDevCtrl */
TickType_t xExtDevCtrlLastWakeTime;
const TickType_t xExtDevCtrlTimeIncrement = pdMS_TO_TICKS(10); // 10ms interval for ExtDevCtrl
/**
* @brief Function implementing the Ext_Dev_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExtDevCtrl */
void StartExtDevCtrl(void *argument)
{
  /* USER CODE BEGIN StartExtDevCtrl */
   InitExtDevCtrl();

  // Initialize xExtDevCtrlLastWakeTime with the current FreeRTOS time
  xExtDevCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
     RunExtDevCtrl();
    // Wait for the next task
	  vTaskDelayUntil(&xExtDevCtrlLastWakeTime, xExtDevCtrlTimeIncrement);
  }
  /* USER CODE END StartExtDevCtrl */
}

/* USER CODE BEGIN Header_StartGaitCtrl */
TickType_t xGaitCtrlLastWakeTime;
const TickType_t xGaitCtrlTimeIncrement = pdMS_TO_TICKS(100); // 1ms interval for GaitCtrl
/**
* @brief Function implementing the Gait_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGaitCtrl */
void StartGaitCtrl(void *argument)
{
  /* USER CODE BEGIN StartGaitCtrl */
  InitGaitCtrl();

  // Initialize xGaitCtrlLastWakeTime with the current FreeRTOS time
  xGaitCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    RunGaitCtrl();
    // Wait for the next task
	  vTaskDelayUntil(&xGaitCtrlLastWakeTime, xGaitCtrlTimeIncrement);
  }
  /* USER CODE END StartGaitCtrl */
}

/* USER CODE BEGIN Header_StartIMUCtrl */
TickType_t xIMUCtrlLastWakeTime;
const TickType_t xIMUCtrlTimeIncrement = pdMS_TO_TICKS(1); // 1ms interval for IMUCtrl
/**
* @brief Function implementing the IMU_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUCtrl */
void StartIMUCtrl(void *argument)
{
  /* USER CODE BEGIN StartIMUCtrl */
  InitIMUCtrl();

  // Initialize xIMUCtrlLastWakeTime with the current FreeRTOS time
  xIMUCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    RunIMUCtrl();
    // Wait for the next task
	  vTaskDelayUntil(&xIMUCtrlLastWakeTime, xIMUCtrlTimeIncrement);
  }
  /* USER CODE END StartIMUCtrl */
}

/* USER CODE BEGIN Header_StartWholeBodyCtrl */
TickType_t xWholeBodyCtrlLastWakeTime;
const TickType_t xWholeBodyCtrlTimeIncrement = pdMS_TO_TICKS(1); // 1ms interval for WholeBodyCtrl
/**
* @brief Function implementing the Whole_Body_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWholeBodyCtrl */
void StartWholeBodyCtrl(void *argument)
{
  /* USER CODE BEGIN StartWholeBodyCtrl */
  InitWholeBodyCtrl();

  // Initialize xWholeBodyCtrlLastWakeTime with the current FreeRTOS time
  xWholeBodyCtrlLastWakeTime = xTaskGetTickCount();
  
  /* Infinite loop */
  for(;;)
  {
    RunWholeBodyCtrl();
    // Wait for the next task
	  vTaskDelayUntil(&xWholeBodyCtrlLastWakeTime, xWholeBodyCtrlTimeIncrement);
  }
  /* USER CODE END StartWholeBodyCtrl */
}

/* USER CODE BEGIN Header_StartBLEComm */
TickType_t xBLECommHdlrLastWakeTime;
const TickType_t xBLECommHdlrPeriod = pdMS_TO_TICKS(100); // (x)ms period for BLECommHdlr
/**
* @brief Function implementing the BLE_Comm_Hdlr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBLEComm */
void StartBLEComm(void *argument)
{
  /* USER CODE BEGIN StartBLEComm */
  InitBLEComm();

  // Define the task's initial and period values
  xBLECommHdlrLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    RunBLEComm();
    // Delay until the next period
	  vTaskDelayUntil(&xBLECommHdlrLastWakeTime, xBLECommHdlrPeriod);
  }
  /* USER CODE END StartBLEComm */
}

/* USER CODE BEGIN Header_StartDataCtrl */
TickType_t xDataCtrlLastWakeTime;
const TickType_t xDataCtrlPeriod = pdMS_TO_TICKS(100); // (x)ms period for DataCtrl
/**
* @brief Function implementing the Data_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBLEComm */
void StartDataCtrl(void *argument)
{
  /* USER CODE BEGIN StartBLEComm */
  InitDataCtrl();

  // Define the task's initial and period values
  xDataCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    RunDataCtrl();
    // Delay until the next period
	  vTaskDelayUntil(&xDataCtrlLastWakeTime, xDataCtrlPeriod);
  }
  /* USER CODE END StartBLEComm */
}

TickType_t xAudioCtrlLastWakeTime;
const TickType_t xAudioCtrlTimeIncrement = pdMS_TO_TICKS(10); // 10ms interval for SystemCtrl

void StartAudioCtrl(void *argument)
{
  /* USER CODE BEGIN StartSystemCtrl */
	InitAudioTask();

  // Initialize xWholeBodyCtrlLastWakeTime with the current FreeRTOS time
	xAudioCtrlLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  RunAudioTask();
    // Wait for the next task
	  vTaskDelayUntil(&xAudioCtrlLastWakeTime, xAudioCtrlTimeIncrement);
  }
  /* USER CODE END StartSystemCtrl */
}


TickType_t xWiFiCommLastWakeTime;
const TickType_t xWiFiCommTimeIncrement = pdMS_TO_TICKS(10);

void StartWiFiComm(void *argument)
{
  /* USER CODE BEGIN StartSystemCtrl */
	InitWiFiComm();

	xWiFiCommLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  RunWiFiComm();
    // Wait for the next task
	  vTaskDelayUntil(&xWiFiCommLastWakeTime, xWiFiCommTimeIncrement);
  }
  /* USER CODE END StartSystemCtrl */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
