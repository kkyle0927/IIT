/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef L30_MD_REV06_ENABLED
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "ext_dev_ctrl.h"
#include "low_level_ctrl.h"
#include "mid_level_ctrl.h"
#include "msg_hdlr.h"
#include "system_ctrl.h"
#include "gait_ctrl.h"
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "ext_dev_ctrl.h"
#include "low_level_ctrl.h"
#include "mid_level_ctrl.h"
#include "msg_hdlr.h"
#include "system_ctrl.h"
#include "gait_ctrl.h"
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "ext_dev_ctrl.h"
#include "low_level_ctrl.h"
#include "mid_level_ctrl.h"
#include "msg_hdlr.h"
#include "system_ctrl.h"
#include "gait_ctrl.h"
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "ext_dev_ctrl.h"
#include "low_level_ctrl.h"
#include "mid_level_ctrl.h"
#include "msg_hdlr.h"
#include "system_ctrl.h"
#include "gait_ctrl.h"
#endif /* SUIT_MD_ENABLED */

#ifdef DAC_ENABLE
#include "dac.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define READ_FLASH_ARRAY_SIZE 8 // 32 Byte Aligned
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MD_UPDATE_FLAG ((uint32_t*)0x38000000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* For FLASH READ/WRITE Addr Debugging */
uint32_t writeAddr = 0;
uint32_t readAddr = 0;

extern uint32_t _flash_fw_start;
extern uint32_t _flash_fw_finish;

uint32_t fw_bin_size = 0;

#ifdef _USE_DEBUG_CLI
extern IOIF_AbsEnc_t cliAbs1Data;
extern IOIF_AbsEnc_t cliAbs2Data;
#endif /* _USE_DEBUG_CLI */

//For FDCAN Test
FDCAN_HandleTypeDef hfdcan;
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxData0[64] = {
		0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
		0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
		0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
		0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
		0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
		0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
		0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40
};

uint8_t TxData1[64] = {
		0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
		0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50,
		0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
		0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60,
		0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
		0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70,
		0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
		0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80
};

uint8_t TxData2[64] = {
		0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
		0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90,
		0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
		0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0,
		0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
		0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0,
		0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
		0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0
};

uint8_t RxData0[64];
uint8_t RxData1[64];
uint8_t RxData2[64];
__IO uint8_t UserButtonClickEvent = RESET;

static int8_t t_passfail=0;

/* ------------------- FLASH Data Save ------------------- */
Flash_Sensor_t flashsensorObj __attribute__((section(".sensorcaldata")));

uint32_t MDUpdateFlag_Value = 0;

uint32_t FW_Update_Flag = 0;
uint32_t FW_Backup_Flag = 0;
uint32_t FW_EOT_Pass_Flag = 0;
//uint32_t FW_Copy_Flag = 0;
uint32_t EOT1=0;
uint32_t MD_bin_size=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void UploadProperties();
void SaveProperties();
void DownloadProperties();
void SaveProperties_NeutralPosture();

uint32_t ReadMDUpdateFlag();
void MD_ReadFlags();

#ifdef _USE_DEBUG_CLI
static void CLI_IOIF_ModuleInit(void);
#endif /* _USE_DEBUG_CLI */

//For FDCAN test
static uint32_t BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void FDCAN_Test_Start();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  fw_bin_size = (uint32_t)&_flash_fw_finish - (uint32_t)&_flash_fw_start;
  MD_fw_binary_size = fw_bin_size;

  /* USER CODE BEGIN 1 */
	__enable_irq();
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
/* ########################## L30_MD_REV06_ENABLED ############################## */
#ifdef L30_MD_REV06_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
#endif /* L30_MD_REV06_ENABLED */

/* ########################## L30_MD_REV07_ENABLED ############################## */
#ifdef L30_MD_REV07_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
#endif /* L30_MD_REV07_ENABLED */

/* ########################## L30_MD_REV08_ENABLED ############################## */
#ifdef L30_MD_REV08_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_SPI5_Init();
#endif /* L30_MD_REV08_ENABLED */

/* ########################## SUIT_MD_ENABLED ############################## */
#ifdef SUIT_MD_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
#ifndef USE_FSR_SENSOR
  MX_TIM17_Init();
#endif
#endif /* SUIT_MD_ENABLED */
  /* USER CODE END SysInit */

#ifdef DAC_ENABLE
  MX_DAC1_Init();
  DAC1_Init_User_Debug();
#endif

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  DownloadProperties();

  DOP_CreateSDOTable();
  DOP_CreatePDOTable();

  MS_enum = IDLE;

  InitExtDevCtrl();
  InitSysMngtTask();
#ifndef _USE_DEBUG_CLI
  InitMsgHdlr();
#endif /* !_USE_DEBUG_CLI */
  InitMidLvCtrl();
  InitLowLvCtrl();
  InitGaitCtrl();

  GetMES();

  MD_ReadFlags();

  if((FW_Update_Flag == 1)&&(FW_Backup_Flag == 1)){
	  EOT1=1;
 	}

#ifdef _USE_DEBUG_CLI
    IOIF_InitUSB(IOIF_USBD_CDC, IOIF_USB_TIM_NULL);		// Init. USB
	CLI_Init();											// Init. CLI
	CLI_IOIF_ModuleInit();								// Init. CLI Modules
	cliAbs1Data.offset=AbsObj1.offset;
	cliAbs2Data.offset=AbsObj2.offset;
	cliAbs1Data.sign=AbsObj1.sign;
	cliAbs2Data.sign=AbsObj2.sign;
#endif /* _USE_DEBUG_CLI */

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
  IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_TEST_Pin, IOIF_GPIO_PIN_SET);
#endif

#ifdef SUIT_MD_ENABLED
  IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_SET); // STATUS_LED_B_Pin
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  if(EOT1==1){
		  Send_EOT(1);
		  EOT1=0;
	  }

#ifdef _USE_DEBUG_CLI
	  CLI_Run();
#else
    if(MS_enum == IDLE)                         				continue;
    else if(MS_enum == UPLOAD_PROPERTIES)       				UploadProperties();
    else if(MS_enum == SAVE_PROPERTIES)         				SaveProperties();
    else if(MS_enum == DOWNLOAD_PROPERTIES)     				DownloadProperties();
    else if(MS_enum == ELECTRICAL_SYSTEM_ID)    				Cal_Elec_System_ID_Batch();
    else if(MS_enum == BEMF_ID)                 				Send_Elec_BEMF_Value();
    else if(MS_enum == CURRENT_BANDWIDTH_CHECK) 				Send_Elec_Bandwidth_Data();
    else if(MS_enum == AUTO_TUNING)             				Tune_Gain();
    else if(MS_enum == ADV_FRICTION_ID)         				Send_Advanced_Friction_ID_Data();
    else if(MS_enum == CAL_FRICTION_LUT)        				Cal_Friction_LUT();
  	else if(MS_enum == AGING_TEST_DONE)        					Send_Aging_Test_Result(); // JS -
    else if(MS_enum == INIT_ELEC_ANGLE_ID)      				Send_Elec_Angle_Value(); // JS -
    else if(MS_enum == IMPORT_MES_HWFW)      					ImportMES_HWFW();
//    else if(MS_enum == IMPORT_MES_CODE)      					ImportMES_Code();
    else if(MS_enum == FDCAN_TEST_START)      					FDCAN_Test_Start();
    else if(MS_enum == FDCAN_TEST_RESET)      					FDCAN_Test_Reset();
    else if(MS_enum == SAVE_PROPERTIES_NeutralPosture)      	SaveProperties_NeutralPosture();
    else if(MS_enum == SEND_TORQUE_ACCURACY_RESULT)      		Send_Torque_Accuracy_Result();
    else if(MS_enum == SEND_TORQUE_UNIFORMITY_RESULT)      		Send_Torque_Uniformity_Result();
    else if(MS_enum == SEND_ACTUATOR_BACKDRIVERBILITY_RESULT)   Send_Actuator_backdriverbility_Result();
    else if(MS_enum == SEND_ENCODER_LINEARITY_RESULT)   		Send_Encoder_Linearity_Result();

    else                                        continue;
#endif /* _USE_DEBUG_CLI */

    // MS_enum = IDLE;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void MD_ReadFlags(){
	uint32_t readAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;
	uint32_t t_FW_Update_Flag, t_FW_Backup_Flag=0;
	IOIF_ReadFlash_FF(readAddr, &FW_Update_Flag,       	IOIF_FLASH_READ_SIZE_4B);
	for(int i=0; i<7000;i++){}
	t_FW_Update_Flag=FW_Update_Flag;
	readAddr += IOIF_FLASH_READ_SIZE_32B;

	IOIF_ReadFlash_FF(readAddr, &FW_Backup_Flag,         	IOIF_FLASH_READ_SIZE_4B);
	for(int i=0; i<7000;i++){}
	t_FW_Backup_Flag=FW_Backup_Flag;
	readAddr += IOIF_FLASH_READ_SIZE_32B;

	IOIF_ReadFlash_FF(readAddr, &MD_bin_size,        		IOIF_FLASH_READ_SIZE_4B);
	for(int i=0; i<7000;i++){}

	if(MD_bin_size != MD_fw_binary_size){
		Boot_SetFlags(t_FW_Update_Flag,t_FW_Backup_Flag);
	}
}
uint32_t ReadMDUpdateFlag(){
	return *MD_UPDATE_FLAG;
}

void UploadProperties()
{
	uint8_t memory_isUsed = 0;
	uint8_t buf = 0;
	uint16_t t_identifier = 0;

	IOIF_ReadFlash(IOIF_FLASH_START_USER_ADDR, &memory_isUsed, MEMORY_SECOND_HAND_CHECK);

	if(memory_isUsed == 0xFF){	// First Use
		for(int i = 0; i<25000; ++i) {}

		t_identifier = GUI_SYNC|FIRST_USE;
		Send_MSG(t_identifier, &buf, 0);

		MS_enum = IDLE;
		return;
	} else {
		uint32_t len = memory_isUsed + 1;
		IOIF_ReadFlash(IOIF_FLASH_START_USER_ADDR, &motor_properties.name_length, len);

		for(int i = 0; i < 25000; ++i) {}

		t_identifier = GUI_SYNC|SECOND_USE;
		Send_MSG(t_identifier, &motor_properties.name_length, len);
	}

	MS_enum = IDLE;
	return;
}

void SaveProperties()
{
	writeAddr = 0;
	float memArr1[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr2[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr3[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr4[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr5[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr6[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr7[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr8[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr9[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr10[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr11[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr12[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr13[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr14[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr15[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr16[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr17[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr18[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr19[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr20[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr21[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr22[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr23[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr24[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr25[READ_FLASH_ARRAY_SIZE] = {0};

	float memArr26[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr27[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr28[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr29[READ_FLASH_ARRAY_SIZE] = {0};


#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || \
    defined (L30_MD_REV08_ENABLED)
    float memArr50[READ_FLASH_ARRAY_SIZE] = {0};
#endif

	IOIF_EraseFlash(IOIF_FLASH_START_USER_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_START_USER_ADDR;

	/* Save Name */
	IOIF_WriteFlash(writeAddr, &motor_properties.name_length);
	writeAddr += 32;

	/* Save Motor Setting */
	memcpy(&memArr1[0], &motor_properties.pole_pair,     	   	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr1[1], &inc25KhzObj.resolution,         	   	sizeof(inc25KhzObj.resolution));
	memcpy(&memArr1[2], &motor_properties.gear_ratio,    	   	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr1[3], &motor_properties.Kt,            	   	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr1[4], &motor_properties.Ke,            	   	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr1[5], &motor_setting.peakCurr_limit,			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr1[6], &motor_setting.contCurr_limit,			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr1[7], &motor_setting.max_velocity_rpm,		IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr1);
	writeAddr += 32;

	memcpy(&memArr2[0], &motor_setting.commutation_set.done,   IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[1], &motor_setting.commutation_set.cc_dir, IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[2], &motor_setting.commutation_set.ma_dir, IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[3], &motor_setting.commutation_set.ea_dir, IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[4], &motor_properties.R,                   IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[5], &motor_properties.L,                   IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[6], &kf_current_object.kf_A,               IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr2[7], &kf_current_object.kf_B,               IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr2);
	writeAddr += 32;

	memcpy(&memArr3[0], &kf_current_object.kf_C,				IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[1], &motor_setting.low_level_kalman_on,		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[2], &motor_setting.currCtrl_BW_radPsec,		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[3], &motor_properties.J,					IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[4], &motor_properties.B,					IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[5], &motor_properties.a,					IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[6], &motor_properties.b,					IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr3[7], &motor_properties.c,					IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr3);
	writeAddr += 32;

	memcpy(&memArr4[0], &motor_properties.d,        IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr4[1], &mid_ctrl_saturation,       IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr4[2], &velCtrl.Kp,                IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr4[3], &velCtrl.Ki,                IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr4[4], &velCtrl.Ctrl_BW_Hz,        IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr4[5], &IRC.numerator_length,      IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr4[6], &IRC.denominator_length,    sizeof(IRC.denominator_length));
	memcpy(&memArr4[7], &IRC.saturation,            IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr4);
	writeAddr += 32;

	memcpy(&memArr5[0], IRC.irc_num,                sizeof(IRC.irc_num));	// 4*6
	IOIF_WriteFlash(writeAddr, memArr5);
	writeAddr += 32;

	memcpy(&memArr6[0], IRC.irc_den,                sizeof(IRC.irc_den));	// 4*6
	memcpy(&memArr6[6], &veObj.type,                IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr6[7], &posCtrl.Kp,                IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr6);
	writeAddr += 32;

	memcpy(&memArr7[0], &posCtrl.Kd,                IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[1], &posCtrl.R,                 IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[2], &AbsObj1.offset,            IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[3], &AbsObj1.sign,              IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[4], &VSD.lower_limit,           IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[5], &VSD.upper_limit,           IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[6], &VSD.lower_damped_range,    IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr7[7], &VSD.upper_damped_range,    IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr7);
	writeAddr += 32;

	memcpy(&memArr8[0], &VSD.lower_stiff_range,     IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr8[1], &VSD.upper_stiff_range,     IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr8[2], &VSD.lower_damper,          IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr8[3], &VSD.upper_damper,          IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr8[4], &VSD.lower_stiffness,       IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr8[5], &VSD.upper_stiffness,       IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr8[6], &VSD.saturation,            IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr8);
	writeAddr += 32;

	memcpy(&memArr9[0], &posDOB.wc_Q,               sizeof(posDOB.wc_Q));
	memcpy(&memArr9[1], &posDOB.saturation,         sizeof(posDOB.saturation));
	memcpy(&memArr9[2], &posDOB.gq_num,             sizeof(posDOB.gq_num)); //4*6
	IOIF_WriteFlash(writeAddr, memArr9);
	writeAddr += 32;

	memcpy(&memArr10[0], &posDOB.gq_den,            sizeof(posDOB.gq_den)); //4*6
	memcpy(&memArr10[6], &posFF.num_length,         sizeof(posFF.num_length));
	memcpy(&memArr10[7], &posFF.den_length,         sizeof(posFF.den_length));
	IOIF_WriteFlash(writeAddr, memArr10);
	writeAddr += 32;

	memcpy(&memArr11[0], &posDOB.q_num,             sizeof(posDOB.q_num)); //4*4
	memcpy(&memArr11[4], &posDOB.q_den,             sizeof(posDOB.q_den)); //4*4
	IOIF_WriteFlash(writeAddr, memArr11);
	writeAddr += 32;

	memcpy(&memArr12[0], posFF.num,                 sizeof(posFF.num));		//4*4
	memcpy(&memArr12[4], posFF.den,                 sizeof(posFF.den));		//4*4
	IOIF_WriteFlash(writeAddr, memArr12);
	writeAddr += 32;

	memcpy(&memArr13[0], &veObj.lpf_a, 				IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[1], &veObj.lpf_b, 				IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[2], &veObj.masking_size, 		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[3], &veObj.leadlag_a, 			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[4], &veObj.leadlag_b, 			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[5], &posDOB.gq_num_length,  	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[6], &posDOB.gq_den_length,  	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr13[7], &posDOB.q_num_length, 		IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr13);
	writeAddr += 32;

	memcpy(&memArr14[0], &posDOB.q_den_length, 					IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[1], &advanced_friction_id.scaling_factor, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[2], &advanced_friction_id.lut_mdl, 		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[3], &advanced_friction_id.lut_epsilon, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[4], &advanced_friction_id.lut_p1, 			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[5], &advanced_friction_id.lut_p2, 			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[6], &advanced_friction_id.lut_p3, 			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr14[7], &advanced_friction_id.lut_p4, 			IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr14);
	writeAddr += 32;

	memcpy(&memArr15[0], &advanced_friction_id.lut_p5, 		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[1], &advanced_friction_id.lut_d_mu_v, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[2], &advanced_friction_id.gain, 		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[3], &advanced_friction_id.max_vel1, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[4], &advanced_friction_id.max_vel2, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[5], &advanced_friction_id.vel_num1, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[6], &advanced_friction_id.vel_num2, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr15[7], &advanced_friction_id.P_gain, 		IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr15);
	writeAddr += 32;

	memcpy(&memArr16[0], &advanced_friction_id.I_gain, 				IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr16[1], &advanced_friction_id.gainScalingFactor, 	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr16[2], &advanced_friction_id.time_per_vel, 		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr16[3], &advanced_friction_id.cut_time, 			IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr16);
	writeAddr += 32;

	/**/
	for(int i = 0; i < (uint16_t)(FRICTION_LUT_SIZE/8); ++i) { // FRICTION_LUT_SIZE/8 = 1024
		memcpy(memArr17, &advanced_friction_id.adv_friction_compensator_LUT[i*8], IOIF_FLASH_WRITE_SIZE_4B*8);
		IOIF_WriteFlash(writeAddr, memArr17);
		writeAddr += 32;
	}

	for(int i = 0; i < (((uint16_t)FRICTION_LUT_SIZE)%8); ++i){	// FRICTION_LUT_SIZE%8 = 1
		memcpy(&memArr18[i], &advanced_friction_id.adv_friction_compensator_LUT[((uint16_t)FRICTION_LUT_SIZE/8)*8 + i], IOIF_FLASH_WRITE_SIZE_4B);
	}
	IOIF_WriteFlash(writeAddr, memArr18);
	writeAddr += 32;

	memcpy(&memArr19[0], &motor_setting.commutation_set.hall_sensor_table[0],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[1], &motor_setting.commutation_set.hall_sensor_table[1],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[2], &motor_setting.commutation_set.hall_sensor_table[2],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[3], &motor_setting.commutation_set.hall_sensor_table[3],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[4], &motor_setting.commutation_set.hall_sensor_table[4],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[5], &motor_setting.commutation_set.hall_sensor_table[5],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[6], &motor_setting.commutation_set.hall_sensor_dir,	    IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr19[7], &motor_setting.commutation_set.abs_encoder_offset,	  	IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr19);
	writeAddr += 32;

	memcpy(&memArr20[0], &motor_setting.sensor_setting.e_angle_homing_sensor,  	 IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[1], &motor_setting.sensor_setting.m_angle_homing_sensor,    IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[2], &motor_setting.sensor_setting.commutation_sensor,	     IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[3], &motor_setting.sensor_setting.pos_feedback_sensor,	     IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[4], &motor_setting.sensor_setting.temperature_sensor_usage, IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[5], &motor_setting.sensor_setting.imu_6axis_usage,	         IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[6], &motor_setting.sensor_setting.imu_3axis_usage,	         IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr20[7], &AbsObj1.location,	                                     IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr20);
	writeAddr += 32;

	memcpy(&memArr21[0], &AbsObj2.location,										IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[1], &AbsObj2.offset,										IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[2], &AbsObj2.sign,											IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[3], &impedanceCtrl.Kp_max,									IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[4], &impedanceCtrl.Kd_max,									IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[5], &impedanceCtrl.option,									IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[6], &impedanceCtrl.opt1_i_buffer.epsilon_target,			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr21[7], &impedanceCtrl.opt1_i_buffer.Kp_target,               	IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr21);
	writeAddr += 32;

	memcpy(&memArr22[0], &impedanceCtrl.opt1_i_buffer.Kd_target,                IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr22[1], &impedanceCtrl.opt1_i_buffer.lambda_target,            IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr22[2], &impedanceCtrl.opt1_i_buffer.duration,                 IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr22);
	writeAddr += 32;

	memcpy(&memArr26[0], &motor_setting.commutation_set.raw_elecAngle[0],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[1], &motor_setting.commutation_set.raw_elecAngle[1],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[2], &motor_setting.commutation_set.raw_elecAngle[2],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[3], &motor_setting.commutation_set.raw_elecAngle[3],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[4], &motor_setting.commutation_set.raw_elecAngle[4],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[5], &motor_setting.commutation_set.raw_elecAngle[5],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[6], &motor_setting.commutation_set.raw_elecAngle[6],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr26[7], &motor_setting.commutation_set.raw_elecAngle[7],	IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr26);
	writeAddr += 32;

	memcpy(&memArr27[0], &motor_setting.commutation_set.raw_elecAngle[8],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[1], &motor_setting.commutation_set.raw_elecAngle[9],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[2], &motor_setting.commutation_set.raw_elecAngle[10],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[3], &motor_setting.commutation_set.raw_elecAngle[11],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[4], &motor_setting.commutation_set.raw_elecAngle[12],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[5], &motor_setting.commutation_set.raw_elecAngle[13],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[6], &motor_setting.commutation_set.std_dev,			IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr27[7], &motor_setting.commutation_set.compensation_enable,IOIF_FLASH_WRITE_SIZE_4B);

	IOIF_WriteFlash(writeAddr, memArr27);
	writeAddr += 32;

	memcpy(&memArr28[0], &motor_setting.commutation_set.raw_mechAngle[0],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[1], &motor_setting.commutation_set.raw_mechAngle[1],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[2], &motor_setting.commutation_set.raw_mechAngle[2],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[3], &motor_setting.commutation_set.raw_mechAngle[3],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[4], &motor_setting.commutation_set.raw_mechAngle[4],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[5], &motor_setting.commutation_set.raw_mechAngle[5],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[6], &motor_setting.commutation_set.raw_mechAngle[6],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr28[7], &motor_setting.commutation_set.raw_mechAngle[7],	IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr28);
	writeAddr += 32;

	memcpy(&memArr29[0], &motor_setting.commutation_set.raw_mechAngle[8],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr29[1], &motor_setting.commutation_set.raw_mechAngle[9],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr29[2], &motor_setting.commutation_set.raw_mechAngle[10],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr29[3], &motor_setting.commutation_set.raw_mechAngle[11],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr29[4], &motor_setting.commutation_set.raw_mechAngle[12],	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr29[5], &motor_setting.commutation_set.raw_mechAngle[13],	IOIF_FLASH_WRITE_SIZE_4B);

	IOIF_WriteFlash(writeAddr, memArr29);
	writeAddr += 32;

	IOIF_EraseFlash(IOIF_FLASH_SECTOR_5_BANK2_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_SECTOR_5_BANK2_ADDR;

    flashsensorObj.initOffsetIMUDeg = filteredAngleDataObj_Sagittal.initOffsetIMUDeg;
    flashsensorObj.scaling_gyro = imuProcessedDataObj.scaling_gyro;
    flashsensorObj.bias_gyro = imuProcessedDataObj.bias_gyro;
    flashsensorObj.accMeanVal_calibrated = imuProcessedDataObj.accMeanVal_calibrated;
    flashsensorObj.autoCalibratedFlag = autoCalibratedFlag;
    flashsensorObj.AbsEnc1_raw_offset = AbsObj1.raw_offset;

	memcpy(&memArr23[0], &flashsensorObj.initOffsetIMUDeg,                    	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr23[1], &flashsensorObj.degAbsOffset,                        	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr23[2], &flashsensorObj.scaling_gyro,                       	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr23[3], &flashsensorObj.bias_gyro,                          	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr23[4], &flashsensorObj.accMeanVal_calibrated,              	IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr23[5], &flashsensorObj.autoCalibratedFlag,                    IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr23[6], &flashsensorObj.AbsEnc1_raw_offset,                    IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr23);
	writeAddr += 32;

	IOIF_EraseFlash(IOIF_FLASH_SECTOR_6_BANK2_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_SECTOR_6_BANK2_ADDR;

	memcpy(&memArr24[0], &MES.code,				    						30);

    IOIF_WriteFlash(writeAddr, memArr24);
	writeAddr += 32;

	memcpy(&memArr25[0], &MES.code_length,				    				sizeof(MES.code_length));
    memcpy(&memArr25[1], &MES.HWVer,										IOIF_FLASH_WRITE_SIZE_4B);
    memcpy(&memArr25[2], &MES.FWVer.major,			    					IOIF_FLASH_WRITE_SIZE_4B);
    memcpy(&memArr25[3], &MES.FWVer.minor,			    					IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr25[4], &MES.FWVer.patch,				    				IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr25[5], &MES.FWVer.debug,				    				IOIF_FLASH_WRITE_SIZE_4B);

    IOIF_WriteFlash(writeAddr, memArr25);
	writeAddr += 32;


#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || \
    defined (L30_MD_REV08_ENABLED)
	memcpy(&memArr50[0], &uprightObj.slope,         IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr50[1], &uprightObj.offset,        IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr50[2], &uprightObj.DirMove,       IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr50[3], &uprightObj.DirButton,     IOIF_FLASH_WRITE_SIZE_4B);
    IOIF_WriteFlash(writeAddr, memArr50);
	writeAddr += 32;
#endif
	/********************************************************************************************************************/
	Send_MSG((uint16_t)(GUI_SYNC|SAVE_DONE), (uint8_t*)0, 1);

	MS_enum = IDLE;
	return;
}

void SaveProperties_NeutralPosture()
{
	writeAddr = 0;
	float memArr[READ_FLASH_ARRAY_SIZE] = {0};

	IOIF_EraseFlash(IOIF_FLASH_SECTOR_7_BANK2_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_SECTOR_7_BANK2_ADDR;

	memcpy(&memArr[0], &neutralizedFlag,                              		IOIF_FLASH_WRITE_SIZE_4B);
	memcpy(&memArr[1], &filteredAngleDataObj_Sagittal.NeutralPostureBias,   IOIF_FLASH_WRITE_SIZE_4B);
	IOIF_WriteFlash(writeAddr, memArr);
	writeAddr += 32;

	/********************************************************************************************************************/
	Send_MSG((uint16_t)(GUI_SYNC|SAVE_DONE), (uint8_t*)0, 1);

	MS_enum = IDLE;
	return;
}


void DownloadProperties()
{
    readAddr = IOIF_FLASH_START_USER_ADDR;
    // IOIF_ReadFlash(readAddr, nameTestRes, IOIF_FLASH_READ_SIZE_32B);
    readAddr += 32;

    /* Download Motor Setting */
    IOIF_ReadFlash(readAddr, &motor_properties.pole_pair,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &inc25KhzObj.resolution,           IOIF_FLASH_READ_SIZE_4B);
    IOIF_ReadFlash(readAddr, &inc1KhzObj.resolution, 	        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.gear_ratio,	    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.Kt,			    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.Ke,              IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.peakCurr_limit, 	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.contCurr_limit, 	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.max_velocity_rpm,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    /* rpm 2 rad/s */
    motor_setting.max_velocity_radPsec = motor_setting.max_velocity_rpm * M_PI / 30;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.done,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.cc_dir,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.ma_dir,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.ea_dir,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.R,                    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.L,                    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &kf_current_object.kf_A,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &kf_current_object.kf_B,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &kf_current_object.kf_C,             IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.low_level_kalman_on,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.currCtrl_BW_radPsec,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.J,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.B,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.a,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.b,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_properties.c,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B; //garbage

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &motor_properties.d,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &mid_ctrl_saturation,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &velCtrl.Kp,               IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &velCtrl.Ki,               IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &velCtrl.Ctrl_BW_Hz,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &IRC.numerator_length,     IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &IRC.denominator_length,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &IRC.saturation,           IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, IRC.irc_num, sizeof(IRC.irc_num)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_32B; // float 4 * 8

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, IRC.irc_den, sizeof(IRC.irc_den));     readAddr += IOIF_FLASH_READ_ADDR_SIZE_24B; // float 4 * 6
    IOIF_ReadFlash(readAddr, &veObj.type, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posCtrl.Kp, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &posCtrl.Kd,             IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posCtrl.R,              IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &AbsObj1.offset,         IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &AbsObj1.sign,           IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.lower_limit,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.upper_limit,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.lower_damped_range, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.upper_damped_range, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &VSD.lower_stiff_range,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.lower_stiff_range,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.lower_damper,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.upper_damper,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.lower_stiffness,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.upper_stiffness,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &VSD.saturation,         IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_8B; // float 4 * 2

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &posDOB.wc_Q,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posDOB.saturation,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, posDOB.gq_num,       sizeof(posDOB.gq_num));   readAddr += IOIF_FLASH_READ_ADDR_SIZE_24B; // float 4 * 6

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, posDOB.gq_den,     sizeof(posDOB.gq_den));   readAddr += IOIF_FLASH_READ_ADDR_SIZE_24B; // float 4 * 6
    IOIF_ReadFlash(readAddr, &posFF.num_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posFF.den_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, posDOB.q_num, sizeof(posDOB.q_num)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_16B; // float 4 * 4
    IOIF_ReadFlash(readAddr, posDOB.q_den, sizeof(posDOB.q_den)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_16B; // float 4 * 4

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, posFF.num, sizeof(posFF.num)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_16B; // float 4 * 4
    IOIF_ReadFlash(readAddr, posFF.den, sizeof(posFF.den)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_16B; // float 4 * 4

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &veObj.lpf_a,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &veObj.lpf_b,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &veObj.masking_size,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &veObj.leadlag_a,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &veObj.leadlag_b,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posDOB.gq_num_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posDOB.gq_den_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &posDOB.q_num_length,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &posDOB.q_den_length,                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.scaling_factor,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_mdl,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_epsilon,	    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_p1,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_p2,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_p3,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_p4,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_p5,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.lut_d_mu_v,	    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.gain,			IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.max_vel1,		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.max_vel2,		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.vel_num1,		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.vel_num2,		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.P_gain,		    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i) {}

    IOIF_ReadFlash(readAddr, &advanced_friction_id.I_gain,			    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.gainScalingFactor,	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.time_per_vel,		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.cut_time,			IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_20B;

    /**/
    for(int i = 0; i<500; ++i) {}

    for(int i = 0; i < (uint16_t)(FRICTION_LUT_SIZE/8); ++i) { // FRICTION_LUT_SIZE/8 = 1024
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+1],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+2],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+3],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+4],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+5],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+6],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+7],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
        
        for(int j = 0; j<500; ++j) {}
    }

    for(int i = 0; i < (((uint16_t)FRICTION_LUT_SIZE) % 8); ++i) { // FRICTION_LUT_SIZE%8 = 1
        IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[((uint16_t)FRICTION_LUT_SIZE/8)*8 + i], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    }
    readAddr += (32 - (((uint16_t)FRICTION_LUT_SIZE) % 8) * 4);

    for(int i = 0; i< 500; ++i) {}

    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[0],   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[1],   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[2],   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[3],   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[4],   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[5],   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_dir,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.abs_encoder_offset,     IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i< 500; ++i) {}

    // IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.hall_sensor_usage,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.e_angle_homing_sensor,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.m_angle_homing_sensor,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.commutation_sensor,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.pos_feedback_sensor,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.temperature_sensor_usage, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.imu_6axis_usage,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.imu_3axis_usage,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &AbsObj1.location,                                      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i< 500; ++i) {}

    IOIF_ReadFlash(readAddr, &AbsObj2.location,                                      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &AbsObj2.offset,                                        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &AbsObj2.sign,                                          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.Kp_max,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.Kd_max,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.option,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.epsilon_target,            IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.Kp_target,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i< 500; ++i) {}

    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.Kd_target,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.lambda_target,            IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.duration,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_24B;

	for(int i = 0; i< 500; ++i) {}

	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[0],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[1],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[2],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[3],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[4],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[5],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[6],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[7],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	for(int i = 0; i< 500; ++i) {}

	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[8],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[9],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[10],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[11],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[12],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_elecAngle[13],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.std_dev,				IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.compensation_enable,	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	for(int i = 0; i< 500; ++i) {}

	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[0],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[1],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[2],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[3],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[4],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[5],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[6],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[7],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	for(int i = 0; i< 500; ++i) {}

	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[8],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[9],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[10],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[11],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[12],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.raw_mechAngle[13],		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	for(int i = 0; i< 500; ++i) {}

    readAddr = IOIF_FLASH_SECTOR_5_BANK2_ADDR;

    IOIF_ReadFlash(readAddr, &flashsensorObj.initOffsetIMUDeg,                    	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &flashsensorObj.degAbsOffset,                        	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &flashsensorObj.scaling_gyro,                       	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &flashsensorObj.bias_gyro,                       		IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &flashsensorObj.accMeanVal_calibrated,              	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &flashsensorObj.autoCalibratedFlag,					IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &flashsensorObj.AbsEnc1_raw_offset,							IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    filteredAngleDataObj_Sagittal.initOffsetIMUDeg = flashsensorObj.initOffsetIMUDeg;
    imuProcessedDataObj.scaling_gyro = flashsensorObj.scaling_gyro;
    imuProcessedDataObj.bias_gyro = flashsensorObj.bias_gyro;
    imuProcessedDataObj.accMeanVal_calibrated = flashsensorObj.accMeanVal_calibrated;
    autoCalibratedFlag = flashsensorObj.autoCalibratedFlag;
    AbsObj1.raw_offset = flashsensorObj.AbsEnc1_raw_offset;

    for(int i = 0; i< 500; ++i) {}

    readAddr = IOIF_FLASH_SECTOR_6_BANK2_ADDR;

    IOIF_ReadFlash(readAddr, &MES.code,										    	32); 			readAddr += 32;

    for(int i = 0; i< 500; ++i) {}

    IOIF_ReadFlash(readAddr, &MES.code_length,										IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &MES.HWVer,										    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &MES.FWVer.major,										IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &MES.FWVer.minor,										IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &MES.FWVer.patch,										IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &MES.FWVer.debug,										IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i< 500; ++i) {}

	readAddr = IOIF_FLASH_SECTOR_7_BANK2_ADDR;

	IOIF_ReadFlash(readAddr, &neutralizedFlag,                    					IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &filteredAngleDataObj_Sagittal.NeutralPostureBias,     IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;


	#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || \
    defined (L30_MD_REV08_ENABLED)
    for(int i = 0; i< 500; ++i) {}

    IOIF_ReadFlash(readAddr, &uprightObj.slope,						               IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &uprightObj.offset,              			   	 	   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &uprightObj.DirMove,                 				   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &uprightObj.DirButton,                				   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
#endif

    /********************** Send Data for Synchronization With UI ***************************/
    uint16_t t_identifier = 0;
    float temp_arr[40] = {0};
    uint8_t temp_arr_index = 0;
    uint8_t temp_arr_uint8[40]={0}; //for MES

    /****************************************************************************************************************************/
    /* MEMORY_MES_HWFW */
    // UI_MES_AGING
    uint8_t t_length= (uint8_t)MES.code_length;

//    memcpy(&temp_arr[0], &AgingCtrl.aging_time,     sizeof(AgingCtrl.aging_time));      temp_arr_index++;
    memcpy(&temp_arr_uint8[0], &MES.HWVer,       		sizeof(MES.HWVer));           		temp_arr_index++;
    memcpy(&temp_arr_uint8[1], &MES.FWVer.major,       	sizeof(MES.FWVer.major));           temp_arr_index++;
    memcpy(&temp_arr_uint8[2], &MES.FWVer.minor,       	sizeof(MES.FWVer.minor));           temp_arr_index++;
    memcpy(&temp_arr_uint8[3], &MES.FWVer.patch,       	sizeof(MES.FWVer.patch));           temp_arr_index++;
    memcpy(&temp_arr_uint8[4], &MES.FWVer.debug,       	sizeof(uint16_t));					temp_arr_index+=2;//sizeof(MES.FWVer.debug));           temp_arr_index++;
    memcpy(&temp_arr_uint8[6], &MES.code_length, 		sizeof(MES.code_length));      		temp_arr_index++;
    memcpy(&temp_arr_uint8[7], &MES.code,       		t_length); 							temp_arr_index+=t_length;//sizeof(MES.code));          		temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_MES_HWFW;
    Send_MSG(t_identifier, (uint8_t*)temp_arr_uint8, temp_arr_index);//40);//(uint8_t*)temp_arr_uint8, 40);//temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr_uint8, 0, sizeof(temp_arr_uint8));
    memset(&temp_arr, 0, sizeof(temp_arr));

    /* MEMORY_UI_MOTOR_PROPERTIES */
    // UI_MOTOR_PROPERTIES
    memcpy(&temp_arr[0], &motor_properties.pole_pair,       sizeof(motor_properties.pole_pair));           temp_arr_index++;
    memcpy(&temp_arr[1], &inc25KhzObj.resolution,           sizeof(inc25KhzObj.resolution));               temp_arr_index++;
    memcpy(&temp_arr[2], &motor_properties.gear_ratio,      sizeof(motor_properties.gear_ratio));          temp_arr_index++;
    memcpy(&temp_arr[3], &motor_properties.Kt,              sizeof(motor_properties.Kt));                  temp_arr_index++;
    memcpy(&temp_arr[4], &motor_properties.Ke,              sizeof(motor_properties.Ke));                  temp_arr_index++;
    memcpy(&temp_arr[5], &motor_setting.peakCurr_limit,     sizeof(motor_setting.peakCurr_limit));         temp_arr_index++;
    memcpy(&temp_arr[6], &motor_setting.contCurr_limit,     sizeof(motor_setting.contCurr_limit));         temp_arr_index++;
    memcpy(&temp_arr[7], &motor_setting.max_velocity_rpm,   sizeof(motor_setting.max_velocity_rpm));       temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_MOTOR_PROPERTIES;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /****************************************************************************************************************************/
    /* MEMORY_UI_SENSOR_SETTING */
    // UI_SENSOR_SETTING
    memcpy(&temp_arr[0],  &motor_setting.sensor_setting.e_angle_homing_sensor,          sizeof(motor_setting.sensor_setting.e_angle_homing_sensor));    temp_arr_index++;
    memcpy(&temp_arr[1],  &motor_setting.sensor_setting.m_angle_homing_sensor,          sizeof(motor_setting.sensor_setting.m_angle_homing_sensor));    temp_arr_index++;
    memcpy(&temp_arr[2],  &motor_setting.sensor_setting.commutation_sensor,             sizeof(motor_setting.sensor_setting.commutation_sensor));       temp_arr_index++;
    memcpy(&temp_arr[3],  &motor_setting.sensor_setting.pos_feedback_sensor,            sizeof(motor_setting.sensor_setting.pos_feedback_sensor));      temp_arr_index++;
    memcpy(&temp_arr[4],  &motor_setting.sensor_setting.temperature_sensor_usage,       sizeof(motor_setting.sensor_setting.temperature_sensor_usage)); temp_arr_index++;
    memcpy(&temp_arr[5],  &motor_setting.sensor_setting.imu_6axis_usage,                sizeof(motor_setting.sensor_setting.imu_6axis_usage));          temp_arr_index++;
    memcpy(&temp_arr[6],  &motor_setting.sensor_setting.imu_3axis_usage,                sizeof(motor_setting.sensor_setting.imu_3axis_usage));          temp_arr_index++;
    memcpy(&temp_arr[7],  &motor_setting.sensor_setting.hall_sensor_usage,              sizeof(motor_setting.sensor_setting.hall_sensor_usage));        temp_arr_index++;
    memcpy(&temp_arr[8],  &AbsObj1.location,                                            sizeof(AbsObj1.location));                                      temp_arr_index++;
    memcpy(&temp_arr[9],  &AbsObj2.location,                                            sizeof(AbsObj2.location));                                      temp_arr_index++;
    memcpy(&temp_arr[10], &motor_setting.commutation_set.abs_encoder_offset, 			sizeof(motor_setting.commutation_set.abs_encoder_offset)); 		temp_arr_index++;
//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_SENSOR_SETTING;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /****************************************************************************************************************************/
    /* MEMORY_UI_ELECTRICAL_PROPERTIES */
    // ELECTRICAL PROPERTIES
    memcpy(&temp_arr[0], &motor_properties.R,                 sizeof(motor_properties.R));                temp_arr_index++;
    memcpy(&temp_arr[1], &motor_properties.L,                 sizeof(motor_properties.L));                temp_arr_index++;
    memcpy(&temp_arr[2], &kf_current_object.kf_C,             sizeof(kf_current_object.kf_C));            temp_arr_index++;
    memcpy(&temp_arr[3], &motor_setting.currCtrl_BW_radPsec,  sizeof(motor_setting.currCtrl_BW_radPsec)); temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_ELECTRICAL_PROPERTIES;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /* MEMORY_UI_FRICTION_ID_PARAMETERS */
    // FRICTION ID PARAMETERS

    float t_P_pctg = 0.0, t_I_pctg = 0.0, t_tps = 0.0, t_ct = 0.0;
    t_P_pctg = (advanced_friction_id.P_gain / motor_setting.contCurr_limit)*advanced_friction_id.gainScalingFactor;
    t_I_pctg = (advanced_friction_id.I_gain / motor_setting.contCurr_limit)*advanced_friction_id.gainScalingFactor;
    t_tps = (float)advanced_friction_id.time_per_vel / LOW_LEVEL_CONTROL_FREQUENCY;
    t_ct = (float)advanced_friction_id.cut_time / LOW_LEVEL_CONTROL_FREQUENCY;

    memcpy(&temp_arr[0], &advanced_friction_id.lut_mdl,		sizeof(advanced_friction_id.lut_mdl));      temp_arr_index++;
    memcpy(&temp_arr[1], &advanced_friction_id.lut_epsilon,	sizeof(advanced_friction_id.lut_epsilon));  temp_arr_index++;
    memcpy(&temp_arr[2], &advanced_friction_id.lut_p1,      sizeof(advanced_friction_id.lut_p1));       temp_arr_index++;
    memcpy(&temp_arr[3], &advanced_friction_id.lut_p2,      sizeof(advanced_friction_id.lut_p2));       temp_arr_index++;
    memcpy(&temp_arr[4], &advanced_friction_id.lut_d_mu_v,  sizeof(advanced_friction_id.lut_d_mu_v));   temp_arr_index++;
    memcpy(&temp_arr[5], &advanced_friction_id.gain,        sizeof(advanced_friction_id.gain));         temp_arr_index++;
    memcpy(&temp_arr[6], &advanced_friction_id.max_vel1,    sizeof(advanced_friction_id.max_vel1));     temp_arr_index++;
    memcpy(&temp_arr[7], &advanced_friction_id.max_vel2,    sizeof(advanced_friction_id.max_vel2));     temp_arr_index++;
    memcpy(&temp_arr[8], &advanced_friction_id.vel_num1,    sizeof(advanced_friction_id.vel_num1));     temp_arr_index++;
    memcpy(&temp_arr[9], &advanced_friction_id.vel_num2,    sizeof(advanced_friction_id.vel_num2));     temp_arr_index++;
    memcpy(&temp_arr[10], &t_P_pctg,	                    sizeof(t_P_pctg));	                        temp_arr_index++;
    memcpy(&temp_arr[11], &t_I_pctg,	                    sizeof(t_I_pctg));	                        temp_arr_index++;
    memcpy(&temp_arr[12], &advanced_friction_id.gainScalingFactor,	sizeof(advanced_friction_id.gainScalingFactor));  temp_arr_index++;
    memcpy(&temp_arr[13], &t_tps,	                        sizeof(t_tps));	                            temp_arr_index++;
    memcpy(&temp_arr[14], &t_ct,	                        sizeof(t_ct));	                            temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_FRICTION_ID_PARAMETERS;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /****************************************************************************************************************************/
    /* MEMORY_UI_MECHANICAL_PROPERTIES */
    memcpy(&temp_arr[0], &motor_properties.J,       sizeof(motor_properties.J));  temp_arr_index++;
    memcpy(&temp_arr[1], &motor_properties.B,       sizeof(motor_properties.B));  temp_arr_index++;
    memcpy(&temp_arr[2], &motor_properties.a,       sizeof(motor_properties.a));  temp_arr_index++;
    memcpy(&temp_arr[3], &motor_properties.b,       sizeof(motor_properties.b));  temp_arr_index++;
    memcpy(&temp_arr[4], &mid_ctrl_saturation,      sizeof(mid_ctrl_saturation)); temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_MECHANICAL_PROPERTIES;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /****************************************************************************************************************************/
    /* MEMORY_UI_CONTROL_PARAMETERS1 */
    memcpy(&temp_arr[0],  &velCtrl.Kp,            sizeof(velCtrl.Kp));            temp_arr_index++;
    memcpy(&temp_arr[1],  &velCtrl.Ki,            sizeof(velCtrl.Ki));            temp_arr_index++;
    memcpy(&temp_arr[2],  &velCtrl.Ctrl_BW_Hz,    sizeof(velCtrl.Ctrl_BW_Hz));    temp_arr_index++;
    memcpy(&temp_arr[3],  &posCtrl.Kp,            sizeof(posCtrl.Kp));            temp_arr_index++;
    memcpy(&temp_arr[4],  &posCtrl.Kd,            sizeof(posCtrl.Kd));            temp_arr_index++;
    memcpy(&temp_arr[5],  &posCtrl.R,             sizeof(posCtrl.R));             temp_arr_index++;
    memcpy(&temp_arr[6],  &posDOB.wc_Q,           sizeof(posDOB.wc_Q));           temp_arr_index++;
    memcpy(&temp_arr[7],  &posDOB.saturation,     sizeof(posDOB.saturation));     temp_arr_index++;
    memcpy(&temp_arr[8],  &veObj.type,            sizeof(veObj.type));            temp_arr_index++;
    memcpy(&temp_arr[9],  &veObj.masking_size,    sizeof(veObj.masking_size));    temp_arr_index++;
    memcpy(&temp_arr[10], &IRC.saturation,        sizeof(IRC.saturation));        temp_arr_index++;
    memcpy(&temp_arr[11], &impedanceCtrl.Kp_max,  sizeof(impedanceCtrl.Kp_max));  temp_arr_index++;
    memcpy(&temp_arr[12], &impedanceCtrl.Kd_max,  sizeof(impedanceCtrl.Kd_max));  temp_arr_index++;
    memcpy(&temp_arr[13], &impedanceCtrl.option,  sizeof(impedanceCtrl.option));  temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_CONTROL_PARAMETERS1;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /****************************************************************************************************************************/
    /* MEMORY_UI_CONTROL_PARAMETERS2 */
    memcpy(&temp_arr[0],  &impedanceCtrl.opt1_i_buffer.epsilon_target,       sizeof(impedanceCtrl.opt1_i_buffer.epsilon_target));    temp_arr_index++;
    memcpy(&temp_arr[1],  &impedanceCtrl.opt1_i_buffer.Kp_target,            sizeof(impedanceCtrl.opt1_i_buffer.Kp_target));         temp_arr_index++;
    memcpy(&temp_arr[2],  &impedanceCtrl.opt1_i_buffer.Kd_target,            sizeof(impedanceCtrl.opt1_i_buffer.Kd_target));         temp_arr_index++;
    memcpy(&temp_arr[3],  &impedanceCtrl.opt1_i_buffer.lambda_target,        sizeof(impedanceCtrl.opt1_i_buffer.lambda_target));     temp_arr_index++;
    memcpy(&temp_arr[4],  &impedanceCtrl.opt1_i_buffer.duration,             sizeof(impedanceCtrl.opt1_i_buffer.duration));          temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_CONTROL_PARAMETERS2;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    /****************************************************************************************************************************/
    /* MEMORY_UI_ADDITIONAL_FUNCTION_PARAMETERS */
    memcpy(&temp_arr[0],  &VSD.lower_limit,         sizeof(VSD.lower_limit));         temp_arr_index++;
    memcpy(&temp_arr[1],  &VSD.upper_limit,         sizeof(VSD.upper_limit));         temp_arr_index++;
    memcpy(&temp_arr[2],  &VSD.lower_damped_range,  sizeof(VSD.lower_damped_range));  temp_arr_index++;
    memcpy(&temp_arr[3],  &VSD.upper_damped_range,  sizeof(VSD.upper_damped_range));  temp_arr_index++;
    memcpy(&temp_arr[4],  &VSD.lower_stiff_range,   sizeof(VSD.lower_stiff_range));   temp_arr_index++;
    memcpy(&temp_arr[5],  &VSD.upper_stiff_range,   sizeof(VSD.upper_stiff_range));   temp_arr_index++;
    memcpy(&temp_arr[6],  &VSD.lower_damper,        sizeof(VSD.lower_damper));        temp_arr_index++;
    memcpy(&temp_arr[7],  &VSD.upper_damper,        sizeof(VSD.upper_damper));        temp_arr_index++;
    memcpy(&temp_arr[8],  &VSD.lower_stiffness,     sizeof(VSD.lower_stiffness));     temp_arr_index++;
    memcpy(&temp_arr[9],  &VSD.upper_stiffness,     sizeof(VSD.upper_stiffness));     temp_arr_index++;
    memcpy(&temp_arr[10], &VSD.saturation,          sizeof(VSD.saturation));          temp_arr_index++;

//    for(int i = 0; i<50000; ++i){}

    IOIF_msDelay(15);

    t_identifier = GUI_SYNC | MEMORY_UI_ADDITIONAL_FUNCTION_PARAMETERS;
    Send_MSG(t_identifier, (uint8_t*)temp_arr, 4 * temp_arr_index);
    temp_arr_index = 0;
    memset(&temp_arr, 0, sizeof(temp_arr));

    MS_enum = IDLE;
}

#ifdef L30_MD_REV06_ENABLED
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 20;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  //PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 20;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  //PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 20;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  //PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Configure the second protected region (New configuration) **/
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;  // Assign a new region number
  MPU_InitStruct.BaseAddress = 0x38000000;     // New region address
  MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;  // New region size
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RW; // Privileged read/write only
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
#endif /* SUIT_MD_ENABLED */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

uint8_t error_flag =0;
void Error_Handler(void)
{
	error_flag=1;
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}

	static uint16_t t_identifier = 0;
	t_passfail = -1;//0;//2; //FAIL

	t_identifier = GUI_SYNC|FDCAN_RESULT;
	Send_MSG(t_identifier, (uint8_t*)(&t_passfail), sizeof(t_passfail));

	/* USER CODE END Error_Handler_Debug */
}

/**
  * @brief Compares two buffers.
  * @par Input
  *  - pBuffer1, pBuffer2: buffers to be compared.
  *  - BufferLength: buffer's length
  * @par Output
  * None.
  * @retval
  *   0: pBuffer1 identical to pBuffer2
  *   1: pBuffer1 differs from pBuffer2
  */
static uint32_t BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
	while(BufferLength--)
	{
		if(*pBuffer1 != *pBuffer2)
		{
			return 1;
		}

		pBuffer1++;
		pBuffer2++;
	}
	return 0;
}

int i=0;
static void FDCAN_Test_Start(){
	/* Bit time configuration adjusted to be closer to MX_FDCAN1_Init:
	 ************************
	          Bit time parameter         | Nominal      |  Data
	          ---------------------------|--------------|----------------
	          fdcan_ker_ck               | 20 MHz       | 20 MHz
	          Time_quantum (tq)          | 50 ns        | 50 ns
	          Synchronization_segment    | 1 tq         | 1 tq
	          Propagation_segment        | 23 tq        | 7 tq
	          Phase_segment_1            | 4 tq         | 11 tq
	          Phase_segment_2            | 2 tq         | 4 tq
	          Synchronization_Jump_width | 2 tq         | 4 tq
	          Bit_length                 | 30 tq = 1.5 μs | 12 tq = 0.6 μs
	          Bit_rate                   | 0.666 MBit/s | 1.666 MBit/s
	 */

	// Initialize FDCAN peripheral in external loopback mode
	hfdcan.Instance = FDCAN1;
	hfdcan.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hfdcan.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
	hfdcan.Init.AutoRetransmission = ENABLE;
	hfdcan.Init.TransmitPause = ENABLE;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.ProtocolException = DISABLE;  // Set to match MX_FDCAN1_Init

	hfdcan.Init.NominalPrescaler = 10;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.NominalSyncJumpWidth = 2;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.NominalTimeSeg1 = 5;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.NominalTimeSeg2 = 2;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.DataPrescaler = 1;  // No change needed
	hfdcan.Init.DataSyncJumpWidth = 4;  // No change needed
	hfdcan.Init.DataTimeSeg1 = 11;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.DataTimeSeg2 = 4;  // No change needed
	hfdcan.Init.MessageRAMOffset = 0;
	hfdcan.Init.StdFiltersNbr = 1;
	hfdcan.Init.ExtFiltersNbr = 1;
	hfdcan.Init.RxFifo0ElmtsNbr = 4;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.RxFifo1ElmtsNbr = 4;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.RxBuffersNbr = 4;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.RxBufferSize = FDCAN_DATA_BYTES_64;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.TxEventsNbr = 4;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.TxBuffersNbr = 4;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.TxFifoQueueElmtsNbr = 16;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;  // Set to match MX_FDCAN1_Init
	hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_64;  // Set to match MX_FDCAN1_Init
	if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
	{
		Error_Handler();
	}

	// Configure extended ID reception filter to Rx FIFO 1
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;
	sFilterConfig.RxBufferIndex = 0;
	if(HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// Start the FDCAN module
	if(HAL_FDCAN_Start(&hfdcan)!= HAL_OK)
	{
		Error_Handler();
	}

	t_passfail=0; // INIT

	// Repeat Tx and Rx actions ten times
	for (i = 0; i < 10; i++) {
		// Add first message to Tx FIFO
		TxHeader.Identifier = 0x112;
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		TxHeader.DataLength = FDCAN_DLC_BYTES_64;
		TxHeader.ErrorStateIndicator = BSP_FDCAN_ESI_ACTIVE;
		TxHeader.BitRateSwitch = BSP_FDCAN_BRS_ON;
		TxHeader.FDFormat = FDCAN_FD_CAN;
		TxHeader.TxEventFifoControl = BSP_FDCAN_NO_TX_EVENTS;
		TxHeader.MessageMarker = 0xAA;
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, TxData0)!= HAL_OK)
		{
			Error_Handler();
		}

		// Add second message to Tx FIFO
		TxHeader.Identifier = 0x115;
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		TxHeader.DataLength = FDCAN_DLC_BYTES_64;
		TxHeader.ErrorStateIndicator = BSP_FDCAN_ESI_ACTIVE;
		TxHeader.BitRateSwitch = BSP_FDCAN_BRS_ON;
		TxHeader.FDFormat = FDCAN_FD_CAN;
		TxHeader.TxEventFifoControl = BSP_FDCAN_NO_TX_EVENTS;
		TxHeader.MessageMarker = 0xBB;
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, TxData1)!= HAL_OK)
		{
			Error_Handler();
		}

		// Add third message to Tx FIFO
		TxHeader.Identifier = 0x118;
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		TxHeader.DataLength = FDCAN_DLC_BYTES_64;
		TxHeader.ErrorStateIndicator = BSP_FDCAN_ESI_ACTIVE;
		TxHeader.BitRateSwitch = BSP_FDCAN_BRS_ON;
		TxHeader.FDFormat = FDCAN_FD_CAN;
		TxHeader.TxEventFifoControl = BSP_FDCAN_NO_TX_EVENTS;
		TxHeader.MessageMarker = 0xCC;
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, TxData2)!= HAL_OK)
		{
			Error_Handler();
		}

		IOIF_msDelay(20);

		// Retrieve first message from Rx FIFO 1
		if(HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO1,&RxHeader, RxData0)!= HAL_OK)
		{
			Error_Handler();
		}
		// Retrieve next message from Rx FIFO 1
		if(HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO1,&RxHeader, RxData1)!= HAL_OK)
		{
			Error_Handler();
		}
		// Retrieve last message from Rx FIFO 1
		if(HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO1,&RxHeader, RxData2)!= HAL_OK)
		{
			Error_Handler();
		}

		// Compare payload to expected data
		if (BufferCmp8b(TxData0, RxData0, 64) != 0) {
			t_passfail = -1; // FAIL
			Error_Handler();
		}
		t_passfail++; // PASS

		// Compare payload to expected data
		if (BufferCmp8b(TxData1, RxData1, 64) != 0) {
			t_passfail = -1; // FAIL
			Error_Handler();
		}
		t_passfail++; // PASS

		// Compare payload to expected data
		if (BufferCmp8b(TxData2, RxData2, 64) != 0) {
			t_passfail = -1; // FAIL
			Error_Handler();
		}
		t_passfail++; // PASS

	}

	static uint16_t t_identifier = 0;
	t_identifier = GUI_SYNC|FDCAN_RESULT;
	Send_MSG(t_identifier, (uint8_t*)(&t_passfail), sizeof(t_passfail));
	MS_enum=IDLE; // so that can do as many times the test, no auto restart for now
	//	fdcan_test_run_start = 0;
	//	MS_enum = FDCAN_RESET; //Start/stop both need SW reset afterwards
}

#ifdef _USE_DEBUG_CLI
static void CLI_IOIF_ModuleInit(void)
{
	IOIF_InitFDCAN1(MD_nodeID);
    IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, NULL);
	CLI_CMDAdd("fdcan",CLI_RunFdcan);

//	IOIF_SysCtrlInit();
//	CLI_CMDAdd("system", CLI_RunSysCtrl);

#if defined(IOIF_LTC2944_ENABLED)
	IOIF_InitBat(IOIF_I2C4);
	CLI_CMDAdd("ltc2944", CLI_RunLtc2944);
#endif

#if defined(FATFS_USB_ENABLE)
	IOIF_FATFS_USB_Init(IOIF_FATFS_USB_MSC, (uint8_t*) "0:");
	CLI_CMDAdd("fatfs_usb", CLI_RunFafts);
#endif

//#if defined(IOIF_TMCS1100A2_ENABLED)
//
//#endif

//#if defined(IOIF_FSR_SZH_HWS004_ENABLED)
//
//#endif


#if defined(IOIF_RMB20IC_ENABLED)
//	IOIF_InitIncEnc(IOIF_TIM5, IOIF_TIM_CHANNEL_ALL , &incData);
	CLI_CMDAdd("rmb20ic",CLI_RunRMB20IC);
#endif

#if defined(IOIF_RMB20SC_ENABLED)
   	IOIF_InitAbsEnc(1, 1, &cliAbs1Data, 3, 0x1FFF, 1000);
	IOIF_InitAbsEnc(3, 2, &cliAbs2Data, 1, 0x1FFF, 1000);//IOIF_SPI3 IOIF_ABS_ENC_ACTUATOR_INPUT
	CLI_CMDAdd("rmb20sc_1",CLI_RunRMB20SC1); //AbsObj1
	CLI_CMDAdd("rmb20sc_2",CLI_RunRMB20SC2); //AbsObj2
#endif

#if defined(IOIF_503NTC_ENABLED)
	CLI_CMDAdd("503ntc",CLI_Run503NTC);
#endif

#if defined(IOIF_BM1422AGMV_ENABLED)
	IOIF_InitMag(IOIF_I2C1);
	CLI_CMDAdd("bm1422agmv",CLI_RunBM1422AGMV);
#endif

#if defined(IOIF_ICM20608G_ENABLED)
	IOIF_Init6Axis(IOIF_I2C2);
	CLI_CMDAdd("icm20608g",CLI_RunICM20608G);
#endif

}
#endif /* _USE_DEBUG_CLI */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
