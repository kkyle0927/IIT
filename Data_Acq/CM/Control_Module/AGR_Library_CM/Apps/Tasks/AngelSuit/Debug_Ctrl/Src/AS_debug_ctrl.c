

#include "AS_debug_ctrl.h"

#ifdef SUIT_MINICM_ENABLED
/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */
TaskObj_t debugCtrlTask;

uint32_t debug_task_loop_time;

//IOIF_WavPlayState_t audio_test_init = IOIF_WAVPLAY_STATUS_ERROR, audio_play_test = IOIF_WAVPLAY_STATUS_ERROR;
uint8_t audio_outmode = 0;


/* FATFS basic fnc. test */

// IOIF_SD_Status_t sd_test_init_res = IOIF_MSD_ERROR;
// IOIF_fCMD_res_t mount_res = FR_NOT_READY, open_res = FR_NOT_READY, read_res = FR_NOT_READY, write_res= FR_NOT_READY;
IOIF_fCMD_res_t mount_usb_res = FR_NOT_READY, open_usb_res = FR_NOT_READY, read_usb_res = FR_NOT_READY, write_usb_res= FR_NOT_READY;

// FATFS 		  fatfs_test, fatfs_usb_test;
// IOIF_FILE_t   file_t, file_usb_t;

uint8_t SD_readData[30];
uint8_t SD_writeData[30] = "A10_WRITE_TEST_OK!\r\n";

// uint32_t r_byte, w_byte;
bool usb_test_write_state = 0;

float disk_free, disk_free_usb;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes for this module.
 */
/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */
//static void SendTestMsg(void);

/* ----------------------- ROUTINE ------------------------ */

/* -------------------- STATE FUNCTION -------------------- */

static void StateOff_Ent(void);
static void StateStandby_Ent(void);
static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);
static void StateError_Run(void);

#ifdef _USE_DEBUG_CLI
static void CLI_IOIF_ModuleInit(void);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* --------------------- SDO CALLBACK --------------------- */


/* ------------------------- MAIN ------------------------- */
void InitDebugTask()
{
	/* Init Task */
	InitTask(&debugCtrlTask);
	DOPC_AssignTaskID(&debugCtrlTask, TASK_IDX_DEBUG_CTRL);

	IOIF_InitUSB(IOIF_USBD_CDC, IOIF_USB_TIM_NULL);

	/*CLI Enable*/
#ifdef _USE_DEBUG_CLI
	CLI_Init();
	CLI_IOIF_ModuleInit();
#endif /*_USE_DEBUG_CLI*/

	/* State Definition */
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_OFF,     StateOff_Ent,     NULL,            NULL,               false);
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_STANDBY, StateStandby_Ent, NULL,            NULL,               true);
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, StateEnable_Ext,    false);
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_ERROR,   NULL,             StateError_Run,  NULL,               false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&debugCtrlTask,   1, NULL,   	Send_Debug_Msg,  	NULL);

	/* DOD Definition */
	// DOD
	// DOP_CreateDOD(TASK_IDX_DEBUG_CTRL);

	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_DEBUG_CTRL, debugCtrlTask);

	// SDO
	// DOP_COMMON_SDO_CREATE(TASK_IDX_DEBUG_CTRL)


	/* File-system_test */

	//    sd_test_init_res = IOIF_FATFS_SD_Init(IOIF_SD1, (uint8_t*) "0:");
	//
	//    mount_res = IOIF_FileMount(&fatfs_test, (uint8_t*)"0:");
	//    //mount_usb_res = IOIF_FileMount(&fatfs_usb_test, (uint8_t*)"1:");
	//
	//    disk_free = IOIF_Disk_GetFreeSpace((uint8_t*)"0:", &fatfs_test);
	//    //disk_free_usb = IOIF_Disk_GetFreeSpace((uint8_t*)"1:", &fatfs_usb_test);
	//
	//    read_res      = IOIF_FileRead(&file_t, (uint8_t*)"0:A10_ioif_test.txt", SD_readData, sizeof(SD_readData), &r_byte);
	//    //write_res	  = IOIF_FileWrite(&file_t, (uint8_t*)"0:A10_ioif_test2.txt", SD_writeData, sizeof(SD_writeData), &w_byte);
	//    //write_usb_res = IOIF_FileWrite(&file_usb_t, (uint8_t*)"1:A10_ioif_test.txt", SD_readData, sizeof(SD_readData), &w_byte);



	//     audio_test_init = WavAudio_FS_Init((uint8_t*)"0:", IOIF_SAI_WAVPLAY_SD);	 //audio init.

	//PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav");

	//audio_play_test = PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav"); //wave file play
}

void RunDebugTask(void)
{
	RunTask(&debugCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE MACHINE --------------------- */
static void StateOff_Ent(void)
{
	StateTransition(&debugCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Ent(void)
{
	//     PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav");
	// 	osDelay(3000);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, MC_24V_PWR_EN_Pin, IOIF_GPIO_PIN_SET); // Port : MC_24V_PWR_EN_GPIO_Port, Pin : MC_24V_PWR_EN_Pin

	StateTransition(&debugCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	debug_task_loop_time = 0;
	// Ent_Routines(&debugCtrlTask.routine);
}

static void StateEnable_Run(void)
{

	debug_task_loop_time++;
#ifdef _USE_DEBUG_CLI
	CLI_Run();
#endif


}

static void StateEnable_Ext(void)
{

}

static void StateError_Run(void)
{

}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */


#ifdef _USE_DEBUG_CLI

static void CLI_IOIF_ModuleInit(void)
{
	/*System Ctrl*/
	IOIF_SYS_CtrlInit();
	CLI_CMDAdd("system", CLI_RunSysCtrl);

	/*FDCAN*/
	IOIF_InitFDCAN1(NODE_ID_CM);
	IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, NULL);
	IOIF_InitFDCAN2(NODE_ID_CM);
	IOIF_SetFDCANRxCB(IOIF_FDCAN2, IOIF_FDCAN_RXFIFO0CALLBACK, NULL);
	CLI_CMDAdd("fdcan", CLI_RunFdcan);

	/*SAI*/
	IOIF_SAI_PlaySoundInit();
	CLI_CMDAdd("sai_basic", CLI_RunSAICommon);

#if defined(IOIF_BATMONITOR_ENABLED)
	IOIF_BatMonitor_Init();
	CLI_CMDAdd("ltc2944", CLI_RunLtc2944);
#endif

#if defined(IOIF_PCA9957HNMP_ENABLED)
	IOIF_LED24chDriverInit(IOIF_SPI1);
	CLI_CMDAdd("pca9957hnmp", CLI_RunPca9957hnmp);
#endif

#if defined(FATFS_SD_ENABLE)
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_5, IOIF_GPIO_PIN_SET); //Enable SD Card Power On
	IOIF_SYS_BlockingDelay(10);
	IOIF_FATFS_SD_Init(IOIF_SD1, (uint8_t*) "0:");
	CLI_CMDAdd("fatfs_sd", CLI_RunFafts);
#endif

#if defined(IOIF_AUDIO_WAV_SAI_ENABLED)
	IOIF_SAI_WavAudioFSInit((uint8_t*) "1:", IOIF_SAI_WAVPLAY_SD);
	CLI_CMDAdd("sai_fatfs", CLI_RunSAIWav);
#endif

#if defined(IOIF_MAGNETO_ENABLED)
	IOIF_Magneto_Init();
#endif

#if defined(IOIF_ACCGYRO_ENABLED)
	IOIF_AccGyro_Init();
#endif

#if defined(IOIF_ESP32_ENABLED)
	IOIF_ESP32_Init();
	CLI_CMDAdd("esp32", CLI_RunEsp32);
#endif

#if defined(IOIF_MCP79510_ENABLED)
	IOIF_MCP79510_Init();
	CLI_CMDAdd("mcp79510", CLI_RunMCP79510);
#endif

#if defined(IOIF_MDBT42Q_AT_ENABLED)
	IOIF_BLE_Init(9600);
	IOIF_BLE_SetBaudrate115200();
	CLI_CMDAdd("mdbt42qat", CLI_RunMdbt42qat);
#endif

}

/* ----------------------- ROUTINE ------------------------ */
#endif /*_USE_DEBUG_CLI*/
#endif /* SUIT_MINICM_ENABLED */
