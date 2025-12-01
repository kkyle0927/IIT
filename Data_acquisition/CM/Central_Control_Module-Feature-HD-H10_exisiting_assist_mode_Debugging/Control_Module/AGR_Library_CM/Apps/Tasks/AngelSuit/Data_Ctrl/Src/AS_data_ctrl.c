

#include "AS_data_ctrl.h"
#include "AS_Audio_Ctrl.h"
#include "queue.h"
#include "ioif_tim_common.h"
// #include "bsp_common.h"  // Commented out until BinSem_SDMMC1Handle is available

#define SENSOR_DATA_BUFFER_SIZE (15 * 1024)
#define ROBOT_LOG_ROOT "0:RobotData"
#define ROBOT_LOG_NAME "sensordata.bin"
#define ROBOT_LOG_META_NAME "meta.json"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

TaskObj_t dataCtrlTask;

/* default audio value */
SUIT_AudioState_t  SuitAudioState = {
		.audio_Isplay = false,
		.audio_playback = false,
		.audio_id = AUDIO_ID_INVALID,
		.latest_played_audio_id = AUDIO_ID_INVALID,
		.audio_lang = AUDIO_LANGUAGE_KOREAN,
};

// AUDIO_ID_xxx 의 index 와 맞춰서 사용
const char* g_AudioFileNames[] =
{
    "INVALID",                               // 0: AUDIO_ID_INVALID
    "001_system_init_volum_changed.wav",     // 1
    "002_gait_start.wav",                    // 2
    "003_left_neutral_posture_not_cal.wav",  // 3
    "004_right_neutral_posture_not_cal.wav", // 4
    "005_both_neutral_posture_not_cal.wav",  // 5
    "006_left_comm_error.wav",               // 6
    "007_right_comm_error.wav",              // 7
    "008_both_comm_error.wav",               // 8
    "009_left_imu_error.wav",                // 9
    "010_right_imu_error.wav",               // 10
    "011_both_imu_error.wav",                // 11
    "012_neutral_completed.wav",             // 12
    "013_battery5_alarm.wav",                // 13
    "014_system_init_volum_changed.wav",     // 14
    "015_system_exit.wav",                   // 15
    "016_audio_preview.wav",                 // 16
    "017_battery15_alarm.wav",               // 17
    "018_safety_warning.wav",                // 18
    "019_assist_to_standby.wav",             // 19
    "020_gait_assist_mode.wav",              // 20
    "021_resist_mode.wav",                   // 21
    "022_load_adjust_mode.wav",              // 22
    "023_joint_limit_mode.wav",              // 23
    "INVALID",                               // 24
    "025_RM_init_fail.wav",                  // 25
    "026_RM_system_error_poweroff.wav",      // 26
    "027_RM_system_overtemp_warning.wav",    // 27
    "028_RM_system_overtemp_poweroff.wav",   // 28
    "029_RM_motor_overtemp_poweroff.wav",    // 29
    "030_RM_motor_undervlot.wav",            // 30
    "031_RM_sdcard_contentsfile_missing.wav",// 31
    "032_RM_sdcard_full.wav",                // 32
    "INVALID",                               // 33
    "INVALID",                               // 34
    "035_sts_mode.wav",                      // 35
    "036_sitting_ready.wav",                 // 36
    "037_sitting_start.wav",                 // 37
    "038_standing_ready.wav",                // 38
    "039_standing_start.wav",                // 39
    "040_standing.wav",                      // 40
    "041_sitting.wav",                       // 41
};

static const char* const g_AudioDriveName[AUDIO_LANGUAGE_MAX] =
{
    "1:",   // 한국어 drive name
    "1:",   // 영어 drive name (굳이 구분지을 필요는 없지만 만들어둠)
};

static const char* const g_AudioLangDirectory[AUDIO_LANGUAGE_MAX] =
{
    "AudioFiles/",   	// Default, Korean
    "AudioFiles/EN/",   // English
};


extern osSemaphoreId_t BinSem_PlaySoundHandle;

/* FATFS basic fnc. test */
//IOIF_SD_Status_t initSDcardRes = IOIF_MSD_ERROR;
extern IOIF_SD_Status_t initSDcardRes;
extern bool audio_end;

/* System logger */
static uint32_t _datactrl_standby_run_count = 0;
static uint32_t _log_write_successcnt = 0;
bool sdcard_ready_to_write = false;
static bool _log_fileopen_status = false;
uint8_t IsSDCard_busy = 0;

extern bool 	log_test_on;
extern uint8_t  manu_mode;
extern bool 	audio_check;
extern bool 	init_error;

IOIF_fCMD_res_t mountRes = FR_NOT_READY;
IOIF_fCMD_res_t openRes = FR_NOT_READY;
IOIF_fCMD_res_t readRes = FR_NOT_READY;
IOIF_fCMD_res_t writeRes = FR_NOT_READY;
IOIF_fCMD_res_t syncRes = FR_NOT_READY;

IOIF_fCMD_res_t loopend_FileClose_res = FR_NOT_READY;

FATFS		fatfsObj 	__attribute__((section(".FATFS_RAMD1_data")));
IOIF_FILE_t	fileObj		__attribute__((section(".FATFS_RAMD1_data")));
static uint8_t _sensor_data_buf[SENSOR_DATA_BUFFER_SIZE] __attribute__((section(".FATFS_RAMD1_data")));


dataSaveFlag_t dataSaveMode = DATA_SAVE_ABORT;
audioType_t	   audioOutputMode = AUDIO_BEEP_MODE;

uint8_t  audioLanguage = 0;
uint16_t audioVolume = 100;

DIR directory;

static IOIF_FILE_t _meta_file __attribute__((section(".FATFS_RAMD1_data")));

static uint16_t _sensor_position = 0;
static uint32_t _buffer_overflow = 0;

static bool _data_save_mode = false; 			// TRUE => data, FALSE => System Log

FileRollingInfo_t fileRollingInfo = {0};



/**
 *------------------------------------------------------------
 *                           VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

uint32_t dataCtrlEnableLoopCnt;

static uint32_t _writeByte = 1;

uint32_t writeFailCnt;
uint32_t mountFailCnt;

static bool _fileConsecutiveWrite = false;

//static uint16_t file_index = 1; 				// 파일 인덱스
//static uint32_t current_row_count = 0; 			// 현재 파일의 행 수
//static uint32_t current_file_size = 0; 			// 현재 파일 크기(바이트)

static char _system_log_date[16];
static char _system_log_time[50];

extern MetaItem_t meta;


osMessageQueueId_t sensorDataLogQueueHandle;
const osMessageQueueAttr_t sensorDataLogQueue_attributes = {
  .name = "SensorDataLogQueue"
};

uint8_t json_buf[512] __attribute__((section(".FATFS_RAMD1_data")));

uint8_t file_path[128];
uint16_t str_pos=1;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Ent(void);
static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ----------------------- FUNCTION ----------------------- */
static void _SaveTrainingMetadata(uint8_t*);
static FRESULT _CreateFileDirectory(rtcTime_t *rtc);
static FRESULT _ManageLogStorage(bool is_folder_mode);
static void _InitFileRolling(const char* base_path);
static IOIF_fCMD_res_t _RollToNextFile(void);
static IOIF_fCMD_res_t _ProcessSensorLogQueueWithRolling(void);
static bool _CheckSDCardSpace(void);
static void _ProcessSystemLogging(void);
static IOIF_fCMD_res_t _InitSensorLogSession(void);

static const char* Audio_BuildFullPath(SUIT_Audio_Language lang, int audio_id);

//static void OpenNewFile(void);
//static void UpdateRowCount(const uint8_t *buffer, uint_fast16_t bufferSize);

/* ----------------------- ROUTINE ------------------------ */
/* --------------------- SDO CALLBACK --------------------- */


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(dataCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitDataCtrl(void)
{
	/* Init Task */
    InitTask(&dataCtrlTask);
	DOPC_AssignTaskID(&dataCtrlTask, TASK_IDX_DATA_CTRL);

	/* Init Device */
	sensorDataLogQueueHandle = osMessageQueueNew (150, sizeof(Sensor_LogPacket_V1_t), &sensorDataLogQueue_attributes);

	/* State Definition */
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,				true);
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_STANDBY,  StateStandby_Ent,	StateStandby_Run,	NULL,       		false);
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);



	/* Routine Definition */

	/* DOD Definition */
	// DOD
	// DOP_CreateDOD(TASK_IDX_DATA_CTRL);

   	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_DATA_CTRL, dataCtrlTask);

	// SDO
	// DOP_COMMON_SDO_CREATE(TASK_IDX_DATA_CTRL)
	#ifdef SENSOR_DATA_LOGGING_ENABLED

	_data_save_mode = true;

	#endif
}

void RunDataCtrl(void)
{
	RunTask(&dataCtrlTask);
}


/* ----------------------- FUNCTION ----------------------- */


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE MACHINE --------------------- */
static void StateOff_Run(void)
{
	dataCtrlEnableLoopCnt = 0;

	uint8_t t_wholebodyCtrlState = DOPC_GetTaskState(TASK_IDX_WHOLE_BODY_CTRL);

	if (t_wholebodyCtrlState == TASK_STATE_STANDBY || t_wholebodyCtrlState == TASK_STATE_ENABLE) {
		mountRes = IOIF_FileMount(&fatfsObj, (uint8_t*)"0:");	// Mount the file system
		StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_STANDBY);
	}

}

static void StateStandby_Ent(void)
{
	_datactrl_standby_run_count=0;

}

static void StateStandby_Run(void)
{
	if (rm_power_off == true )
		audioOutputMode = AUDIO_INVALID; // 전원 꺼지기 전 md 저장을 위해 대기 시, 오디오 중복 재생 방지

	uint8_t t_wholebodyCtrlState = DOPC_GetTaskState(TASK_IDX_WHOLE_BODY_CTRL);
	// Check SD card space periodically
	if (t_wholebodyCtrlState == TASK_STATE_ENABLE && _datactrl_standby_run_count % SD_SPACE_CHECK_INTERVAL == 0) {
		bool space_available = _CheckSDCardSpace();
		if (!space_available) {
			dataSaveMode = DATA_SAVE_OFF;
		} else if (dataSaveMode == DATA_SAVE_OFF) {
			dataSaveMode = DATA_SAVE_ON;
		}
	}

#if defined(_USE_DEBUG_CLI) || defined(USB_LOG_ENABLED)
	if(audioOutputMode == AUDIO_FILE_PLAY && dataSaveMode == DATA_SAVE_ABORT)	// USB log 기능 사용 시 USB detection 시 오디오 재생 사용 가능하게 변경
#else
	if(audioOutputMode == AUDIO_FILE_PLAY && dataSaveMode == DATA_SAVE_ABORT && IOIF_IsUSBD_connected() == false)
#endif
	{
		//if(!(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 63)) {
		if (osSemaphoreAcquire(BinSem_PlaySoundHandle, osWaitForever) == osOK)
		        {
		            if ((SuitAudioState.audio_Isplay == false) &&
		                (SuitAudioState.audio_id != AUDIO_ID_INVALID))
		            {

		            	SuitAudioState.audio_lang = audioLanguage;		//audio language update

		            	const char* drive_name = g_AudioDriveName[SuitAudioState.audio_lang];
		            	const char* full_path  = Audio_BuildFullPath(SuitAudioState.audio_lang,SuitAudioState.audio_id);
		                if (drive_name != NULL && full_path != NULL)
		                {
		                    SuitAudioState.audio_Isplay = true;
		                    IsSDCard_busy = 1;

		                    IOIF_SAI_PlayWaveFile((uint8_t*)drive_name,(uint8_t*)full_path, audioVolume);

		                    if (audio_end == true)
		                    {
		                        SuitAudioState.audio_Isplay = false;
		                        IsSDCard_busy = 0;
		                    }

		                    SuitAudioState.latest_played_audio_id = SuitAudioState.audio_id;
		                }

		                SuitAudioState.audio_id     = AUDIO_ID_INVALID;
		                audioOutputMode             = AUDIO_INVALID;
		            }
		        }
	}
	else if (dataSaveMode == DATA_SAVE_PREPARING && manu_mode != 1 && _data_save_mode){
		StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}
	else {
		// Process system logging when not doing sensor logging
		_ProcessSystemLogging();
	}

	_datactrl_standby_run_count++;
}


static void StateEnable_Ent(void)
{
    dataCtrlEnableLoopCnt = 0;    
    // Initialize sensor log session
    IOIF_fCMD_res_t init_result = _InitSensorLogSession();
    
    if (init_result != FR_OK) {
        StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ERROR);
        return;
    }
}

static void StateEnable_Run(void)
{
    // Enable sensor data saving now that initialization is complete
    if (dataSaveMode == DATA_SAVE_PREPARING) {
        dataSaveMode = DATA_SAVE_ON;
    }

    if (dataSaveMode == DATA_SAVE_ABORT)
    {
        StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_STANDBY);
    }
    if (openRes != FR_OK)
    {
    	StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ERROR);
    	return;
    }

    // Check SD card space periodically
    if (dataCtrlEnableLoopCnt % SD_SPACE_CHECK_INTERVAL == 0) {
        bool space_available = _CheckSDCardSpace();
        if (!space_available) {
            dataSaveMode = DATA_SAVE_OFF;
            StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_STANDBY);
            return;
        } else if (dataSaveMode == DATA_SAVE_OFF) {
            dataSaveMode = DATA_SAVE_ON;
        }
    }

    // Init Sensor Data Buffer0
    syncRes = IOIF_FileSync(&fileObj);

    writeRes = _ProcessSensorLogQueueWithRolling();
    
    if (writeRes == FR_INT_ERR) {
        StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ERROR);
        return;
    }

    dataCtrlEnableLoopCnt++;

    uint8_t t_rmState = DOPC_GetTaskState(TASK_IDX_RISKMNGT_HDLR);
    if (t_rmState == TASK_STATE_ERROR)
        StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ERROR);

}

static void StateEnable_Ext(void)
{
    writeRes = _ProcessSensorLogQueueWithRolling();
    
    if (writeRes == FR_INT_ERR) {
        StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ERROR);
        return;
    }

    /* 파일 닫기 */
    if (openRes == FR_OK) {
        loopend_FileClose_res = IOIF_FileClose(&fileObj);
        openRes = FR_NOT_READY;
        writeRes = FR_NOT_READY;
    }
    
    /* 롤링 상태 초기화 */
    memset(&fileRollingInfo, 0, sizeof(fileRollingInfo));

//    /* SD 카드 언마운트 (필요 시) */
//    if (dataSaveMode != DATA_SAVE_ON) {
//        mountRes = IOIF_FileUnmount((uint8_t*)"0:");
//    }

    /* 상태 초기화 */
    _fileConsecutiveWrite = false;
}

static void StateError_Run(void)
{
	static bool once = false;

	static uint8_t log_write_cnt = 0;
	static int8_t  log_write_res = 0;

	if(once == false) {
	_ManageLogStorage(true);

	if(openRes != FR_OK)
	{
		memset(_system_log_date,0,sizeof(_system_log_date));
		memset(_system_log_time,0,sizeof(_system_log_time));

		_CreateFileDirectory(&RTC_Time);
		openRes = _ManageLogStorage(false);

		/* 8. 새 파일 생성 및 시간 설정 */
		_CreateFileDirectory(&RTC_Time);
		if(IOIF_FileOpenCreate(&fileObj, (uint8_t*)_system_log_time) != FR_OK){
			_log_fileopen_status = false;			// log file open fails
		}
		else {
			_log_fileopen_status = true;
		}

		sdcard_ready_to_write = true;
	}
	else if(openRes == FR_OK && audio_check == true)
	{
		log_write_res = log_write_to_sdcard(&fileObj, &_writeByte);
	}
	else if(openRes == FR_OK && log_write_cnt > 19)
	{
		//osMutexAcquire(SDIOmutexHandle, BUFFER_SD_TIMEOUT);					// Mutex Lock
		log_write_res = log_write_to_sdcard(&fileObj, &_writeByte);

		if(log_write_res == 0)			//No error
			_log_write_successcnt++;
		else	//write fail then retry
		{
			//				IOIF_FileClose(&fileObj);
			//				openRes = FR_NOT_READY;
			//				sdcard_ready_to_write = false;
		}
		log_write_cnt = 0;

		//osMutexAcquire(SDIOmutexHandle, BUFFER_SD_TIMEOUT);					// Mutex Unlock
	}
	log_write_cnt++;

	if(_log_fileopen_status == true)	IOIF_FileClose(&fileObj);  // open 되었을때만 close
	openRes = FR_NOT_READY;
	sdcard_ready_to_write = false;
	//log_init();						// log re-init.

	once  = true;

	}



}


/* ----------------------- FUNCTION ----------------------- */


static void _InitFileRolling(const char* base_path)
{
    fileRollingInfo.current_file_index = 0;
    fileRollingInfo.current_record_count = 0;
    fileRollingInfo.rolling_enabled = true;
    
    strncpy(fileRollingInfo.base_file_path, base_path, sizeof(fileRollingInfo.base_file_path) - 1);
    fileRollingInfo.base_file_path[sizeof(fileRollingInfo.base_file_path) - 1] = '\0';
    
    // Generate first file path
    snprintf(fileRollingInfo.current_file_path, sizeof(fileRollingInfo.current_file_path),
             "%s_%03lu.bin", fileRollingInfo.base_file_path, (unsigned long)fileRollingInfo.current_file_index);
}

static IOIF_fCMD_res_t _RollToNextFile(void)
{
    IOIF_fCMD_res_t result = FR_OK;
    
    // Close current file
    if (openRes == FR_OK) {
        IOIF_FileClose(&fileObj);
    }
    
    // Increment file index
    fileRollingInfo.current_file_index++;
    fileRollingInfo.current_record_count = 0;
    
    // Check maximum files limit
    if (fileRollingInfo.current_file_index >= MAX_ROLLING_FILES) {
        // Stop rolling and return error
        fileRollingInfo.rolling_enabled = false;
        return FR_INT_ERR;
    }
    
    // Generate new file path
    snprintf(fileRollingInfo.current_file_path, sizeof(fileRollingInfo.current_file_path),
             "%s_%03lu.bin", fileRollingInfo.base_file_path, (unsigned long)fileRollingInfo.current_file_index);
    
    // Open new file
    openRes = IOIF_FileOpenCreate(&fileObj, (uint8_t*)fileRollingInfo.current_file_path);
    
    return openRes;
}

static IOIF_fCMD_res_t _ProcessSensorLogQueueWithRolling(void)
{
    IOIF_fCMD_res_t result = FR_INT_ERR;
    uint32_t records_to_write = 0;
    
    // TODO: Add semaphore protection when BinSem_SDMMC1Handle is available
    // if (osSemaphoreAcquire(BinSem_SDMMC1Handle, osWaitForever) == osOK) {
        memset(_sensor_data_buf, 0, SENSOR_DATA_BUFFER_SIZE);
        _sensor_position = 0;

        while( osMessageQueueGet(sensorDataLogQueueHandle, &_sensor_data_buf[_sensor_position], NULL, 0) == osOK )
        {
            _sensor_position += sizeof(Sensor_LogPacket_V1_t);
            records_to_write++;
            
            if ((_sensor_position + sizeof(Sensor_LogPacket_V1_t)) > SENSOR_DATA_BUFFER_SIZE)
            {
                _buffer_overflow++;
                // osSemaphoreRelease(BinSem_SDMMC1Handle);
                return FR_INT_ERR; // Buffer overflow error
            }
        }

        // Check if we need to roll to next file
        if (fileRollingInfo.rolling_enabled && 
            (fileRollingInfo.current_record_count + records_to_write) > MAX_RECORDS_PER_FILE) {
            
            result = _RollToNextFile();
            if (result != FR_OK) {
                // osSemaphoreRelease(BinSem_SDMMC1Handle);
                return result;
            }
        }

        // Write data to current file
        result = IOIF_fWrite(&fileObj, _sensor_data_buf, _sensor_position, &_writeByte);
        
        // Update record count
        if (result == FR_OK) {
            fileRollingInfo.current_record_count += records_to_write;
        }
        
        // osSemaphoreRelease(BinSem_SDMMC1Handle);
    // }
    
    return result;
}

static bool _CheckSDCardSpace(void)
{
    // Use existing IOIF function to get free space
    uint64_t free_space_bytes = IOIF_Disk_GetFreeSpace((uint8_t*)"0:", (IOIF_FATFS_t*)&fatfsObj);
    
    // Check for error return value
    if (free_space_bytes == (uint64_t)-1) {
        return false;
    }
    
    // Convert bytes to MB  
    uint32_t free_space_mb = (uint32_t)(free_space_bytes / (1024 * 1024));
    
    // Check if we have enough free space
    if (free_space_mb < SD_MIN_FREE_SPACE_MB) {
        FaultArrange_t ID = {6,1};
        FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_MID, FaultIDRegister, &ID, sizeof(ID));
        return false;
    }
    
    return true;
}


static void _ProcessSystemLogging(void)
{
    // Check conditions for system logging
    if (!(log_test_on == true && IOIF_IsUSBD_connected() == false && 
          _datactrl_standby_run_count > 150 && manu_mode != 1 && !_data_save_mode)) {
        // Cleanup if logging is disabled
        if((log_test_on == false && openRes == FR_OK) || manu_mode == 1) {
            if(_log_fileopen_status == true) IOIF_FileClose(&fileObj);
            openRes = FR_NOT_READY;
            sdcard_ready_to_write = false;
        }
        else if(IOIF_IsUSBD_connected() == true) { // USB port is connected
            _datactrl_standby_run_count = 0;
            if(_log_fileopen_status == true) IOIF_FileClose(&fileObj);
            openRes = FR_NOT_READY;
            sdcard_ready_to_write = false;
        }
        return;
    }
    
    // System logging logic
    static uint8_t log_write_cnt = 0;
    static int8_t  log_write_res = 0;

    _ManageLogStorage(true);

    if(openRes != FR_OK) {
        memset(_system_log_date,0,sizeof(_system_log_date));
        memset(_system_log_time,0,sizeof(_system_log_time));

        _CreateFileDirectory(&RTC_Time);
        openRes = _ManageLogStorage(false);

        // Create new file and set time
        _CreateFileDirectory(&RTC_Time);
        if(IOIF_FileOpenCreate(&fileObj, (uint8_t*)_system_log_time) != FR_OK){
            _log_fileopen_status = false; // log file open fails
        }
        else {
            _log_fileopen_status = true;
        }
        sdcard_ready_to_write = true;
    }
    else if(openRes == FR_OK && audio_check == true) {
        log_write_res = log_write_to_sdcard(&fileObj, &_writeByte);
    }
    else if(openRes == FR_OK && log_write_cnt > 19) {
        log_write_res = log_write_to_sdcard(&fileObj, &_writeByte);

        if(log_write_res == 0) { // No error
            _log_write_successcnt++;
        }
        else { // write fail then retry
            // Error handling could be added here
        }
        log_write_cnt = 0;
    }
    log_write_cnt++;
}

static IOIF_fCMD_res_t _InitSensorLogSession(void)
{
	FRESULT res = FR_OK;

    dataCtrlEnableLoopCnt = 0;
    
    // Check SD card mount
    if (mountRes != FR_OK) {
        mountRes = IOIF_FileMount(&fatfsObj, (uint8_t*)"0:");
        if (mountRes != FR_OK) {
            return FR_NOT_READY;
        }
    }
    
    // Create session directory and initialize files
    uint8_t file_path[128];
    uint8_t rtc_date[7];
    uint8_t rtc_time[7];
    rtcTime_t rtc = {0,};
    bool rtc_valid = false;
    
    // Try to get RTC time (max 3 retries)
    for (int retry = 0; retry < 3; retry++) {
        if (IOIF_MCP79510_GetTime(&rtc) == IOIF_MCP79510_STATE_OK) {
            rtc_valid = true;
            snprintf((char*)rtc_date, 7, "%02d%02d%02d", 
                     (rtc.year % 100), (rtc.mon % 100), (rtc.mday % 100));
            snprintf((char*)rtc_time, 7, "%02d%02d%02d", 
                     (rtc.hour % 100), (rtc.min % 100), (rtc.sec % 100));
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Create directory structure
    if (rtc_valid) {
        snprintf((char*)file_path, sizeof(file_path), "%s/%s", ROBOT_LOG_ROOT, rtc_date);
        res = f_mkdir((const TCHAR*)file_path);
        
        snprintf((char*)file_path, sizeof(file_path), "%s/%s/%s", ROBOT_LOG_ROOT, rtc_date, rtc_time);
        res = f_mkdir((const TCHAR*)file_path);
    } else {
        // Fallback to system tick if RTC fails
        uint32_t tick = IOIF_GetTick();
        snprintf((char*)file_path, sizeof(file_path), "0:RobotData/NO_RTC_%lu_", tick);
        res = f_mkdir((const TCHAR*) file_path);
    }
    
    // Wait for SD card to be ready (audio playback to finish)
    while(IsSDCard_busy == 1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Save metadata and initialize rolling files
    _SaveTrainingMetadata(file_path);
    
    if (openRes != FR_OK) {
        return openRes;
    }
    
    // Set initialization flags
    _fileConsecutiveWrite = true;
    writeFailCnt = 0;
    
    return res;
}

static void _SaveTrainingMetadata(uint8_t* pathname) {

	memset(json_buf, 0, sizeof(json_buf));

	// Metadata is written only once at session start, no need for semaphore protection
	str_pos = snprintf((char*)json_buf, sizeof(json_buf), "{\r\n"\
		"\"pv\": %d,\r\n"\
		"\"p\": %d,\r\n"\
		"\"t\": %lu,\r\n"\
		"\"hwv\": \"%d.%d.%d\",\r\n"\
		"\"swv\": \"%d.%d.%d\",\r\n"\
		"\"lhv\": \"%d.%d.%d\",\r\n"\
		"\"rhv\": \"%d.%d.%d\",\r\n"\
		"\"espv\": \"%d.%d.%d\",\r\n"\
		"\"rid\": \"%s\",\r\n"\
		"\"lnp\": %.2f,\r\n"\
		"\"rnp\": %.2f,\r\n"\
        "\"tnp\": %.2f,\r\n"\
        "\"tid\": %lu,\r\n"\
		"\"pid\": %lu,\r\n"\
		"\"uid\": %lu,\r\n"\
		"\"gen\": %d,\r\n"\
		"\"hgt\": %d,\r\n"\
		"\"wgt\": %d,\r\n"\
		"\"fsz\": %d,\r\n"\
		"\"mod\": %d,\r\n"\
		"\"asp0\": %.2f,\r\n"\
		"\"asp1\": %.2f,\r\n"\
		"\"asp2\": %.2f,\r\n"\
		"\"asp3\": %.2f,\r\n"\
		"\"asp4\": %.2f,\r\n"\
		"\"asp5\": %.2f,\r\n"\
		"\"asp6\": %.2f,\r\n"\
		"\"asp7\": %.2f\r\n"\
	"}", \
	meta.packet_ver, meta.period, (unsigned long)meta.time, \
	meta.cm_hw_ver.major, meta.cm_hw_ver.minor, meta.cm_hw_ver.patch, \
	meta.cm_sw_ver.major, meta.cm_sw_ver.minor, meta.cm_sw_ver.patch, \
	meta.lh_md_sw_ver.major, meta.lh_md_sw_ver.minor, meta.lh_md_sw_ver.patch, \
	meta.rh_md_sw_ver.major, meta.rh_md_sw_ver.minor, meta.rh_md_sw_ver.patch, \
	meta.esp32_sw_ver.major, meta.esp32_sw_ver.minor, meta.esp32_sw_ver.patch, \
	meta.h10_id, \
	meta.left_neutral_posture_angle, meta.right_neutral_posture_angle, meta.trunk_neutral_posture_angle, \
	(unsigned long)meta.tablet_id, (unsigned long)meta.prouser_id, (unsigned long)meta.user_id, \
	meta.user_gender, meta.user_height, meta.user_weight, meta.user_footsize, \
	meta.assist_mode, \
	meta.assist_param[0], meta.assist_param[1], meta.assist_param[2], \
	meta.assist_param[3], meta.assist_param[4], meta.assist_param[5], \
	meta.assist_param[6], meta.assist_param[7]);

	// 버퍼 오버플로우 체크
	if (str_pos >= sizeof(json_buf)) {
		str_pos = sizeof(json_buf) - 1; // null terminator 고려
	}
	snprintf((char*)file_path, sizeof(file_path), "%s/%s", pathname, ROBOT_LOG_META_NAME);

	openRes = IOIF_FileOpenCreate(&_meta_file, file_path);
	
	writeRes = IOIF_fWrite(&_meta_file, json_buf, str_pos, &_writeByte);
	
	vTaskDelay( 100 / portTICK_PERIOD_MS );
	IOIF_FileSync(&_meta_file); 
	IOIF_FileClose(&_meta_file);

	memset(file_path, 0, sizeof(file_path));
	
	// Initialize file rolling with base path
	char base_file_path[128];
	snprintf(base_file_path, sizeof(base_file_path), "%s/sensordata", pathname);
	_InitFileRolling(base_file_path);
	
	// Open first rolling file
	openRes = IOIF_FileOpenCreate(&fileObj, (uint8_t*)fileRollingInfo.current_file_path);
}


static FRESULT _CreateFileDirectory(rtcTime_t *rtc) {
    char date_str[DATE_LEN];
    FRESULT res;

	IOIF_MCP79510_GetTime(&RTC_Time);

    /*folder name*/
	snprintf(date_str, sizeof(date_str), "%02d%02d%02d", (rtc->year % 100), (rtc->mon % 100), (rtc->mday % 100));
	snprintf(_system_log_date, sizeof(_system_log_date), "%s/%s", LOG_DIR_PATH, date_str);

    /*txt file name*/
	snprintf(_system_log_time, sizeof(_system_log_time), "%s/%s_%s_%02d.%02d.%02d.txt", _system_log_date, LOG_FILE_PREFIX, date_str, rtc->hour, rtc->min, rtc->sec);

    res = f_mkdir(_system_log_date);

    return res;
}

static FRESULT _ManageLogStorage(bool is_folder_mode) {
    DIR dir;
    FILINFO fno;
    uint32_t oldest_item = 0xFFFFFFFF;
    uint8_t item_count = 0;
    char oldest_path[MAX_PATH_LEN] = {0};
    char full_path[FULL_PATH_LEN] = {0};

    const char* target_path = is_folder_mode ? LOG_DIR_PATH : _system_log_date;
    FRESULT res = f_opendir(&dir, target_path);
    if (res != FR_OK) return res;

    memset(&fno, 0, sizeof(fno));

    while ((res = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != 0) {
        // 모드에 따라 처리 대상 필터링
        if ((is_folder_mode && !(fno.fattrib & AM_DIR)) ||
            (!is_folder_mode && strncmp(fno.fname, LOG_FILE_PREFIX, strlen(LOG_FILE_PREFIX)) != 0)) {
            continue;
        }

        item_count++;
        snprintf(full_path, sizeof(full_path), "%s/%s", target_path, fno.fname);

        if (is_folder_mode) {
            // 폴더 모드: 날짜 기반 정렬
            char date_str[FOLDER_DATE_LEN + 1] = {0};
            strncpy(date_str, fno.fname, FOLDER_DATE_LEN);
            uint32_t item_date = atoi(date_str);

            if (item_date < oldest_item) {
                oldest_item = item_date;
                strncpy(oldest_path, full_path, sizeof(oldest_path));
            }
        } else {
            // 파일 모드: 시간 기반 정렬
            char* last_underscore = strrchr(fno.fname, '_');
            if (last_underscore != NULL && strlen(last_underscore) > 8) {
                uint8_t tt = 0, mm = 0, ss = 0;
                if (sscanf(last_underscore + 1, "%2hhu.%2hhu.%2hhu", &tt, &mm, &ss) == 3) {
                    tt = (tt > 23) ? 23 : tt;
                    mm = (mm > 59) ? 59 : mm;
                    ss = (ss > 59) ? 59 : ss;

                    uint32_t current_time = tt * 3600 + mm * 60 + ss;
                    if (current_time < oldest_item) {
                        oldest_item = current_time;
                        strncpy(oldest_path, full_path, sizeof(oldest_path));
                    }
                }
            }
        }
    }

    f_closedir(&dir);

    uint8_t max_items = is_folder_mode ? MAX_LOG_FOLDERS : MAX_LOG_FILES;
    if (item_count > max_items && oldest_path[0] != '\0') {
    	res = f_unlink(oldest_path);

    	if (res == FR_DENIED) {

    		DIR delete_dir;
    		FRESULT delete_res = f_opendir(&delete_dir, oldest_path);
    		if (delete_res == FR_OK) {
    			FILINFO delete_fno;
    			while (f_readdir(&delete_dir, &delete_fno) == FR_OK && delete_fno.fname[0] != 0) {
    				char sub_path[FULL_PATH_LEN];
    				snprintf(sub_path, sizeof(sub_path), "%s/%s", oldest_path, delete_fno.fname);
    				f_unlink(sub_path);
    			}
    			f_closedir(&delete_dir);
    		}

    		f_chmod(oldest_path, 0, AM_RDO);
    		res = f_unlink(oldest_path);
    	}
    }

    return res;

    /* 지우지 마세요 - 추후 test 해볼 에정
     while(1)
     {
     	FILINFO current_file;
     	memset(&current_file, 0, sizeof(current_file));
     	openRes = f_readdir(&directory, &current_file);
     	if (openRes != FR_OK || current_file.fname[0] == 0) break;

     	if(strncmp(current_file.fname, LOG_FILE_PREFIX, strlen(LOG_FILE_PREFIX)) == 0)
     	{
     		file_count++;

     		// 파일 시간 계산 예시 (date + time → uint32)
     		uint32_t file_time = ((uint32_t)current_file.fdate << 16) | current_file.ftime;

     		// 전체 경로 생성
     		snprintf(full_path, sizeof(full_path), "%s/%s", _system_log_date, current_file.fname);

     		if(file_time < oldest_time)
     		{
     			oldest_time = file_time;
     			strcpy(oldest_file_path, full_path);
     		}
     	}

     }

     FILINFO new_file_time;
     memset(&new_file_time, 0, sizeof(new_file_time));
     new_file_time.fdate = ((RTC_Time.year + 2000 - 1980) << 9) | (RTC_Time.mon << 5) | RTC_Time.mday;
     new_file_time.ftime = (RTC_Time.hour << 11) | (RTC_Time.min << 5) | (RTC_Time.sec / 2);  // FatFs는 초를 2초 단위로 저장
           				f_utime(_system_log_time, &new_file_time);
     */
}

//static void OpenNewFile(void)
//{
//	// TODO : 데이터 저장 날짜와 시간 필요!?
//    char filename[64];
//    sprintf(filename, "0:RobotData/SUIT_LOGGED_DATA-%d.csv", file_index++);
//
//    // 파일 열기
//    openRes = IOIF_FileOpenCreateAppend(&fileObj, (uint8_t*)filename);
//    _fileConsecutiveWrite = (openRes == FR_OK); // 파일 열기 성공 시 연속 쓰기 활성화
//    current_row_count = 0;
//    current_file_size = 0;
//}
//
//// 데이터의 개행 문자 수 계산하여 행 수 업데이트
//static void UpdateRowCount(const uint8_t *buffer, uint_fast16_t bufferSize)
//{
//    for (int i = 0; i < bufferSize; i++) {
//        if (buffer[i] == '\n') {
//            current_row_count++;
//        }
//    }
//}




static const char* Audio_BuildFullPath(SUIT_Audio_Language lang, int audio_id)
{
    static char fullpath[128]; // 충분한 길이로 확보

    const char* dir  = g_AudioLangDirectory[lang];
    const char* file = g_AudioFileNames[audio_id];

    if (dir == NULL || file == NULL)
        return NULL;

    snprintf(fullpath, sizeof(fullpath), "%s%s", dir, file);
    return fullpath;
}


#endif /* SUIT_MINICM_ENABLED */
