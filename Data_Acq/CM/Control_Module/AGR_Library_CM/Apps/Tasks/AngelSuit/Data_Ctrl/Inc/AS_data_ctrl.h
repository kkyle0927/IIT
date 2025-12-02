

#ifndef DATA_CTRL_INC_AS_DATA_CTRL_H_
#define DATA_CTRL_INC_AS_DATA_CTRL_H_

#include <inttypes.h>

#include "main.h"

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "data_object_common.h"
#include "ioif_sai_wavplay.h"
#include "ioif_fatfs.h"
#include "ring_buffer.h"

#include "AS_whole_body_ctrl.h"
#include "ioif_pca9957hnmp.h"

#include "ioif_usb_common.h"
#include "csv_parser.h"

#include "AS_system_log.h"
#include "ioif_mcp79510.h"

#include <stdio.h>
#include <string.h>
#include "ff.h" //FatFS 관련 함수 선언 필요
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define CONSTANT_TO_STRING(constant) ConstantToString(constant)

#define LOG_DIR_PATH "0:LOG"
#define LOG_FILE_PREFIX "H10_LOG"

#define MAX_LOG_FOLDERS 10//10
#define MAX_LOG_FILES 	29//29 //파일 제한 후 새로 파일 생성하여 +1 됨

#define MAX_PATH_LEN 100
#define FOLDER_DATE_LEN 6
#define DATE_LEN 10
#define FULL_PATH_LEN 256 + 128
#define H10_ID_LEN (16)

#define MAX_STR_LEN     16

// File Path Configuration
#define MAX_FILE_PATH_LEN       128     // Maximum length for file path strings

// File Rolling Configuration
#define MAX_RECORDS_PER_FILE    (1800 * 1000)  // Maximum sensor records per file before rolling
#define MAX_ROLLING_FILES       1000    // Maximum number of rolling files per session

// SD Card Storage Management
#define SD_MIN_FREE_SPACE_MB    1 // TODO: To be 7680    // Minimum free space to maintain (7.5GB in MB)
#define SD_SPACE_CHECK_INTERVAL 100     // Check SD space every N cycles in StateEnable_Run and StateStandby_Run


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct {
    uint32_t current_file_index;
    uint32_t current_record_count;
    char base_file_path[MAX_FILE_PATH_LEN];     // Base path for rolling files
    char current_file_path[MAX_FILE_PATH_LEN + 16];  // Current active file path
    bool rolling_enabled;
} FileRollingInfo_t;

typedef enum {
    DATA_SAVE_OFF,
    DATA_SAVE_PREPARING,
	DATA_SAVE_ON,
    DATA_SAVE_ABORT
} dataSaveFlag_t;

typedef enum {
	AUDIO_INVALID,
	AUDIO_BEEP_MODE,
	AUDIO_FILE_PLAY,
} audioType_t;


typedef struct /*__attribute__((__packed__))*/
{
    float encoder;
    struct{
        float x;
        float y;
        float z;
    } acc_global;

    struct{
        float x;
        float y;
        float z;
    } gyro_global;

    struct{
        float roll;
        float pitch;
        // float yaw;
    } euler;

    float act_current;

} Sensor_Log_Packet_MD_t;

typedef struct /*__attribute__((__packed__))*/
{
    //CM
    uint32_t seq;       //packet count
    uint32_t assist_ratio;
    uint32_t fsm_state;

    //MD
    Sensor_Log_Packet_MD_t left_hip;
    Sensor_Log_Packet_MD_t right_hip;

//    uint32_t padding;

} Sensor_LogPacket_V1_t;


typedef struct{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
}version_t;

typedef struct __attribute__((__packed__))
{
    uint8_t packet_ver;

    uint8_t period;

    uint32_t time;

    version_t cm_hw_ver;
    version_t cm_sw_ver; 
    version_t lh_md_sw_ver;
    version_t rh_md_sw_ver;
    version_t esp32_sw_ver;

    char h10_id[H10_ID_LEN];

    float left_neutral_posture_angle;
    float right_neutral_posture_angle;
    float trunk_neutral_posture_angle;

    uint32_t tablet_id;
    uint32_t prouser_id;
    uint32_t user_id;

    uint8_t user_gender;
    uint16_t user_height;
    uint16_t user_weight;
    uint16_t user_footsize;

    uint8_t assist_mode;
    float assist_param[8];

//    uint32_t padding;

} MetaItem_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


extern FATFS        fatfsObj;
extern IOIF_FILE_t  fileObj;

extern dataSaveFlag_t dataSaveMode;

extern uint16_t audioVolume;
extern audioType_t audioOutputMode;
extern rtcTime_t RTC_Time;

extern FileRollingInfo_t fileRollingInfo;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitDataCtrl(void);
void RunDataCtrl(void);


#endif /* SUIT_MINICM_ENABLED */


#endif /* APPS_TASKS_ANGELSUIT_DATA_CTRL_INC_AS_DATA_CTRL_H_ */
