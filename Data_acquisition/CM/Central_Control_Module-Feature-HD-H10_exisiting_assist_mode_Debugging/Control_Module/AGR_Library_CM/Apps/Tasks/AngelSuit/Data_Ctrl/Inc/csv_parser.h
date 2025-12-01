#ifndef CSV_PARSER_H
#define CSV_PARSER_H

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ioif_fatfs.h"

/* contents file headers */
#include "robot_Setting.h"
#include "robot_FSM.h"
#include "robot_DMS.h"
#include "robot_motion_map.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define CSV_LINE_BUF_SIZE 256
#define FSM_BUF_SIZE 64

// Content CSV 파일의 최대 column 개수 
// 1 row 당 ',' 로 구분되는 항목 최대 개수, 한 cell 내에도 ',' 로 구분되어 질 수 있어 여유 있게 설정
#define CONTENT_COLUMN_MAX 64
// Content CSV 파일의 한 셀의 최대 길이(byte), DMS의 Description 의 길이 때문에 여유 있게 설정
#define CONTENT_CELL_MAX 128

// 각 Content 파일의 DATA 가 시작되는 index
#define ROBOT_SETTING_START_IDX 3
#define FSM_START_IDX 4
#define DMS_START_IDX 3
#define MOTION_MAP_START_IDX 4

// 상수 정의
#define ROBOT_ID "robot_id"
#define FILE_VERSION "file_version"
#define RS_ID_FIXED "FIXED"

#define NOT_AVAILABLE '-'
#define CARRAGE_RETURN 0xd

#define PARSE_FAILURE -1
#define PARSE_SUCCESS 0
#define LINE_ENDING 1
#define END_OF_FILE 2
#define CONTENT_FILE_ERR	3

#define ID_ROBOT_SETTING            1
#define ID_OPERATIONAL_FSM          2
#define ID_SERVICE_FSM              3
#define ID_DATA_MANAGEMENT_SETTING  4
#define ID_MOTION_MAP               5

#define CONTENTSFILE_HEADER         1
#define CONTENTSFILE_DATA           2

typedef enum
{
	READ_CONTENTS_OK,
	READ_CONTENTS_ERROR,
} read_contents_status;


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
    ROBOT_SETTING = 1,
    OPERATION_FSM = 2,
    SERVICE_FSM = 3,
    DMS = 4,
    MOTION_MAP = 5,
} CONTENT_TYPE;

// Content CSV 파일의 공통 row
// row 0 : robot ID
// row 1 : file version
typedef enum {
    CSV_ROBOT_ID_ROW = 0,
    CSV_FILE_VERSION_ROW = 1,
} CSV_INDEX;

// Robot ID 및 파일 버전 정보 프로토콜에 의해 Content file 을 byte array 로 변환할 때 사용하는 byte array의 index
typedef enum {
    CONTENT_FILE_ID_IDX = 0,
    CONTENT_DATA_ID_IDX = 1,
    CONTENT_ROBOT_ID_IDX = 2,
    CONTENT_FILE_VERSION_IDX = 3,
    CONTENT_CRC_IDX = 60,
} CONTENT_INDEX;

// Robot setting vector 전송 프로토콜에 의해 Content file 을 byte array 로 변환할 때 사용하는 byte array의 index
typedef enum {
    RS_FILE_ID_IDX = 0,
    RS_DATA_ID_IDX = 1,
    RS_CURRENT_ROW_IDX = 2,
    RS_USAGE_IDX = 4,
    RS_DEVICE_ID_IDX = 5,
    RS_CH_IDX = 7,
    RS_NAME_IDX = 8,
    RS_CRC_IDX = 60,
} ROBOT_SETTING_IDX;

// FSM state vector 전송 프로토콜에 의해 Content file 을 byte array 로 변환할 때 사용하는 byte array의 index
typedef enum {
    FSM_FILE_ID_IDX = 0,
    FSM_DATA_ID_IDX = 1,
    FSM_CURRENT_ROW_IDX = 2,
    FSM_STATE_ID_IDX = 4,
    FSM_MOTION_SET_ID_IDX = 5,
    FSM_ACTION_LED_ID_IDX = 6,
    FSM_ACTION_AUDIO_ID_IDX = 7,
    FSM_ACTION_TABLET_MSG_ID_IDX = 8,
    FSM_ACTION_EMPTY_ID_IDX = 9,
    FSM_TABLET_MODE_ID_IDX = 10,
    FSM_TIMEOUT_IDX = 11,
    FSM_DEFAULT_TARGET_STATE_IDX = 13,
    FSM_EXIT_CONDITION_IDX = 14,
    FSM_TARGET_STATE_IDX = 34,
    FSM_CRC_IDX = 60,
} FSM_IDX;

// DMS vector 전송 프로토콜에 의해 Content file 을 byte array 로 변환할 때 사용하는 byte array의 index
typedef enum {
    DMS_FILE_ID_IDX = 0,
    DMS_DATA_ID_IDX = 1,
    DMS_CURRENT_ROW_IDX = 2,
    DMS_CHECK_FOR_MEASUREMENT_IDX = 4,
    DMS_MEASURING_DEVICE_IDX = 5,
    DMS_SAVE_TO_CM_IDX = 6,
    DMS_SEND_TO_AM_IDX = 7,
    DMS_CRC_IDX = 60,
} DMS_IDX;

// Motion Map vector 전송 프로토콜에 의해 Content file 을 byte array 로 변환할 때 사용하는 byte array의 index
typedef enum {
    MM_FILE_ID_IDX = 0,
    MM_DATA_ID_IDX = 1,
    MM_MS_ID_IDX = 2,
    MM_MAX_P_V_LENGTH_IDX = 3,
    MM_MAX_F_V_LENGTH_IDX = 4,
    MM_MAX_I_V_LENGTH_IDX = 5,
    MM_MD_ID_IDX = 6,
    MM_FF_IDX = 7,
    MM_VECTOR_IDX = 7, // P, F, I vector 전송 프로토콜에서 byte array의 vector 시작 주소(P[0], P[5], F[0], F[5], I[0], I[5] 에 해당)
    MM_PD_IDX = 8,
    MM_IC_IDX = 9,
    MM_DOB_IDX = 10,
    MM_IRC_IDX = 11,
    MM_FC_IDX = 12,
    MM_CRC_IDX = 60,
} MM_IDX;


typedef struct _contents_file_version_t{
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
}contents_file_version_t;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

// CSV 파일을 구분자로 파싱하는 함수
int ParseOneRow_CSV(IOIF_FILE_t* FileObject, uint8_t* Filename, char* dest);
// Robot ID 및 파일 버전 정보 프로토콜에 맞게 byte array를 생성해주는 함수
//int create_content_info_packet(uint8_t* buf, CONTENT_TYPE type, uint8_t robot_id, uint8_t file_version);
int create_content_info_packet(uint8_t* buf, CONTENT_TYPE type, uint8_t robot_id, contents_file_version_t file_ver);
// 각 content 파일의 vector 전송 프로토콜에 맞게 byte array를 생성해주는 함수(Motion Map 제외)
int create_content_data_packet(uint8_t *buf, char (*csv_read)[CONTENT_CELL_MAX], int row, CONTENT_TYPE type);
// Motion Map vector 전송 프로토콜에 맞게 byte array를 생성해주는 함수
int create_mm_data_packet(uint8_t* buf, char(*csv_read)[CONTENT_CELL_MAX], int row, int mm_index);
// content 파일을 읽어서, 파싱하고 프로토콜에 맞는 byte array 를 생성 및 전송하는 함수
int ReadContentFile(IOIF_FILE_t* FileObject, uint8_t* Filename, CONTENT_TYPE type);

/* file version parser 함수*/
void parse_version(const char* version_str, contents_file_version_t* filever);

#endif /* SUIT_MINICM_ENABLED */

#endif //CSV_PARSER_H
