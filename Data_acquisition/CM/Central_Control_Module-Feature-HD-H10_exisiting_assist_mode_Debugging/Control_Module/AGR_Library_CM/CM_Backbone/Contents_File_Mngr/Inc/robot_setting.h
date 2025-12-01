
#ifndef CM_BACKBONE_CCONTENTS_FILE_ROBOT_SETTING_H_
#define CM_BACKBONE_CCONTENTS_FILE_ROBOT_SETTING_H_

#include "module.h"

#include <stdint.h>
#include <string.h>

#include "device_id.h"

#include "ioif_flash_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef uint8_t RobotSettingFile[MAX_N_DEV][3];

typedef char DeviceName[MAX_L_NAME];

typedef struct _RobotSettingVector {
	uint8_t  usage;
	uint8_t  FDCAN_CH;
	uint16_t FDCAN_ID;

	DeviceName name;
} RobotSettingVector;


typedef struct _RobotSettingFileInfo {
	uint8_t     robot_id;
	uint8_t     file_version_major;
	uint8_t		file_version_minor;
	uint8_t		file_version_patch;

	RobotSettingVector vec[MAX_N_DEV];
	RobotSettingVector MD_setting[MAX_N_MD];
} RobotSettingFileInfo;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern RobotSettingFile RS_File1;
extern RobotSettingFileInfo RS_File;

extern RobotSettingFile RS_File_AS_Test;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Make_RobotSetting_Examples(void);
void Save_RobotSetting(void);
void Download_RobotSetting(void);
int Check_RobotSetting_Save(void);

void Download_Test_RobotSetting(RobotSettingFile RS_file);


#endif /* CM_BACKBONE_CCONTENTS_FILE_ROBOT_SETTING_H_ */
