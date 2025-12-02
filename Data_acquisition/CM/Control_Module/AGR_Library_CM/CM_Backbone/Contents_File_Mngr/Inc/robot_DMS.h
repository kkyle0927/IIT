#ifndef CM_BACKBONE_CCONTENTS_FILE_MNGR_DMS_H_
#define CM_BACKBONE_CCONTENTS_FILE_MNGR_DMS_H_

#include "module.h"

#include <string.h>
#include <stdint.h>

#include "device_id.h"

#include "robot_setting.h"
#include "ioif_flash_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MAX_N_CM_IF_DATA 1536  //1536 = 256 x 6
#define MAX_N_DEV_RX_DATA 10
#define MAX_N_USB_SAVE_DATA 30
#define MAX_N_SD_SAVE_DATA  30
#define MAX_N_AM_SEND_DATA  30
#define NUMEL_DMS_VECTOR 4


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _DMSVector {
	uint8_t Enable;
	uint8_t DeviceID;
	uint8_t CM_Save_Opt;
	uint8_t AM_Send_Opt;
} DMSVector;

typedef struct _DMSFileInfo {
	uint8_t     robot_id;
	uint8_t     file_version_major;
	uint8_t		file_version_minor;
	uint8_t		file_version_patch;

	DMSVector   vec[MAX_N_CM_IF_DATA];
} DMSFileInfo;

// (Devices) --> (CM) Data List
typedef struct _oneDEV2CM_DataList {
	uint16_t DataID[MAX_N_DEV_RX_DATA];
	uint8_t  Length;
} oneDEV2CM_DataList;

typedef struct _DEV2CM_DataList {
	oneDEV2CM_DataList FDCAN_Dev[MAX_N_CANFD_DEV];
} DEV2CM_DataList;

// (CM) --> (USB Stick) Data List
typedef struct _CM2USB_DataList {
	uint16_t DataID[MAX_N_USB_SAVE_DATA];
	uint8_t  Length;

} CM2USB_DataList;

// (CM) --> (SDcard) Data List
typedef struct _CM2SD_DataList {
	uint16_t DataID[MAX_N_SD_SAVE_DATA];
	uint8_t  Length;

} CM2SD_DataList;

// (CM) --> (AM) Data List
typedef struct _CM2AM_DataList {
	uint16_t DataID[MAX_N_AM_SEND_DATA];
	uint8_t  Length;

} CM2AM_DataList;

typedef uint8_t DMSFile[MAX_N_CM_IF_DATA][NUMEL_DMS_VECTOR];


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern DMSFileInfo DMS_File;
extern DMSFile     DMSFile1;

extern DEV2CM_DataList DEV2CM_PDODataList;
extern CM2AM_DataList  CM2AM_PDODataList;
extern CM2USB_DataList CM2USB_PDODataList;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Make_DMS_Examples(void);
void Save_DMS(void);
void Download_DMS(void);
int Check_DMS_Save(void);

void Download_Test_DMS(DMSFile DMS_file);
void Make_Overall_PDODataList(void);


#endif /* CM_BACKBONE_CCONTENTS_FILE_MNGR_DMS_H_ */
