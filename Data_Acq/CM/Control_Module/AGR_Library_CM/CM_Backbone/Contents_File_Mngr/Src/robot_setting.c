
#include "robot_setting.h"


/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

RobotSettingFileInfo RS_File;
RobotSettingFileInfo RS_File_Read;

RobotSettingFile RS_File1 =
{
		{1, -1, -1},
		{1, 1, -1},
		{1, 2, 1},
		{1, 3, 2},
		{1, 4, 1},
		{1, 5, 2},
		{1, 6, 1},
		{1, 7, 2},
		{1, 8, 1},
		{1, 9, 2},
		{1, 10, 1},
		{1, 11, 2},
		{1, 12, 1},
		{1, 13, 2},
		{0, 14, 0},
		{0, 15, 2},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
};

RobotSettingFileInfo RS_File_AS;

RobotSettingFile RS_File_AS_Test =
{
		{0, 0, 0},
		{1, 1, 1},
		{0, 2, 0},
		{0, 3, 0},
		{0, 4, 0},
		{0, 5, 0},
		{1, 6, 1},
		{1, 7, 1},
		{1, 8, 1},
		{1, 9, 1},
		{0, 10, 0},
		{0, 11, 0},
		{1, 12, 1},
		{1, 13, 1},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
};


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */




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
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/************************* Begin(2023-12-01) *************************/
void Make_RobotSetting_Examples(void)
{
	RS_File.robot_id = 10;
	RS_File.file_version_major = 20;
	for (int i = 0; i < MAX_N_DEV; i++) {
		RS_File.vec[i].usage = i+5;
		RS_File.vec[i].FDCAN_ID = i+4;
		RS_File.vec[i].FDCAN_CH = i+2;
	}
}

void Save_RobotSetting(void)
{
	uint32_t writeAddr = 0;

	IOIF_EraseFlash(IOIF_FLASH_START_RS_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_START_RS_ADDR;

	/* (1) Save ID & Version */
	uint32_t memArr[4] = {0};
	memcpy(&memArr[0], &RS_File.robot_id,     sizeof(RS_File.robot_id));
	memcpy(&memArr[1], &RS_File.file_version_major, sizeof(RS_File.file_version_major));
	memcpy(&memArr[2], &RS_File.file_version_minor, sizeof(RS_File.file_version_minor));
	memcpy(&memArr[3], &RS_File.file_version_patch, sizeof(RS_File.file_version_patch));

	IOIF_WriteFlash(writeAddr, memArr);
	writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

	for (int i = 0; i < MAX_N_DEV; ++i) {

		uint32_t memArr1[3] = {0};
		memcpy(&memArr1[0], &RS_File.vec[i].usage,              sizeof(RS_File.vec[i].usage));
		memcpy(&memArr1[1], &RS_File.vec[i].FDCAN_CH,           sizeof(RS_File.vec[i].FDCAN_CH));
		memcpy(&memArr1[2], &RS_File.vec[i].FDCAN_ID,           sizeof(RS_File.vec[i].FDCAN_ID));

		IOIF_WriteFlash(writeAddr, memArr1);
		writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

		uint8_t memArr2[MAX_L_NAME] = {0};
		for (int j = 0; j < MAX_L_NAME; ++j)
		{
			memcpy(&memArr2[j], &RS_File.vec[i].name[j], sizeof(RS_File.vec[i].name[j]));
		}
		IOIF_WriteFlash(writeAddr, memArr2);
		writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
	}

}

void Download_RobotSetting(void) {

//	uint32_t readAddr;
//	readAddr = IOIF_FLASH_START_RS_ADDR;
//
//	IOIF_ReadFlash(readAddr,&RS_File.robot_id,                sizeof(RS_File.robot_id));            readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
//	IOIF_ReadFlash(readAddr,&RS_File.file_version,            sizeof(RS_File.file_version));        readAddr += IOIF_FLASH_READ_ADDR_SIZE_28B;
//
//
//	for (int i = 0; i < MAX_N_DEV; i++) {
//
//		IOIF_ReadFlash(readAddr,&RS_File.vec[i].usage,        sizeof(RS_File.vec[i].usage));        readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
//		IOIF_ReadFlash(readAddr,&RS_File.vec[i].FDCAN_CH,     sizeof(RS_File.vec[i].FDCAN_CH));     readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
//		IOIF_ReadFlash(readAddr,&RS_File.vec[i].FDCAN_ID,     sizeof(RS_File.vec[i].FDCAN_ID));     readAddr += IOIF_FLASH_READ_ADDR_SIZE_24B;
//		for (int j = 0; j < MAX_L_NAME; j++) {
//			IOIF_ReadFlash(readAddr, &RS_File.vec[i].name[j], sizeof(RS_File.vec[i].name[j]));      readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
//		}
//		readAddr += (IOIF_FLASH_READ_ADDR_SIZE_32B - MAX_L_NAME);
//	}

	uint8_t drv_idx = 0;
	for (int i = DEV_IDX_MD1; i < DEV_IDX_MD16; i++) {
		if (RS_File.vec[i].usage == 1) {
			RS_File.MD_setting[drv_idx].usage = 1;
			RS_File.MD_setting[drv_idx].FDCAN_CH = RS_File.vec[i].FDCAN_CH;
			RS_File.MD_setting[drv_idx].FDCAN_ID = RS_File.vec[i].FDCAN_ID;
		} else {
			RS_File.MD_setting[drv_idx].usage = 0;
			RS_File.MD_setting[drv_idx].FDCAN_CH = 0;
			RS_File.MD_setting[drv_idx].FDCAN_ID = 0;
		}
		drv_idx++;
	}
}

int Check_RobotSetting_Save(void)
{
	int err = 0;

	if (RS_File.file_version_major != RS_File_Read.file_version_major) err++;
	if (RS_File.file_version_minor != RS_File_Read.file_version_minor) err++;
	if (RS_File.file_version_patch != RS_File_Read.file_version_patch) err++;
	if (RS_File.robot_id     != RS_File_Read.robot_id)     err++;

	for (int i = 0; i < MAX_N_DEV; i++)
	{
		RobotSettingVector in  = RS_File.vec[i];
		RobotSettingVector out = RS_File_Read.vec[i];

		if (in.FDCAN_CH != out.FDCAN_CH) err++;
		if (in.FDCAN_ID != out.FDCAN_ID) err++;
		//if (in.name     != out.name)     err++;
		if (in.usage    != out.usage)    err++;

	}
	return err;


}
/************************* End (2023-12-01) *************************/


void Download_Test_RobotSetting(RobotSettingFile RS_Testfile)
{
	for (int i = 0; i < MAX_N_DEV; i++)
	{
		uint8_t t_usage = RS_Testfile[i][0];
	    if ((t_usage != 0) && (t_usage != 1)) {
	    	t_usage = 0;
	    }
	    RS_File.vec[i].usage = t_usage;
		if (t_usage == 1) {
			RS_File.vec[i].FDCAN_ID = RS_Testfile[i][1];
			RS_File.vec[i].FDCAN_CH = RS_Testfile[i][2];
		} else {
			RS_File.vec[i].FDCAN_ID = 0;
			RS_File.vec[i].FDCAN_CH = 0;
		}
	}

	// uint8_t drv_idx = 0;
	uint8_t drv_idx = 2;
	for (int i = DEV_IDX_MD1; i < DEV_IDX_MD16; i++) {
		if (RS_File.vec[i].usage == 1) {
			RS_File.MD_setting[drv_idx].usage = 1;
			RS_File.MD_setting[drv_idx].FDCAN_CH = RS_File.vec[i].FDCAN_CH;
			RS_File.MD_setting[drv_idx].FDCAN_ID = RS_File.vec[i].FDCAN_ID;
		} else {
			RS_File.MD_setting[drv_idx].usage = 0;
			RS_File.MD_setting[drv_idx].FDCAN_CH = 0;
			RS_File.MD_setting[drv_idx].FDCAN_ID = 0;
		}
		drv_idx++;
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
