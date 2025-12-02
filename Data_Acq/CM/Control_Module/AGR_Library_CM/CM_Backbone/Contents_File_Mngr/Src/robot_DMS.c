#include "robot_DMS.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

DMSFile DMSFile1 =
{
		{1, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 1, 0},
		{1, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 1, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 1},
		{0, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_MD2, 0, 1},
		{1, DEV_IDX_MD2, 0, 0},
		{1, DEV_IDX_MD3, 1, 0},
		{1, DEV_IDX_MD4, 1, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 1, 0},
		{1, DEV_IDX_MD6, 1, 0},
		{1, DEV_IDX_MD6, 1, 0},
		{1, DEV_IDX_MD6, 0, 0},
		{1, DEV_IDX_MD8, 1, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_MD8, 0, 1},
		{1, DEV_IDX_MD6, 0, 1},
		{1, DEV_IDX_MD3, 1, 1},
		{1, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 1, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 0, 1},
		{1, DEV_IDX_MD2, 0, 0},
		{1, DEV_IDX_MD10, 0, 1},
		{1, DEV_IDX_MD11, 0, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 1, 1},
		{0, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_MD10, 1, 1},
		{1, DEV_IDX_MD11, 0, 0},
		{1, DEV_IDX_MD11, 0, 1},
		{1, DEV_IDX_MD12, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_MD12, 0, 1},
		{1, DEV_IDX_MD12, 0, 0},
		{1, DEV_IDX_MD10, 0, 1},
		{1, DEV_IDX_MD10, 0, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_MD6, 0, 0},
		{1, DEV_IDX_MD6, 0, 1},
		{1, DEV_IDX_MD10, 0, 1},
		{1, DEV_IDX_CM, 0, 1},
		{1, DEV_IDX_CM, 0, 1},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 0, 0},
		{1, DEV_IDX_CM, 0, 1},
		{1, DEV_IDX_CM, 0, 1},
		{1, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0},
		{0, DEV_IDX_CM, 0, 0}
};


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

DEV2CM_DataList DEV2CM_PDODataList;
CM2AM_DataList  CM2AM_PDODataList;
CM2USB_DataList CM2USB_PDODataList;
CM2SD_DataList CM2SD_PDODataList;

DMSFileInfo   DMS_File;
DMSFileInfo  DMS_File_Read;


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

static void Clear_Dev2CM_DataList(void);
static void Append_Dev2CM_DataList(uint8_t t_Dev_FDCAN_ID, uint16_t t_DataID);
static void Clear_CM2USB_DataList(void);
static void Append_CM2USB_DataList(uint16_t t_DataID);
static void Clear_CM2SD_DataList(void);
static void Append_CM2SD_DataList(uint16_t t_DataID);
static void Clear_CM2AM_DataList(void);
static void Append_CM2AM_DataList(uint16_t t_DataID);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/************************* (2023-12-01) *************************/
void Make_DMS_Examples(void)
{
	DMS_File.robot_id = 10;
	DMS_File.file_version_major = 20;
	for (int i = 0 ; i < MAX_N_CM_IF_DATA ; i++) {
		DMS_File.vec[i].Enable = i;
		DMS_File.vec[i].DeviceID = i+1;
		DMS_File.vec[i].CM_Save_Opt = i+2;
		DMS_File.vec[i].AM_Send_Opt = i+3;
	}
}

void Save_DMS(void) 
{
	uint32_t writeAddr = 0;

	IOIF_EraseFlash(IOIF_FLASH_START_DMS_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_START_DMS_ADDR;

	IOIF_WriteFlash(writeAddr,&DMS_File.robot_id);                    		  writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	IOIF_WriteFlash(writeAddr,&DMS_File.file_version_major);			  	  writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	IOIF_WriteFlash(writeAddr,&DMS_File.file_version_minor);			  	  writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	IOIF_WriteFlash(writeAddr,&DMS_File.file_version_patch);			  	  writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	for (int i = 0; i < MAX_N_CM_IF_DATA; i++) {
		IOIF_WriteFlash(writeAddr,&DMS_File.vec[i].Enable);           writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		IOIF_WriteFlash(writeAddr,&DMS_File.vec[i].DeviceID);         writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		IOIF_WriteFlash(writeAddr,&DMS_File.vec[i].CM_Save_Opt);      writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		IOIF_WriteFlash(writeAddr,&DMS_File.vec[i].AM_Send_Opt);      writeAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
    }
}

void Download_DMS(void)
{
	uint32_t readAddr = 0;

	readAddr = IOIF_FLASH_START_DMS_ADDR;

	IOIF_ReadFlash(readAddr,&DMS_File.robot_id    ,          sizeof(DMS_File.robot_id));     readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	IOIF_ReadFlash(readAddr,&DMS_File.file_version_major,    sizeof(DMS_File.file_version_major)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	IOIF_ReadFlash(readAddr,&DMS_File.file_version_minor,    sizeof(DMS_File.file_version_minor)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	IOIF_ReadFlash(readAddr,&DMS_File.file_version_patch,    sizeof(DMS_File.file_version_patch)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
	for (int i = 0; i < MAX_N_CM_IF_DATA; i++) {
		IOIF_ReadFlash(readAddr,&DMS_File.vec[i].Enable,      sizeof(DMS_File.vec[i].Enable));       readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		IOIF_ReadFlash(readAddr,&DMS_File.vec[i].DeviceID   , sizeof(DMS_File.vec[i].DeviceID));     readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		IOIF_ReadFlash(readAddr,&DMS_File.vec[i].CM_Save_Opt, sizeof(DMS_File.vec[i].CM_Save_Opt));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		IOIF_ReadFlash(readAddr,&DMS_File.vec[i].AM_Send_Opt, sizeof(DMS_File.vec[i].AM_Send_Opt));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
    }
}

int Check_DMS_Save(void)
{
	int err = 0;

	if (DMS_File.file_version_major != DMS_File_Read.file_version_major) err++;
	if (DMS_File.file_version_minor != DMS_File_Read.file_version_minor) err++;
	if (DMS_File.file_version_patch != DMS_File_Read.file_version_patch) err++;
	if (DMS_File.robot_id     != DMS_File_Read.robot_id)     err++;

	for (int i = 0; i < MAX_N_CM_IF_DATA; i++)
	{
		DMSVector in  = DMS_File.vec[i];
		DMSVector out = DMS_File_Read.vec[i];

		if (in.AM_Send_Opt != out.AM_Send_Opt) err++;
		if (in.CM_Save_Opt != out.CM_Save_Opt) err++;
		if (in.DeviceID    != out.DeviceID)    err++;
		if (in.Enable      != out.Enable) err++;
	}

	return err;
}


void Download_Test_DMS(DMSFile DMS_file)
{
	memset(&DMS_File, 0, sizeof(DMSFileInfo));

    for (int i = 0; i < MAX_N_CM_IF_DATA; i++)
	{
    	DMS_File.vec[i].Enable            = DMS_file[i][0];

		if (DMS_File.vec[i].Enable == 1)
		{
			DMS_File.vec[i].DeviceID          = DMS_file[i][1];
			DMS_File.vec[i].CM_Save_Opt       = DMS_file[i][2];
			DMS_File.vec[i].AM_Send_Opt       = DMS_file[i][3];
		}
		else
		{
			DMS_File.vec[i].DeviceID          = -1;
			DMS_File.vec[i].CM_Save_Opt       = -1;
			DMS_File.vec[i].AM_Send_Opt       = -1;
		}
	}
}

/*********************** Overall Function ********************************/
void Make_Overall_PDODataList(void)
{
	Clear_Dev2CM_DataList();
	Clear_CM2USB_DataList();
	Clear_CM2SD_DataList();
	Clear_CM2AM_DataList();

    for (int data_idx = 0; data_idx < MAX_N_CM_IF_DATA; data_idx++)
	{
    	uint8_t t_enable = DMS_File.vec[data_idx].Enable;
    	if (t_enable == 1)
    	{
        	// (1) DEV2CM DataList
        	uint8_t t_dev_id    = DMS_File.vec[data_idx].DeviceID;
    		uint8_t t_usage     = RS_File.vec[t_dev_id].usage;

    		if (t_usage == 1)
    		{
        		uint8_t t_fdcan_id  = RS_File.vec[t_dev_id].FDCAN_ID;

        		// if Device FDCAN ID is in proper region
        		if ((t_fdcan_id > DEV_IDX_CM) && (t_fdcan_id <= MAX_CANFD_ID))
        			Append_Dev2CM_DataList(t_fdcan_id, data_idx);
    		}
    		else
    		{
    			// error handler: Robot Setting File, DMS File Mismatch error
    		}

    		// (2) CM2USB DataList
    		uint8_t t_USB_save_opt = DMS_File.vec[data_idx].CM_Save_Opt;
    		if (t_USB_save_opt == 1)
    		{
    			Append_CM2USB_DataList(data_idx);
    		}

    		// (3) CM2AM DataList
    		uint8_t t_AM_save_opt = DMS_File.vec[data_idx].AM_Send_Opt;
    		if (t_AM_save_opt == 1)
    		{
    			Append_CM2AM_DataList(data_idx);
    		}
    	}
	}
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/*********************** Functions for Dev2CM DataList ********************************/
static void Clear_Dev2CM_DataList(void)
{
	for (int i = 0; i < MAX_N_CANFD_DEV; i++)
	{
		DEV2CM_PDODataList.FDCAN_Dev[i].Length = 0;
		for (int j = 0; j < MAX_N_DEV_RX_DATA; j++)
		{
			DEV2CM_PDODataList.FDCAN_Dev[i].DataID[j] = 0;
		}
	}
}

static void Append_Dev2CM_DataList(uint8_t t_Dev_FDCAN_ID, uint16_t t_DataID)
{
	uint8_t t_cursor = DEV2CM_PDODataList.FDCAN_Dev[t_Dev_FDCAN_ID].Length;
	if (t_cursor < MAX_N_DEV_RX_DATA)
	{
		DEV2CM_PDODataList.FDCAN_Dev[t_Dev_FDCAN_ID].DataID[t_cursor] = t_DataID;
		DEV2CM_PDODataList.FDCAN_Dev[t_Dev_FDCAN_ID].Length++;
	}
	else
	{
		//error handler: too many data
	}
}

/*********************** Functions for CM2USB DataList ********************************/
static void Clear_CM2USB_DataList(void)
{
	CM2USB_PDODataList.Length = 0;
	for (int i = 0; i < MAX_N_USB_SAVE_DATA; i++)
	{
		CM2USB_PDODataList.DataID[i] = 0;
	}
}

static void Append_CM2USB_DataList(uint16_t t_DataID)
{
	uint8_t t_cursor = CM2USB_PDODataList.Length;
	if (t_cursor < MAX_N_USB_SAVE_DATA)
	{
		CM2USB_PDODataList.DataID[t_cursor] = t_DataID;
		CM2USB_PDODataList.Length++;
	}
	else
	{
		//error handler: too many data
	}
}

/*********************** Functions for CM2SD DataList ********************************/
static void Clear_CM2SD_DataList(void)
{
	CM2SD_PDODataList.Length = 0;
	for (int i = 0; i < MAX_N_USB_SAVE_DATA; i++)
	{
		CM2SD_PDODataList.DataID[i] = 0;
	}
}

static void Append_CM2SD_DataList(uint16_t t_DataID)
{
	uint8_t t_cursor = CM2SD_PDODataList.Length;
	if (t_cursor < MAX_N_USB_SAVE_DATA)
	{
		CM2SD_PDODataList.DataID[t_cursor] = t_DataID;
		CM2SD_PDODataList.Length++;
	}
	else
	{
		//error handler: too many data
	}
}

/*********************** Functions for CM2USB DataList ********************************/
static void Clear_CM2AM_DataList(void)
{
	CM2AM_PDODataList.Length = 0;
	for (int i = 0; i < MAX_N_AM_SEND_DATA; i++)
	{
		CM2AM_PDODataList.DataID[i] = 0;
	}
}

static void Append_CM2AM_DataList(uint16_t t_DataID)
{
	uint8_t t_cursor = CM2AM_PDODataList.Length;
	if (t_cursor < MAX_N_AM_SEND_DATA)
	{
		CM2AM_PDODataList.DataID[t_cursor] = t_DataID;
		CM2AM_PDODataList.Length++;
	}
	else
	{
		//error handler: too many data
	}
}

