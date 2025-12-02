/*
 * ioif_fatfs_sd.c
 *
 *  Created on: Sep 19, 2023
 *      Author: Angelrobotics
 */


#include "ioif_fatfs.h"

#ifdef IOIF_FATFS_ENABLED

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

#ifdef FATFS_SD_ENABLE
extern const Diskio_drvTypeDef SD_Driver;
#endif

#ifdef FATFS_USB_ENABLE
extern const Diskio_drvTypeDef USBH_Driver;
#endif


#ifdef _USE_DEBUG_CLI
#include "cli.h"
#endif

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */


static uint64_t IOIF_FreeSpaceSize;

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

#ifdef FATFS_SD_ENABLE

IOIF_SD_Status_t IOIF_FATFS_SD_Init(IOIF_SD_t SDPortNum, uint8_t* DrivePath)
{
	BSP_SD_Status_t sd_status = BSP_MSD_ERROR;

	sd_status = BSP_InitSD((BSP_SD_t)SDPortNum);
	FATFS_LinkDriver(&SD_Driver, (char*)DrivePath);

	return (IOIF_SD_Status_t) sd_status;
}

#endif

#ifdef FATFS_USB_ENABLE
IOIF_USB_Status_t IOIF_FATFS_USB_Init(IOIF_FATFS_USB_t usbmode, uint8_t* DrivePath)
{
	IOIF_USB_Status_t status = IOIF_FATFS_USB_OK;
	bool usb_status = false;

	/* USB driver init. code */
//	if(usbmode ==  IOIF_FATFS_USB_MSC)
//	{
//		usb_status = BSP_USBInit(BSP_USBH_MSC);
//		if(usb_status != true)
//			status = IOIF_FATFS_USB_FAIL;
//
//	}
//	else		/* Todo : Will be adding CDC Mass Storage mode */
//	{
//		status = IOIF_FATFS_USB_FAIL;
//	}

	/* Linked Driver */
	FATFS_LinkDriver(&USBH_Driver, (char*)DrivePath);

	return status;
}
#endif


IOIF_fCMD_res_t IOIF_FileMount(IOIF_FATFS_t* FileSysObject, uint8_t* DrivePath)
{
	return f_mount(FileSysObject, (const TCHAR*)DrivePath, 1);
}


IOIF_fCMD_res_t IOIF_FileUnmount(uint8_t* DrivePath)
{
	return f_mount(NULL, (const TCHAR*)DrivePath, 0);
}

IOIF_fCMD_res_t IOIF_FileOpen(IOIF_FILE_t* FileObject, uint8_t* Filename)
{
	/* File open with read/write function, if file is not exist, will return failure */
	return f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING);
}

IOIF_fCMD_res_t IOIF_FileOpenReadOnly(IOIF_FILE_t* FileObject, uint8_t* Filename)
{
	/* File open with read/write function, if file is not exist, will return failure */
	return f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_OPEN_EXISTING);
}

IOIF_fCMD_res_t IOIF_FileCreate(IOIF_FILE_t* FileObject, uint8_t* Filename)
{
	/* File create with read/write function, if file is existing, it will be truncated and overwritten */
	return f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
}

IOIF_fCMD_res_t IOIF_FileCreateNew(IOIF_FILE_t* FileObject, uint8_t* Filename)
{
	/* File create with read/write function, if file is existing, will return failure */
	return f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_WRITE | FA_CREATE_NEW);
}

IOIF_fCMD_res_t IOIF_FileOpenCreate(IOIF_FILE_t* FileObject, uint8_t* Filename)
{
	/* File open with read/write function, if file is not exist, a new file will be created */
	return f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
}

IOIF_fCMD_res_t IOIF_FileOpenCreateAppend(IOIF_FILE_t* FileObject, uint8_t* Filename)
{
	/* File open with read/append write function, if file is not exist, a new file will be created */
	return f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
}


IOIF_fCMD_res_t IOIF_FileClose(IOIF_FILE_t* FileObject)
{
	/* If file system is open, must be closed after access */
	return f_close(FileObject);
}


IOIF_fCMD_res_t IOIF_fWrite(IOIF_FILE_t* FileObject, void* WriteBuff, uint32_t WriteSize, uint32_t* WriteByte)
{
	/*
		FileObject : [IN] Pointer to the file object structure
		WriteBuff  : [IN] Pointer to the data to be written
		WriteSize  : [IN] Number of bytes to write
		WriteByte  : [OUT] Pointer to the variable to return number of bytes written
	*/
	return f_write(FileObject, WriteBuff, (unsigned int)WriteSize, (unsigned int*) WriteByte);
}

IOIF_fCMD_res_t IOIF_fRead(IOIF_FILE_t* FileObject, void* ReadBuff, uint32_t ReadSize, uint32_t* ReadByte)
{
	/*
	FileObject : [IN] File object
	ReadBuff   : [OUT] Buffer to store read data
	ReadSize   : [IN] Number of bytes to read
	ReadByte   : [OUT] Number of bytes read
	*/
	return f_read(FileObject, ReadBuff, (unsigned int) ReadSize, (unsigned int*) ReadByte);
}

IOIF_fCMD_res_t IOIF_fgets(char* readStringBuff, int readStringSize, IOIF_FILE_t* FileObject)
{
	/*
		readStringBuff	: [OUT] Read buffer
		readStringSize	: [IN] Size of the read buffer
		FileObject 		: [IN] File object
	*/
    TCHAR* result = f_gets(readStringBuff, readStringSize, FileObject);

    if (result != NULL) {
        // Read operation succeeded
        return FR_OK; // Assuming IOIF_fCMD_res_t is an enum and IOIF_FCMD_RES_OK indicates success
    } else {
        // Read operation failed
        // Handle the error or return an appropriate error code
        return FR_DISK_ERR; // Assuming IOIF_FCMD_RES_ERROR indicates an error
    }
}


/* File 1-cycle write operation with file open and write then close */

IOIF_fCMD_res_t IOIF_FileWrite(IOIF_FILE_t* FileObject, uint8_t* Filename, void* WriteBuff, uint32_t WriteSize, uint32_t* WriteByte)
{
	IOIF_fCMD_res_t status;

	if (!FileObject|| !Filename) {
		return status = FR_INVALID_PARAMETER;
	}

	status = f_open(FileObject, (const TCHAR*)Filename, FA_WRITE | FA_OPEN_ALWAYS); // File 'always' open with write only

	if (status == FR_OK) {
		status = f_write(FileObject, WriteBuff, (unsigned int)WriteSize, (unsigned int*) WriteByte);
		if (status == FR_OK) {
			f_close(FileObject);
			return status = FR_OK;
		} else {
			return status;
		} 
	} else {
		return status;
	}

	return status;
}


IOIF_fCMD_res_t IOIF_FileWriteAppend(IOIF_FILE_t* FileObject, uint8_t* Filename, void* WriteBuff, uint32_t WriteSize, uint32_t* WriteByte)
{
	IOIF_fCMD_res_t status;

	if (!FileObject|| !Filename) {
		return status = FR_INVALID_PARAMETER;
	}

	status = f_open(FileObject, (const TCHAR*)Filename, FA_WRITE | FA_OPEN_APPEND); // File 'append' open with write only

	if (status == FR_OK) {
		status = f_write(FileObject, WriteBuff, (unsigned int)WriteSize, (unsigned int*) WriteByte);
		if (status == FR_OK) {
			f_close(FileObject);
			return status = FR_OK;
		} else {
			return status;
		}
	} else {
		return status;
	}

	return status;
}


/* File 1-cycle read operation with file open and read then close */

IOIF_fCMD_res_t IOIF_FileRead(IOIF_FILE_t* FileObject, uint8_t* Filename, void* ReadBuff, uint32_t ReadSize, uint32_t* ReadByte)
{
	IOIF_fCMD_res_t status;

	if (!FileObject|| !Filename)
	{
		return status = FR_INVALID_PARAMETER;
	}

	status = f_open(FileObject, (const TCHAR*)Filename, FA_READ | FA_OPEN_ALWAYS); // File 'always' open with read only

	if (status == FR_OK)
	{
		status = f_read(FileObject, ReadBuff, (unsigned int)ReadSize, (unsigned int*)ReadByte);
		if (status == FR_OK)
		{
			f_close(FileObject);
			return status = FR_OK;
		} else return status;
	} else return status;

	return status;
}

/* File Delete */
IOIF_fCMD_res_t IOIF_FileDelete(uint8_t* Filename)
{
	return f_unlink((const TCHAR*)Filename);
}

/* File Synchronization with safe save */

IOIF_fCMD_res_t IOIF_FileSync(IOIF_FILE_t* FileObject)
{
	return f_sync(FileObject);
}

uint64_t IOIF_Disk_TotalSpace(IOIF_FATFS_t* FileSysObject)
{
    uint64_t total_clusters = (uint64_t)(FileSysObject->n_fatent - 2); // 총 클러스터 수 계산
    uint64_t sectors_per_cluster = (uint64_t)FileSysObject->csize; // 클러스터당 섹터 수

    return total_clusters * sectors_per_cluster * SECTOR_SIZE; // 단위: 바이트
}

uint64_t IOIF_Disk_GetFreeSpace(uint8_t* DrivePath, IOIF_FATFS_t* FileSysObject)
{
	IOIF_fCMD_res_t status;
	status = f_getfree((const TCHAR*)DrivePath, (DWORD*)&IOIF_FreeSpaceSize, &FileSysObject);

	uint64_t free_clusters = (uint64_t)IOIF_FreeSpaceSize;
	uint64_t sectors_per_cluster = (uint64_t)FileSysObject->csize;

	if(status == FR_OK)
		return free_clusters * sectors_per_cluster * SECTOR_SIZE; // 단위: 바이트
	else
		return -1;		// return -1 means calculation error
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */



#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

IOIF_fCMD_res_t  climount_res = FR_NOT_READY, cliopen_res = FR_NOT_READY, cliread_res = FR_NOT_READY, cliwrite_res = FR_NOT_READY, clisync_res = FR_NOT_READY;

FATFS 		  cliFatfs_test __attribute__((section(".FATFS_RAMD1_data")));
IOIF_FILE_t   cliFile_t	    __attribute__((section(".FATFS_RAMD1_data")));

uint32_t cliWrite_byte, cliRead_byte;

void CLI_RunFafts(cli_args_t *args)
{
	bool ret = false;

	uint64_t disk_totalspace = 0;
	uint64_t disk_freespace = 0;

	char diskspaceStr_total[200];
	char diskspaceStr_free[200];
	char cliReadBuffer[50];
	char* filenamepStr;
	char* writepdata;

	uint32_t writeappendCount = 0;
	uint32_t writeappendFailCount = 0;
	uint32_t readCount = 0;
	uint32_t readFailCount = 0;
	uint32_t readSize = 0;

	climount_res = IOIF_FileMount(&cliFatfs_test, (uint8_t *) "0");

	/* Function : Getspace - Return disk total and free space */
	if(args->argc == 1 && args->cmpStr(0, "getspace") == true)
	{
		// 드라이브 속성과 저장공간 용량이 차이가 심할 때 _FS_NOFSINFO 값을 0->3으로 수정 후 재확인할 것
		if(climount_res == FR_OK)
		{
			disk_totalspace = IOIF_Disk_TotalSpace(&cliFatfs_test);
			disk_freespace = IOIF_Disk_GetFreeSpace((uint8_t *) "0", &cliFatfs_test);

			sprintf(diskspaceStr_total, " * Disk Total Space : %llu byte, %.2f GB * ", disk_totalspace, (float)disk_totalspace / 1024 / 1024 / 1024);
			sprintf(diskspaceStr_free,  " * Disk Free Space  : %llu byte, %.2f GB * ", disk_freespace, (float)disk_freespace / 1024 / 1024 / 1024);
			CLI_Printf("%s \r\n", diskspaceStr_total);
			CLI_Printf("%s \r\n", diskspaceStr_free);

			ret = true;
		}
		else
		{
			CLI_Printf(" * File-system is un-mounted. *  \r\n");
			ret = true;
		}
	}

	/* Function : Single Write */
	if(args->argc == 3 && args->cmpStr(0, "write") == true)
	{
		if(climount_res == FR_OK)
		{
			filenamepStr = args->getStr(1);				// filename parsing
			writepdata = args->getStr(2);				// write data parsing

			if(IOIF_FileWrite(&cliFile_t, (uint8_t*)filenamepStr, writepdata, strlen(writepdata), &cliWrite_byte) == FR_OK)
			{
				CLI_Printf(" * File Single Write Done. *  \r\n");
				ret = true;
			}
			else
			{
				CLI_Printf(" * File Write Fail! * \r\n");
				ret = true;
			}
		}
		else
		{
			CLI_Printf(" * File-system is un-mounted. *  \r\n");
			ret = true;
		}
	}

	/* Function : Write append for loop */
	if(args->argc == 3 && args->cmpStr(0, "writeappend") == true)
	{
		if(climount_res == FR_OK)
		{
			filenamepStr = args->getStr(1);			// filename parsing
			writepdata = args->getStr(2);			// write data parsing

			while(CLI_KeepLoop())
			{
				if(IOIF_FileWriteAppend(&cliFile_t, (uint8_t*)filenamepStr, writepdata, strlen(writepdata), &cliWrite_byte) == FR_OK)
				{
					writeappendCount++;
					CLI_Printf(" * %d : File Write Append Done. * \r\n", writeappendCount);
					ret = true;
				}
				else
				{
					CLI_Printf(" * File Write Fail! * \r\n");
					ret = true;
					writeappendFailCount++;
				}
				CLI_Delay(100);
			}

			if(writeappendCount > 0)
			{
				CLI_Printf("\r\n");
				CLI_Printf(" * Write Append Test Success Count : %d  * \r\n", writeappendCount);
				CLI_Printf(" * Write Append Test Fail Count    : %d  * \r\n", writeappendFailCount);

				writeappendCount = 0;
				writeappendFailCount = 0;
			}
		}
		else
		{
			CLI_Printf(" * File-system is un-mounted. *  \r\n");
			ret = true;
		}
	}

	/* Function : Read Data for loop */
	if(args->argc == 3 && args->cmpStr(0, "read") == true)
	{
		if(climount_res == FR_OK)
		{
			filenamepStr = args->getStr(1);			// filename parsing
			readSize = args->getData(2);			// number of read byte parsing

			while(CLI_KeepLoop())
			{

				if(IOIF_FileRead(&cliFile_t, (uint8_t*)filenamepStr, cliReadBuffer, readSize, &cliRead_byte) == FR_OK)
				{
					CLI_Printf(" * %d : File Read : %s *  \r\n", readCount, cliReadBuffer);
					readCount++;
					ret = true;
				}

				else
				{
					CLI_Printf(" * File Read Fail! * \r\n");
					ret = true;
					readFailCount++;
				}
				CLI_Delay(100);
			}

			if(readCount > 0)
			{
				CLI_Printf("\r\n");
				CLI_Printf(" * Read Test Success Count : %d  * \r\n", readCount);
				CLI_Printf(" * Read Test Fail Count    : %d  * \r\n", readFailCount);

				readCount = 0;
				readFailCount = 0;
			}
		}
		else
		{
			CLI_Printf(" * File-system is un-mounted. *  \r\n");
			ret = true;
		}
	}

	/* Function : Delete File for loop */
		if(args->argc == 2 && args->cmpStr(0, "delete") == true)
		{
			if(climount_res == FR_OK)
			{
				filenamepStr = args->getStr(1);

				if(IOIF_FileDelete((uint8_t*)filenamepStr) == FR_OK)
				{
					CLI_Printf("File delete is success. \r\n");
				}
				else
				{
					CLI_Printf("File delete is fail! \r\n");
				}
				ret = true;
			}
		}

		/* Test */
		if(args->argc == 1 && args->cmpStr(0, "test") == true)
		{

			bool res = true;

			uint8_t test_sd_buf[10] = {0,};
			uint8_t test_string[] = "testsdcard";

			/* Open Create File And Write Data */

			if(IOIF_FileWrite(&cliFile_t, (uint8_t*)"manu_test_file.txt", test_string, sizeof(test_string), &cliWrite_byte) == FR_OK)
			{
				/* Read Data */
				if(IOIF_FileRead(&cliFile_t, (uint8_t*)"manu_test_file.txt", test_sd_buf, sizeof(test_sd_buf), &cliRead_byte) != FR_OK)
					res = false;
			}
			else
				res = false;

			/* Check */
			if(strcmp((const char*)test_string, (const char*)test_sd_buf) == 0) // 두개의 결과가 같지 않으면
				res = false;

			/* if true, Delete and finish*/
			IOIF_FileDelete((uint8_t*)"manu_test_file.txt");

			if(res == true)
				CLI_Printf("File test done. \r\n");
			else
				CLI_Printf("File test fail! \r\n");

			ret = true;

		}


	if(ret == false)		//help
	{
		CLI_Printf(" * File System - FATFS * \r\n");
		CLI_Printf("   Read/Write Data to SD(or USB) storage for file-system(fatfs). \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   fatfs_sd(or fatfs_usb) getspace \r\n");
		CLI_Printf("   fatfs_sd(or fatfs_usb) write [filename] [write data] \r\n");
		CLI_Printf("   fatfs_sd(or fatfs_usb) writeappend [filename] [write data] \r\n");
		CLI_Printf("   fatfs_sd(or fatfs_usb) read [filename] [num. of read byte (max 50 byte)] \r\n");
		CLI_Printf("   fatfs_sd(or fatfs_usb) delete [filename] \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   fatfs_sd write test.txt test_packet_write! \r\n");
		CLI_Printf("   fatfs_sd writeappend test.txt test_packet_write! \r\n");
		CLI_Printf("   fatfs_sd read test.txt 10 \r\n");
	}
}

#endif

#endif /* IOIF_FATFS_SD_ENABLED */
