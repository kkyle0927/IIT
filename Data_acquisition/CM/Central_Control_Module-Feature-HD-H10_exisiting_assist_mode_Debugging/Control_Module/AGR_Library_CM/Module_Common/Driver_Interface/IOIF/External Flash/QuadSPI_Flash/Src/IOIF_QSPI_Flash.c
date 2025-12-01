/*
 * IOIF_QSPI_Flash.c
 *
 *  Created on: May 22, 2024
 *      Author: Angelrobotics
 */


#include "IOIF_QSPI_Flash.h"

#ifdef IOIF_QSPI_FLASH_ENABLED


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

W25Q128JV_qSPI_IOt qSPIFlashCallbackStruct;

#ifdef _USE_DEBUG_CLI
#include "cli.h"
#endif

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

#ifdef _USE_DEBUG_CLI
static bool cliIsqSPIInit = false;
#endif


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* Origin Callback Functions based on BSP */
static uint8_t IOIF_qSpiInit(void);
static uint8_t IOIF_qSpiGetID(QSPI_Info_t* pInfo);
static uint8_t IOIF_qSpiReset(void);
static uint8_t IOIF_qSpiRead(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
static bool    IOIF_qSpiGetMemMapMode(void);
static uint8_t IOIF_qSpiWrite(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
static uint8_t IOIF_qSpiBlockErase(uint32_t BlockAddr);
static uint8_t IOIF_qSpiEraseAll(void);
static uint8_t IOIF_qSpiGetStatus(void);
static uint8_t IOIF_qSpiSetMemMap(void);
static uint8_t IOIF_qSpiGetInfo(QSPI_Info_t* pInfo);
static uint8_t IOIF_qSpiAbort(void);

#ifdef _USE_DEBUG_CLI
static void cliRun(cli_args_t *args);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_Init(void)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_OK;

	/* Callback Function Registration */
	qSPIFlashCallbackStruct.qspiInit  	   	  = IOIF_qSpiInit;
	qSPIFlashCallbackStruct.qspigetID 	   	  = IOIF_qSpiGetID;
	qSPIFlashCallbackStruct.qspiReset 	   	  = IOIF_qSpiReset;
	qSPIFlashCallbackStruct.qspiRead  	   	  = IOIF_qSpiRead;
	qSPIFlashCallbackStruct.qspiGetMemMapMode = IOIF_qSpiGetMemMapMode;
	qSPIFlashCallbackStruct.qspiWrite	   	  = IOIF_qSpiWrite;
	qSPIFlashCallbackStruct.qspiBlockErase 	  = IOIF_qSpiBlockErase;
	qSPIFlashCallbackStruct.qspiEraseAll   	  = IOIF_qSpiEraseAll;
	qSPIFlashCallbackStruct.qspiGetStatus  	  = IOIF_qSpiGetStatus;
	qSPIFlashCallbackStruct.qspiSetMemMap	  = IOIF_qSpiSetMemMap;
	qSPIFlashCallbackStruct.qspiGetInfo    	  = IOIF_qSpiGetInfo;
	qSPIFlashCallbackStruct.qspiAbort	   	  = IOIF_qSpiAbort;

	W25Q128JV_qSPI_RegisterIOCallBack(&qSPIFlashCallbackStruct);


	/* QSPI Flash Driver Init. */
	if(qSPIFlashInit() != true)
		ret = IOIF_QSPI_ERROR;

#ifdef _USE_DEBUG_CLI
    CLI_CMDAdd("qspi_flash", cliRun);// add cli cmd

    if (ret == IOIF_QSPI_OK)
    	cliIsqSPIInit = true;		// init status check
#endif

	return ret;
}


IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashGetMemMapMode() == true)
	{
		if(ReadAddr < QSPI_MEMMAPPED_ADDRESS || ReadAddr > QSPI_MEMMAPPED_ADDRESS + W25Q128JV_FLASH_SIZE)
			return ret;
	}

	if(qSPIFlashRead(ReadAddr, pData, Size) == true)
		ret = IOIF_QSPI_OK;

	return ret;
}

IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashWrite(WriteAddr, pData, Size) == true)
		ret = IOIF_QSPI_OK;

	return ret;
}

IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_BlockErase(uint32_t blockAddr)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashEraseBlock(blockAddr) == true)
		ret = IOIF_QSPI_OK;

	return ret;
}

IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_Erase(uint32_t addr, uint32_t length)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashErase(addr, length) == true)
		ret = IOIF_QSPI_OK;

	return ret;
}

IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_EraseAll(void)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashEraseAll() == true)
		ret = IOIF_QSPI_OK;

	return ret;
}

IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_SetMemMapMode(bool enabled)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashSetMemMap(enabled) == true)
		ret = IOIF_QSPI_OK;

	return ret;
}


IOIF_QSPI_ModeStatusTypeDef_t IOIF_qSpiFlash_GetMemMapMode(void)
{
	IOIF_QSPI_ModeStatusTypeDef_t ret = IOIF_QSPI_MODE_INDIRECT;

	if(qSPIFlashGetMemMapMode() == true)
		ret = IOIF_QSPI_MODE_MEMMAPPED;

	return ret;
}


IOIF_QSPI_StatusTypeDef_t IOIF_qSpiFlash_GetInfo(QSPI_Info_t* pInfo)
{
	IOIF_QSPI_StatusTypeDef_t ret = IOIF_QSPI_ERROR;

	if(qSPIFlashGetInfo(pInfo) == true)
		ret = IOIF_QSPI_OK;

	return ret;
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static uint8_t IOIF_qSpiInit(void)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Init();

	return (uint8_t)ret;
}

static uint8_t IOIF_qSpiGetID(QSPI_Info_t* pInfo)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_GetID(pInfo);

	return (uint8_t)ret;
}

static uint8_t IOIF_qSpiReset(void)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Reset();

	return (uint8_t)ret;
}

static uint8_t IOIF_qSpiRead(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Read(pData, ReadAddr, Size);

	return (uint8_t)ret;
}

static bool IOIF_qSpiGetMemMapMode(void)
{
	bool ret = false;

	if(BSP_QSPI_GetMemMapMode() == BSP_QSPI_OK)
		ret = true;

	return ret;
}

static uint8_t IOIF_qSpiWrite(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Write(pData, WriteAddr, Size);

	return (uint8_t)ret;
}


static uint8_t IOIF_qSpiBlockErase(uint32_t BlockAddr)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Erase_Block(BlockAddr);

	return (uint8_t)ret;
}


static uint8_t IOIF_qSpiEraseAll(void)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Erase_Chip();

	return (uint8_t)ret;
}


static uint8_t IOIF_qSpiGetStatus(void)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_GetStatus();

	return (uint8_t)ret;
}


static uint8_t IOIF_qSpiSetMemMap(void)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_MemoryMappedMode();

	return (uint8_t)ret;
}

static uint8_t IOIF_qSpiGetInfo(QSPI_Info_t* pInfo)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_GetInfo(pInfo);

	return (uint8_t)ret;
}


static uint8_t IOIF_qSpiAbort(void)
{
	IOIF_QSPI_StatusTypeDef_t ret;

	ret = BSP_QSPI_Abort();

	return (uint8_t)ret;
}



/**
*------------------------------------------------------------
*                      CLI FUNCTIONS
*------------------------------------------------------------
* @brief Functions are supported Commmand Line Interface (CLI).
*/

#ifdef _USB_DEBUG_CLI

static void cliRun(cli_args_t *args)
{
	volatile bool ret = false;

	/* init status check */
	if(args->argc == 1 && args->cmpStr(0, "isinit") == true)
	{
		if(cliIsqSPIInit > 0 )
			CLI_Printf("Init : QSPI Flash init success. \r\n");
		else
			CLI_Printf("Init : QSPI Flash init failure! \r\n");

		ret = true;
	}

	/* flash info */
	if(args->argc == 1 && args->cmpStr(0, "info") == true)
	{
		QSPI_Info_t* FlashInfo = {0};

		IOIF_qSpiFlash_GetInfo(FlashInfo);

		CLI_Printf("QSPI Flash Base Address(Indirect)     : 0x%X \r\n", 0);
		CLI_Printf("QSPI Flash Base Address(MemoryMapped) : 0x%X \r\n", QSPI_MEMMAPPED_ADDRESS);
		CLI_Printf("QSPI Flash Mode Status                : %s \r\n", IOIF_qSpiFlash_GetMemMapMode() ? "MemoryMapped":"Indirect");
		CLI_Printf("QSPI Flash Size                       : %d Byte \r\n", FlashInfo->FlashSize);

		ret = true;
	}

	/* Set MemMap Mode on/off */
	if(args->argc == 2 && args->cmpStr(0, "memmap") == true)
	{
		if(args->cmpStr(1, "on") == true)
		{
			if(IOIF_qSpiFlash_SetMemMapMode(true) == IOIF_QSPI_OK)
			{
				CLI_Printf("MemoryMapped Mode is enabled. \r\n");
				CLI_Printf("QSPI Flash Mode Status : %s \r\n", IOIF_qSpiFlash_GetMemMapMode() ? "MemoryMapped":"Indirect");
			}
			else
			{
				CLI_Printf("Change Mode Fail! \r\n");
			}

			ret = true;
		}
		else if(args->cmpStr(1, "off") == true)
		{
			if(IOIF_qSpiFlash_SetMemMapMode(false) == IOIF_QSPI_OK)
			{
				CLI_Printf("MemoryMapped mode is disabled, Flash will be reset! \r\n");
				CLI_Printf("QSPI Flash Mode Status : %s \r\n", IOIF_qSpiFlash_GetMemMapMode() ? "MemoryMapped":"Indirect");
			}
			else
			{
				CLI_Printf("Change Mode Fail! \r\n");
			}

			ret = true;
		}
		else
			ret = false;
	}


	/* Read Function */
	if(args->argc == 3 && args->cmpStr(0, "read") == true)
	{
		uint8_t cliReadData;
		IOIF_QSPI_StatusTypeDef_t res;
		uint32_t cliAddr   = args->getData(1);
		uint32_t cliLength = args->getData(2);
		uint32_t showReadData = 0;

		CLI_Printf(" [Address]   [Data] \r\n");

		for(int idx=0; idx<cliLength; idx++)
		{
			res = IOIF_qSpiFlash_Read(&cliReadData, cliAddr + idx, 1);

			if(res == IOIF_QSPI_OK)
			{
				if((idx%16) == 0)	CLI_Printf(" 0x%08X: ", cliAddr + idx);
				//CLI_Printf("%02X", cliReadData);
				showReadData |= cliReadData << (8 * (idx % 4));
				if((idx%4) == 3) {

					CLI_Printf("0x%08X ", showReadData);
					showReadData = 0;
					//CLI_Printf(" ");
				}

				if((idx%16) == 15)  CLI_Printf(" |\r\n");
			}
			else
			{
				CLI_Printf("QSPI read Fail! Addr : 0x%08X. \r\n", cliAddr+idx);
			}
		}

		if(res == IOIF_QSPI_OK)
			CLI_Printf("\r\n\r\n QSPI Read is OK! \r\n");

		ret = true;
	}

	/* Write Function */
	if(args->argc == 3 && args->cmpStr(0, "write") == true)
	{
		uint32_t cliAddr = args->getData(1);
		uint32_t cliData = args->getData(2);

		IOIF_QSPI_StatusTypeDef_t res = IOIF_qSpiFlash_Write((uint8_t*)&cliData, cliAddr, 4);		//32bit mem. 4 byte

		if(res == IOIF_QSPI_OK)
		{
			//ret = true;
			CLI_Printf("QSPI Write ok. \r\n");
			CLI_Printf("Addr : 0x%x, Data : 0x%08x \r\n", cliAddr, cliData);
		}
		else
		{
			CLI_Printf("QSPI Write Fail!. \r\n");
		}

		ret = true;

	}

	/* Erase Function : Block Erase */
	if(args->argc == 3 && args->cmpStr(0, "erase") == true)
	{
		uint32_t cliAddr = args->getData(1);
		uint32_t clilength = args->getData(2);


		IOIF_QSPI_StatusTypeDef_t res = IOIF_qSpiFlash_Erase(cliAddr, clilength);

		if(res == IOIF_QSPI_OK)
		{
			CLI_Printf("QSPI Block Erase OK. \r\n");
			CLI_Printf("Addr : 0x%x, Length : %d \r\n", cliAddr, clilength);
		}
		else
		{
			CLI_Printf("QSPI Block Erase Fail! \r\n");
		}

		ret = true;
	}

	/* Erase Function : Chip Erase */
	if(args->argc == 2 && args->cmpStr(0, "erase") == true)
	{
		if(args->cmpStr(1, "all") == true)
		{

			CLI_Printf("QSPI Chip Erasing... (wait about 40s until full erase is done) \r\n");

			IOIF_QSPI_StatusTypeDef_t res = IOIF_qSpiFlash_EraseAll();

			if(res == IOIF_QSPI_OK)
			{
				CLI_Printf("QSPI Chip Erase is OK. \r\n");
			}
			else
			{
				CLI_Printf("QSPI Chip Erase Fail! \r\n");
			}

		}

		ret = true;
	}

	/* Memory Test */
	if(args->argc == 3 && args->cmpStr(0, "memtest"))
	{
		uint64_t tdata = 0;
		bool cliret = false;

		uint32_t cliaddr   = (uint32_t)args->getData(1);
		uint32_t clilength = (uint32_t)args->getData(2);
		clilength -= (clilength % 8);

		do
		{
			cliret = true;
			CLI_Printf("QSPI Erasing ..");
			if (IOIF_qSpiFlash_Erase(cliaddr, clilength) != IOIF_QSPI_OK)
			{
				cliret = false;
				break;
			}
			CLI_Printf("%s\r\n", cliret ? "OK" : "Fail");


			cliret = true;
			CLI_Printf("QSPI Writing ..");
			for (uint32_t i=0; i<clilength; i+=8)
			{
				tdata = ((uint64_t)i<<32) | ((uint64_t)i<<0);
				if (IOIF_qSpiFlash_Write((uint8_t *)&tdata, cliaddr + i, 8) != IOIF_QSPI_OK)
				{
					CLI_Printf("0x%X ", i);
					cliret = false;
					break;
				}
			}
			CLI_Printf("%s\r\n", cliret ? "OK" : "Fail");

			cliret = true;
			CLI_Printf("QSPI Reading & Verifying ..");
			for (uint32_t i=0; i<clilength; i+=8)
			{
				tdata = 0;
				if (IOIF_qSpiFlash_Read((uint8_t *)&tdata, cliaddr + i, 8) != IOIF_QSPI_OK)
				{
					CLI_Printf("0x%X ", i);
					cliret = false;
					break;
				}
				if (tdata != (((uint64_t)i<<32)|((uint64_t)i<<0)))
				{
					CLI_Printf("Verification Error [Addr] : 0x%X ", i);
					cliret = false;
					break;
				}
				CLI_Delay(1);
			}
			CLI_Printf("%s\r\n", cliret ? "OK" : "Fail");

//			cliret = true;
//			CLI_Printf("QSPI Erasing ..");
//			if (IOIF_qSpiFlash_Erase(cliaddr, clilength) != IOIF_QSPI_OK)
//			{
//				cliret = false;
//				break;
//			}
//			CLI_Printf("%s\r\n", cliret ? "OK" : "Fail");

		} while (0);

		CLI_Printf("%s\r\n", cliret ? "OK" : "Fail");
		ret = true;
	}


	if(ret == false)		//help
	{
		CLI_Printf(" - Usage : \r\n");
		CLI_Printf("   qspi_flash isinit \r\n");
		CLI_Printf("   qspi_flash info \r\n");
		CLI_Printf("   qspi_flash read  [addr] [length] \r\n");
		CLI_Printf("   qspi_flash write [addr] [data(0-4294967295)] \r\n");
		CLI_Printf("   qspi_flash erase [addr] [length] \r\n");
		CLI_Printf("   qspi_flash erase all \r\n");
		CLI_Printf("   qspi_flash memmap [on/off] \r\n");
		CLI_Printf("   qspi_flash memtest [addr] [length] \r\n");
		CLI_Printf(" - Example : qspi_flash write 0x0 123456 \r\n");
	}
}

#endif

#endif /* IOIF_QSPI_FLASH_ENABLED */
