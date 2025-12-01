
/**
 *-----------------------------------------------------------
 *           W25Q128JV SPI FLASH DRIVER HEADER
 *-----------------------------------------------------------
 * @file W25Q128JV.c
 * @date Created on: December 23, 2023
 * @author AngelRobotics HW Team
 * @brief Register map definitions for W25Q128JV.c.
 *
 *
 * Refer to the W25Q128JV datasheet and related documents for more information.
 *
 * @ref W25Q128JV Datasheet
 */

#include "W25Q128JV.h"

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

W25Q128JV_IOt 		W25Q128JV_SPITransReceiveCB;
W25Q128JV_qSPI_IOt  W25Q128JV_qSPICB;

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

static bool SPIFlashBusyWait(uint32_t timeout);
static bool SPIFlashGetDeviceID(uint8_t* id, uint32_t length);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool SPIFlashInit(void)
{
	bool ret = false;

	uint8_t SpiFlashID[4] = {0,};

	/* 1. Flash Reset */
	if(SPIFlashReset() != true)
		return ret;

	/* 2. Get Flash ID & Check */
	if(SPIFlashGetDeviceID(SpiFlashID, 4) == true)
	{
		if (SpiFlashID[1] == SPI_FLASH_MANUFACTURER_ID && SpiFlashID[2] == SPI_FLASH_DEVICE_ID15_8 && SpiFlashID[3] == SPI_FLASH_DEVICE_ID7_0)
			ret = true;
		else ret = false;				//JEDEC ID is not matched
	}
	else ret = false;

	return ret;
}


bool qSPIFlashInit(void)
{
	bool ret = false;
	QSPI_Info_t info = {0};

	/* QSPI Driver Init */
	if (W25Q128JV_qSPICB.qspiInit() == QSPI_FLASH_OK)
		ret = true;
	else
		return ret = false;

	/* QSPI Flash Device ID Check */
	if (W25Q128JV_qSPICB.qspigetID(&info) == QSPI_FLASH_OK)
	{
		if (info.device_id[0] == SPI_FLASH_MANUFACTURER_ID && info.device_id[1] == SPI_FLASH_DEVICE_ID15_8 && info.device_id[2] == SPI_FLASH_DEVICE_ID7_0)
			ret = true;
		else
			ret = false;
	}
	else
		ret = false;

	return ret;
}


bool SPIFlashReset(void)
{
	bool ret = true;

	uint8_t txData[4] = {0,};

	/* 1. Reset Enabled, */
	txData[0] = ENABLE_RESET;
	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 1, true) != true)
		return ret = false;

	/* 2. Execute Reset */
	txData[0] = RESET_DEVICE;
	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 1, true) != true)
		return ret = false;

	/* 3. Wait reset is stable */
	W25Q128JV_SPITransReceiveCB.spiFlashIOwait(10);				//wait 10ms

	return ret;
}


bool qSPIFlashReset(void)
{
  bool ret = false;

  if (W25Q128JV_qSPICB.qspiReset() == QSPI_FLASH_OK)
  {
	  ret = true;
  }

  return ret;
}


bool SPIFlashWriteEnable(void)
{
	bool ret = true;

	uint32_t pre_time = 0;

	uint8_t txData[4] = {0,};
	uint8_t rxData[4] = {0,};

	/* 1. Set Write Enable */
	txData[0] = WRITE_ENABLE;
	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 1, true) != true)
		return ret = false;

	/* 2. Check Read Register & Wait Write Enable Status */
	pre_time = W25Q128JV_SPITransReceiveCB.spiFlashSysGetTick();
	while(1)
	{
		txData[0] = READ_STATUS_REG1;
		ret = W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, rxData, 2, true);

		if(ret == true)
		{
			if(rxData[1] & W25Q128JV_SR_WEL_BIT) break;
		}

		if(W25Q128JV_SPITransReceiveCB.spiFlashSysGetTick() - pre_time >= W25Q128JV_WRITE_ENABLE_TIMEOUT)		 //time-out
		{
			ret = false;
			break;
		}
	}

	return ret;
}


bool SPIFlashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
	bool ret = false;

	uint8_t txData[4];

	txData[0] = READ_DATA;
	txData[1] = addr >> 16;
	txData[2] = addr >> 8;
	txData[3] = addr >> 0;

	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 4, false) == true)
		ret = W25Q128JV_SPITransReceiveCB.spiFlashReceive(p_data, length, true);			//CS control maybe modified? -> verification ok

	return ret;
}


bool qSPIFlashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{

//  assert(qSPIFlashGetMemMapMode() == false);			//* assertion function, use when not support to read/write operation memory in MemMap-mode

  if(qSPIFlashGetMemMapMode() == true)
  {
	  memcpy(p_data, (void*)addr, length);			//memory access directly in MemoryMappedMode
	  return true;
  }

  if (addr >= qSPIFlashGetLength())
  {
    return false;
  }

  if (W25Q128JV_qSPICB.qspiRead(p_data, addr, length) == QSPI_FLASH_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool qSPIFlashGetMemMapMode(void)
{
	return W25Q128JV_qSPICB.qspiGetMemMapMode();
}

uint32_t qSPIFlashGetLength(void)
{
	return W25Q128JV_FLASH_SIZE;
}



bool SPIFlashFastRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
	bool ret = false;

	uint8_t txData[5];

	txData[0] = FAST_READ;
	txData[1] = addr >> 16;
	txData[2] = addr >> 8;
	txData[3] = addr >> 0;
	txData[4] = 0;			//Dummy Byte

	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 5, false) == true)
		ret = W25Q128JV_SPITransReceiveCB.spiFlashReceive(p_data, length, true);			//CS control maybe modified? -> verification ok

	return ret;
}


bool SPIFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
	bool ret = true;

	uint8_t txData[4] = {0,};
	uint32_t end_addr, current_addr, current_size;

	/* Parameter Check */
	if(addr + length > (uint32_t)W25Q128JV_FLASH_SIZE)					 // if address + length larger than total flash size,
		return ret = false;

	current_size = W25Q128JV_PAGE_SIZE - (addr % W25Q128JV_PAGE_SIZE);   // Calculation of the size between the write address and the end of the page,

	if(current_size > length)											 // Check if the size of the data is less than the remaining place in the page,
		current_size = length;

	/* Address Init. Size */
	current_addr = addr;
	end_addr = addr + length;

	do{
		/* 1. Check Write Enable? */
		if(SPIFlashWriteEnable() == false)
			return false;

		/* 2. Write(Program) Page */
		txData[0] = PAGE_PROGRAM;
		txData[1] = addr >> 16;
		txData[2] = addr >> 8;
		txData[3] = addr >> 0;

		if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 4, false) == true)
			ret = W25Q128JV_SPITransReceiveCB.spiFlashTransfer(p_data, NULL, current_size, true);			//CS control maybe modified? -> verification ok
		else ret = false;

		ret &= SPIFlashBusyWait(W25Q128JV_PROGRAM_WAIT_TIME);

		if(ret == false)
			break;					//write process fail

		/* 3. Update the address and size variables for next page write */
		current_addr += current_size;
		p_data += current_size;
		current_size = ((current_addr + W25Q128JV_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q128JV_PAGE_SIZE;
	} while (current_addr < end_addr);

	return ret;
}


bool qSPIFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  //assert(qSPIFlashGetMemMapMode() == false);			//* assertion function, use when not support to read/write operation memory in MemMap-mode


  if(qSPIFlashGetMemMapMode() == true)
	return false;

  if (addr >= qSPIFlashGetLength())
  {
    return false;
  }


  if (W25Q128JV_qSPICB.qspiWrite(p_data, addr, length) == QSPI_FLASH_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SPIFlashErase(uint32_t addr, uint32_t length)
{
	bool ret = true;

	uint32_t block_start_addr, block_end_addr;

	/* 1. Parameter Check */
	if ((addr > W25Q128JV_FLASH_SIZE) || ((addr + length) > W25Q128JV_FLASH_SIZE))		// Total Size check
		return false;

	if (length == 0)	return false;

	/* 2. Calculating Block Address Start and End*/
	block_start_addr = addr / W25Q128JV_BLOCK_SIZE;
	block_end_addr   = (addr + length - 1) / W25Q128JV_BLOCK_SIZE;

	for (uint32_t i = block_start_addr; i <= block_end_addr; i++)
	{
		ret = SPIFlashBlockErase(W25Q128JV_BLOCK_SIZE * i);
		if (ret == false)
			break;
	}

	return ret;
}


bool SPIFlashBlockErase(uint32_t block_addr)
{
	bool ret = true;
	uint8_t txData[4];

	/* 1. Write Enable */
	if (SPIFlashWriteEnable() == false)
		return false;

	/* 2. Erase Block */
	txData[0] = BLOCK_ERASE_64KB;
	txData[1] = block_addr >> 16;
	txData[2] = block_addr >> 8;
	txData[3] = block_addr >> 0;

	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 4, true) != true)
		ret = false;

	ret &= SPIFlashBusyWait(W25Q128JV_BLOCK_ERASE_MAX_TIME);

	return ret;
}


bool qSPIFlashEraseBlock(uint32_t block_addr)
{
  //assert(qSPIFlashGetMemMapMode() == false);			//* assertion function, use when not support to read/write operation memory in MemMap-mode

  if(qSPIFlashGetMemMapMode() == true)
	return false;

  if (W25Q128JV_qSPICB.qspiBlockErase(block_addr) == QSPI_FLASH_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool qSPIFlashErase(uint32_t addr, uint32_t length)
{
	bool ret = true;

	if(qSPIFlashGetMemMapMode() == true)
		return false;

	uint32_t flash_length = W25Q128JV_FLASH_SIZE;
	uint32_t block_size   = W25Q128JV_BLOCK_SIZE;
	uint32_t block_begin, block_end;

	if ((addr > flash_length) || ((addr+length) > flash_length))
	{
		return false;
	}
	if (length == 0)
	{
		return false;
	}

	block_begin = addr / block_size;
	block_end   = (addr + length - 1) / block_size;

	for (uint32_t i=block_begin; i<=block_end; i++)
	{
		if (W25Q128JV_qSPICB.qspiBlockErase(block_size*i) == QSPI_FLASH_OK)
		{
			ret = true;
		}
		else
		{
			ret = false;
			break;
		}
	}

	return ret;
}


bool qSPIFlashEraseAll(void)
{
	if(qSPIFlashGetMemMapMode() == true)
		return false;

	if (W25Q128JV_qSPICB.qspiEraseAll() == QSPI_FLASH_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool SPIFlashSectorErase(uint32_t sector_addr)
{
	bool ret = true;
	uint8_t txData[4];

	/* 1. Write Enable */
	if (SPIFlashWriteEnable() == false)
		return false;

	/* 2. Erase Block */
	txData[0] = SECTOR_ERASE_4KB;
	txData[1] = sector_addr >> 16;
	txData[2] = sector_addr >> 8;
	txData[3] = sector_addr >> 0;

	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, NULL, 4, true) != true)
		ret = false;

	ret &= SPIFlashBusyWait(W25Q128JV_SECTOR_ERASE_MAX_TIME);

	return ret;
}


bool qSPIFlashGetStatus(void)
{
	bool ret = true;

	ret = W25Q128JV_qSPICB.qspiGetStatus();

	if (ret == QSPI_FLASH_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool qSPIFlashSetMemMap(bool onoff)
{
	bool ret = true;

	if(onoff == true)
	{
		if(qSPIFlashGetMemMapMode() == false)
		{
			if(W25Q128JV_qSPICB.qspiSetMemMap() == QSPI_FLASH_OK)
				ret = true;
			else
				ret = false;
		}
	}

	else
	{
		if(qSPIFlashGetMemMapMode() == true)
		{
			W25Q128JV_qSPICB.qspiAbort();			// Abort
			ret = qSPIFlashReset();
		}
		else
			ret = false;
	}

	return ret;
}


bool qSPIFlashGetInfo(QSPI_Info_t* pInfo)
{
	bool ret = true;

	W25Q128JV_qSPICB.qspiGetInfo(pInfo);

	return ret;
}


/* Callback Registration Function */

bool W25Q128JV_RegisterIOCallBack(W25Q128JV_IOt* spiFlashIOCallback)
{
	bool ret = true;

	if(!spiFlashIOCallback->spiFlashTransfer || !spiFlashIOCallback->spiFlashReceive || !spiFlashIOCallback->spiFlashSetWP || !spiFlashIOCallback->spiFlashIsWP || !spiFlashIOCallback->spiFlashIOwait || !spiFlashIOCallback->spiFlashSysGetTick)
		return ret = false;

	W25Q128JV_SPITransReceiveCB.spiFlashTransfer   = spiFlashIOCallback->spiFlashTransfer;
	W25Q128JV_SPITransReceiveCB.spiFlashReceive    = spiFlashIOCallback->spiFlashReceive;
	W25Q128JV_SPITransReceiveCB.spiFlashSetWP	   = spiFlashIOCallback->spiFlashSetWP;
	W25Q128JV_SPITransReceiveCB.spiFlashIsWP       = spiFlashIOCallback->spiFlashIsWP;
	W25Q128JV_SPITransReceiveCB.spiFlashIOwait	   = spiFlashIOCallback->spiFlashIOwait;
	W25Q128JV_SPITransReceiveCB.spiFlashSysGetTick = spiFlashIOCallback->spiFlashSysGetTick;

	return ret;
}


bool W25Q128JV_qSPI_RegisterIOCallBack(W25Q128JV_qSPI_IOt* qspiFlashIOCallback)
{
	bool ret = true;

	if(!qspiFlashIOCallback->qspiInit || !qspiFlashIOCallback->qspigetID || !qspiFlashIOCallback->qspiReset ||!qspiFlashIOCallback->qspiRead || !qspiFlashIOCallback->qspiGetMemMapMode || !qspiFlashIOCallback->qspiWrite || !qspiFlashIOCallback->qspiBlockErase || !qspiFlashIOCallback->qspiEraseAll || !qspiFlashIOCallback->qspiGetStatus || !qspiFlashIOCallback->qspiSetMemMap || !qspiFlashIOCallback->qspiGetInfo || !qspiFlashIOCallback->qspiAbort)
		return ret = false;

	W25Q128JV_qSPICB.qspiInit  		   = qspiFlashIOCallback->qspiInit;
	W25Q128JV_qSPICB.qspigetID 		   = qspiFlashIOCallback->qspigetID;
	W25Q128JV_qSPICB.qspiReset 		   = qspiFlashIOCallback->qspiReset;
	W25Q128JV_qSPICB.qspiRead  		   = qspiFlashIOCallback->qspiRead;
	W25Q128JV_qSPICB.qspiGetMemMapMode = qspiFlashIOCallback->qspiGetMemMapMode;
	W25Q128JV_qSPICB.qspiWrite		   = qspiFlashIOCallback->qspiWrite;
	W25Q128JV_qSPICB.qspiBlockErase    = qspiFlashIOCallback->qspiBlockErase;
	W25Q128JV_qSPICB.qspiEraseAll      = qspiFlashIOCallback->qspiEraseAll;
	W25Q128JV_qSPICB.qspiGetStatus     = qspiFlashIOCallback->qspiGetStatus;
	W25Q128JV_qSPICB.qspiSetMemMap	   = qspiFlashIOCallback->qspiSetMemMap;
	W25Q128JV_qSPICB.qspiGetInfo	   = qspiFlashIOCallback->qspiGetInfo;
	W25Q128JV_qSPICB.qspiAbort		   = qspiFlashIOCallback->qspiAbort;

	return ret;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static bool SPIFlashBusyWait(uint32_t timeout)
{
	bool ret = true;

	uint32_t pre_time = 0;

	uint8_t txData[2] = {0,};
	uint8_t rxData[2] = {0,};

	pre_time = W25Q128JV_SPITransReceiveCB.spiFlashSysGetTick();
	while(1)
	{
		if(W25Q128JV_SPITransReceiveCB.spiFlashSysGetTick() - pre_time >= timeout)
		{
			ret = false;
			break;									// time-out
		}

		/* Busy Wait Check */
		txData[0] = READ_STATUS_REG1;
		ret = W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, rxData, 2, true);

		if(ret == true)
			if((rxData[1] & W25Q128JV_SR_WIP_BIT) == 0)	break;

	}

	return ret;
}


static bool SPIFlashGetDeviceID(uint8_t* id, uint32_t length)
{
	bool ret = true;

	uint8_t txData[4] = {0,};
	uint8_t rxData[4] = {0,};

	txData[0] = READ_JEDEC_ID;		//Read Flash Info. : Manufacturer, Device ID

	if(W25Q128JV_SPITransReceiveCB.spiFlashTransfer(txData, rxData, 4, true) == true)
	{
		 if (length > 4)	length = 4;

		 for (uint32_t i=0; i<length; i++)
		 {
		      id[i] = rxData[i];
		 }
	}
	else ret = false;

	return ret;
}





