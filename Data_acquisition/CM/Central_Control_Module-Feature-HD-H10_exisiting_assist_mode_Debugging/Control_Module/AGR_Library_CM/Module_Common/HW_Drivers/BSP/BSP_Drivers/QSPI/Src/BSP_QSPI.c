/**
 * @file bsp_qspi.c
 * @date Created on: May 21, 2024
 * @author AngelRobotics HW Team
 * @brief Board Support Package for Quad SPI functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#include "bsp_qspi.h"

/** @defgroup FLASH FLASH
  * @brief FLASH HAL BSP module driver
  * @{
  */

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

//QSPI_HandleTypeDef QSPIHandle;
extern QSPI_HandleTypeDef hqspi;			// core/quadspi.h
//extern MDMA_HandleTypeDef hmdma_quadspi_fifo_th;


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

static BSP_QSPI_StatusTypeDef_t BSP_QSPI_ResetMemory          (QSPI_HandleTypeDef *hqspi);
static BSP_QSPI_StatusTypeDef_t BSP_QSPI_DummyCyclesCfg       (QSPI_HandleTypeDef *hqspi);
static BSP_QSPI_StatusTypeDef_t BSP_QSPI_WriteEnable          (QSPI_HandleTypeDef *hqspi);
static BSP_QSPI_StatusTypeDef_t BSP_QSPI_AutoPollingMemReady  (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
static BSP_QSPI_StatusTypeDef_t BSP_QSPI_ReadStatus			  (QSPI_HandleTypeDef *hqspi, uint8_t cmd, uint8_t *p_data);
static BSP_QSPI_StatusTypeDef_t BSP_QSPI_WriteStatus		  (QSPI_HandleTypeDef *hqspi, uint8_t cmd, uint8_t data);


\
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

BSP_QSPI_StatusTypeDef_t BSP_QSPI_Init(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	/* Copy to MX_Init_QSPI code in this part */

	  /* Call the DeInit function to reset the driver */
//	  if (HAL_QSPI_DeInit(&hqspi) != HAL_OK)
//	  {
//	    return QSPI_ERROR;
//	  }

//	  /* System level initialization */
//	  BSP_QSPI_MspInit(&hqspi, NULL);

//	  hqspi.Instance = QUADSPI;
//	  hqspi.Init.ClockPrescaler = 0;
//	  hqspi.Init.FifoThreshold = 4;
//	  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
//	  hqspi.Init.FlashSize = 24;
//	  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
//	  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
//	  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
//	  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
//	  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
//	  {
//	    return BSP_QSPI_ERROR;
//	  }


	/////////////////////////////////////////////
	/* QSPI memory reset */
	if (BSP_QSPI_ResetMemory(&hqspi) != BSP_QSPI_OK)
	{
		return BSP_QSPI_NOT_SUPPORTED;
	}

	/* Configuration of the dummy cycles on QSPI memory side */
	if (BSP_QSPI_DummyCyclesCfg(&hqspi) != BSP_QSPI_OK)
	{
		return BSP_QSPI_NOT_SUPPORTED;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_DeInit(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

//	/* Call the DeInit function to reset the driver */
//	if (HAL_QSPI_DeInit(&hqspi) != HAL_OK)
//	{
//		return BSP_QSPI_ERROR;
//	}
//
//	/* System level De-initialization */
//	BSP_QSPI_MspDeInit(&hqspi, NULL);

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};

	/* Initialize the read command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = QUAD_INOUT_FAST_READ_CMD;			 	// W25Q128JV Quad Fast Read CMD
	s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	s_command.Address           = ReadAddr;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	s_command.AlternateBytesSize= QSPI_ALTERNATE_BYTES_8_BITS;
	s_command.AlternateBytes    = 0;
	s_command.DataMode          = QSPI_DATA_4_LINES;
	s_command.DummyCycles       = W25Q128JV_DUMMY_CYCLES_READ_QUAD; 	// W25Q128JV Read Dummy Cycle : 4
	s_command.NbData            = Size;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Set S# timing for Read command: Min 20ns for W25Q128JV/FV memory */
	MODIFY_REG(hqspi.Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_2_CYCLE);


	/* Reception of the data */
	if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}
	/* Restore S# timing for nonRead commands */
	MODIFY_REG(hqspi.Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_5_CYCLE);

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;


	QSPI_CommandTypeDef s_command = {0};
	uint32_t end_addr, current_size, current_addr;

	/* Calculation of the size between the write address and the end of the page */
	current_addr = 0;

	/* Calculation of the size between the write address and the end of the page */
	current_size = W25Q128JV_PAGE_SIZE - (WriteAddr % W25Q128JV_PAGE_SIZE);

	/* Check if the size of the data is less than the remaining place in the page */
	if (current_size > Size)
	{
		current_size = Size;
	}

	/* Initialize the adress variables */
	  current_addr = WriteAddr;
	  end_addr = WriteAddr + Size;

	  /* Initialize the program command */
	  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	  //s_command.Instruction       = EXT_QUAD_IN_FAST_PROG_CMD;
	  s_command.Instruction       = QUAD_IN_FAST_PROG_CMD;				// W25Q128JV QUAD in Fast Program : 0x32
	  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
	  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	  s_command.DataMode          = QSPI_DATA_4_LINES;
	  s_command.DummyCycles       = 0;
	  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	  /* Perform the write page by page */
	  do
	  {
		  s_command.Address = current_addr;
		  s_command.NbData  = current_size;

		  /* Enable write operations */
		  if (BSP_QSPI_WriteEnable(&hqspi) != BSP_QSPI_OK)
		  {
			  return BSP_QSPI_ERROR;
		  }

		  /* Configure the command */
		  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		  {
			  return BSP_QSPI_ERROR;
		  }

		  /* Transmission of the data */
		  if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		  {
			  return BSP_QSPI_ERROR;
		  }

		  /* Configure automatic polling mode to wait for end of program */
		  if (BSP_QSPI_AutoPollingMemReady(&hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != BSP_QSPI_OK)
		  {
			  return BSP_QSPI_ERROR;
		  }

		  /* Update the address and size variables for next page programming */
		  current_addr += current_size;
		  pData += current_size;
		  current_size = ((current_addr + W25Q128JV_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q128JV_PAGE_SIZE;
	  } while (current_addr < end_addr);

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Erase_Block(uint32_t BlockAddress)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command ={0};

	/* Initialize the erase command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = BLOCK_ERASE_64KB;				//W25Q128JV Blcok Erase CMD : 0xd8
	s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	s_command.Address           = BlockAddress;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_NONE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Enable write operations */
	if (BSP_QSPI_WriteEnable(&hqspi) != BSP_QSPI_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Send the command */
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Configure automatic polling mode to wait for end of erase */
	if (BSP_QSPI_AutoPollingMemReady(&hqspi, W25Q128JV_BLOCK_ERASE_MAX_TIME) != BSP_QSPI_OK)		// Sector Erase Max Time : 3000
	{
		return BSP_QSPI_ERROR;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Erase_Sector(uint32_t SectorAddress)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = SECTOR_ERASE_4KB;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = SectorAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (BSP_QSPI_WriteEnable(&hqspi) != BSP_QSPI_OK)
  {
    return BSP_QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return BSP_QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (BSP_QSPI_AutoPollingMemReady(&hqspi, W25Q128JV_SECTOR_ERASE_MAX_TIME) != BSP_QSPI_OK)
  {
    return BSP_QSPI_ERROR;
  }

  return BSP_QSPI_OK;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Erase_Chip(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};

	/* Initialize the erase command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = CHIP_ERASE;							// W25Q128JV bulk erase cmd : 0xC7
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_NONE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Enable write operations */
	if (BSP_QSPI_WriteEnable(&hqspi) != BSP_QSPI_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Send the command */
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Configure automatic polling mode to wait for end of erase */
	if (BSP_QSPI_AutoPollingMemReady(&hqspi, W25Q128JV_CHIP_ERASE_MAX_TIME) != BSP_QSPI_OK)
	{
		return BSP_QSPI_ERROR;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetStatus(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};
	uint8_t reg;

	/* Initialize the read flag status register command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = READ_FLAG_STATUS_REG_CMD;			// W25Q128JV Read Flag Reg. : 0x70
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Reception of the data */
	if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Check the value of the register */
	if ((reg & (W25Q128JV_FSR_PRERR | W25Q128JV_FSR_VPPERR | W25Q128JV_FSR_PGERR | W25Q128JV_FSR_ERERR)) != 0)
	{
		return BSP_QSPI_ERROR;
	}
	else if ((reg & (W25Q128JV_FSR_PGSUS | W25Q128JV_FSR_ERSUS)) != 0)
	{
		return BSP_QSPI_SUSPENDED;
	}
	else if ((reg & W25Q128JV_FSR_READY) != 0)
	{
		return BSP_QSPI_OK;
	}
	else
	{
		return BSP_QSPI_BUSY;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetID(QSPI_Info_t* pInfo)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};

	/* Initialize the read flag status register command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = READ_JEDEC_ID;					// W25Q128JV Read JEDEC ID. : 0x9F
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = QSPI_FLASH_DEVICE_ID_IDX;			// Send Data (Flash Device ID Index no.) : 10byte
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Reception of the data */
	if (HAL_QSPI_Receive(&hqspi, pInfo->device_id, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetInfo(QSPI_Info_t* pInfo)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	/* Configure the structure with the memory configuration */
	pInfo->FlashSize          = W25Q128JV_FLASH_SIZE;
	pInfo->EraseSectorSize    = W25Q128JV_SECTOR_SIZE;
	pInfo->EraseSectorsNumber = (W25Q128JV_FLASH_SIZE/W25Q128JV_SECTOR_SIZE);
	pInfo->ProgPageSize       = W25Q128JV_PAGE_SIZE;
	pInfo->ProgPagesNumber    = (W25Q128JV_FLASH_SIZE/W25Q128JV_PAGE_SIZE);

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_MemoryMappedMode(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef      s_command = {0};
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

	/* Configure the command for the read instruction */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = QUAD_INOUT_FAST_READ_CMD;					// W25Q128JV QUAD Fast Read CMD
	s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	s_command.AlternateBytesSize= QSPI_ALTERNATE_BYTES_8_BITS;
	s_command.AlternateBytes    = (1<<5);
	s_command.DataMode          = QSPI_DATA_4_LINES;
	s_command.DummyCycles       = W25Q128JV_DUMMY_CYCLES_READ_QUAD;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	//s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	s_command.SIOOMode          = QSPI_SIOO_INST_ONLY_FIRST_CMD;

//	/* Configure the memory mapped mode */
//	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_ENABLE;
//	s_mem_mapped_cfg.TimeOutPeriod     = 1;

	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

	if (HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Reset(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	if (BSP_QSPI_ResetMemory(&hqspi) != BSP_QSPI_OK)
	{
		return BSP_QSPI_NOT_SUPPORTED;
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetMemMapMode(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_ERROR;

	if (HAL_QSPI_GetState(&hqspi) == HAL_QSPI_STATE_BUSY_MEM_MAPPED)
		ret = BSP_QSPI_OK;

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_ConfigQuadMode(void)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_ERROR;

	uint8_t reg = 0;

	if (BSP_QSPI_ReadStatus(&hqspi, READ_STATUS_REG2, &reg) != BSP_QSPI_OK)
	{
		return BSP_QSPI_ERROR;
	}

	// QUAD Mode Enabled,
	if ((reg & (1<<1)) == 0x00)
	{
		reg |= (1<<1);
		if (BSP_QSPI_WriteStatus(&hqspi, WRITE_STATUS_REG2, reg) != BSP_QSPI_OK)
		{
			return BSP_QSPI_ERROR;
		}
	}

	return ret;
}


BSP_QSPI_StatusTypeDef_t BSP_QSPI_Abort(void)
{

	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	if(HAL_QSPI_Abort(&hqspi) != HAL_OK)
		ret = BSP_QSPI_ERROR;

	return ret;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static BSP_QSPI_StatusTypeDef_t BSP_QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};

	/* Initialize the reset enable command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = ENABLE_RESET;						// W25Q128JV Enable Reset CMD : 0x66
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_NONE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Send the reset memory command */
	s_command.Instruction = RESET_DEVICE;						 // W25Q128JV Reset Device : 0x99
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Configure automatic polling mode to wait the memory is ready */
	if (BSP_QSPI_AutoPollingMemReady(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != BSP_QSPI_OK)
	{
		return BSP_QSPI_ERROR;
	}

	return ret;

}


static BSP_QSPI_StatusTypeDef_t BSP_QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};
	uint8_t reg;

	/* Initialize the read volatile configuration register command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = READ_VOL_CFG_REG_CMD;				// W25Q128JV Read Vol. Config CMD : 0x85
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Reception of the data */
	if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Enable write operations */
	if (BSP_QSPI_WriteEnable(hqspi) != BSP_QSPI_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Update volatile configuration register (with new dummy cycles) */
	s_command.Instruction = WRITE_VOL_CFG_REG_CMD;					// W25Q128JV Read Vol. Config CMD : 0x81
	MODIFY_REG(reg, W25Q128JV_VCR_NB_DUMMY, (W25Q128JV_DUMMY_CYCLES_READ_QUAD << POSITION_VAL(W25Q128JV_VCR_NB_DUMMY)));

	/* Configure the write volatile configuration register command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Transmission of the data */
	if (HAL_QSPI_Transmit(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	return ret;
}


static BSP_QSPI_StatusTypeDef_t BSP_QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef     s_command = {0};
	QSPI_AutoPollingTypeDef s_config = {0};

	/* Enable write operations */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = WRITE_ENABLE;							// W25Q128JV WE CMD : 0x06
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_NONE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Configure automatic polling mode to wait for write enabling */
	s_config.Match           = W25Q128JV_SR_WEL_BIT;			// W25Q128JV write enable bit : 0x02
	s_config.Mask            = W25Q128JV_SR_WEL_BIT;
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize = 1;
	s_config.Interval        = 0x10;
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	s_command.Instruction    = READ_STATUS_REG1;				// W25Q128JV read status reg. cmd : 0x05
	s_command.DataMode       = QSPI_DATA_1_LINE;

	if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}


	return ret;
}


static BSP_QSPI_StatusTypeDef_t BSP_QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef     s_command = {0};
	QSPI_AutoPollingTypeDef s_config = {0};

	/* Configure automatic polling mode to wait for memory ready */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = READ_STATUS_REG1;				// W25Q128JV read status reg. cmd : 0x05
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	s_config.Match           = 0;
	s_config.Mask            = W25Q128JV_SR_WIP_BIT;
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize = 1;
	s_config.Interval        = 0x10;
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, Timeout) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	return ret;
}


static BSP_QSPI_StatusTypeDef_t BSP_QSPI_ReadStatus(QSPI_HandleTypeDef *hqspi, uint8_t cmd, uint8_t *p_data)
{
	BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

	QSPI_CommandTypeDef s_command = {0};
	uint8_t reg;

	/* Initialize the read flag status register command */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = cmd;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	/* Reception of the data */
	if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return BSP_QSPI_ERROR;
	}

	*p_data = reg;

	return ret;
}


static BSP_QSPI_StatusTypeDef_t BSP_QSPI_WriteStatus(QSPI_HandleTypeDef *hqspi, uint8_t cmd, uint8_t data)
{
  BSP_QSPI_StatusTypeDef_t ret = BSP_QSPI_OK;

  QSPI_CommandTypeDef s_command = {0};

  /* Initialize the program command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = cmd;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;


  /* Enable write operations */
  if (BSP_QSPI_WriteEnable(hqspi) != BSP_QSPI_OK)
  {
    return BSP_QSPI_ERROR;
  }

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return BSP_QSPI_ERROR;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(hqspi, &data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return BSP_QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of program */
  if (BSP_QSPI_AutoPollingMemReady(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != BSP_QSPI_OK)
  {
    return BSP_QSPI_ERROR;
  }


  return ret;
}



