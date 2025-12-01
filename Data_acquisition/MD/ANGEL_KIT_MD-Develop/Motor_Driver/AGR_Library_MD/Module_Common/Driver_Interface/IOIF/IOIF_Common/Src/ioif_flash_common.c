

#include "ioif_flash_common.h"

/** @defgroup FLASH FLASH
  * @brief FLASH BSP module driver
  * @{
  */
#ifdef BSP_FLASH_MODULE_ENABLED

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

ContentsFile contents_file;


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

static uint32_t GetSector(uint32_t addr);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void make_contents_file(void)
{
	for (int i = 0 ; i < 8 ; i++) {
		contents_file.RAM_data[i] = i+10;
	}
}

/**
  * @brief  Initialize the Flash memory.
  * 
  * This function clears any pending flags related to the Flash memory operations.
  * It should be called before any Flash memory operations to ensure that the 
  * Flash is in a clean state.
  *
  * @retval None
  */
void IOIF_InitFlash(void)
{
    /* Clear pending flags (if any) */
    __BSP_FLASH_CLEAR_FLAG(BSP_FLASH_FLAG_EOP | BSP_FLASH_FLAG_OPERR | BSP_FLASH_FLAG_WRPERR |
                            BSP_FLASH_FLAG_PGSERR | BSP_FLASH_FLAG_WRPERR);
}

/**
  * @brief  Erase the specified flash sector or all flash sectors.
  * @param  startSector: The starting sector to erase.
  * @param  eraseAll: If true, erase all flash sectors; otherwise, erase only the specified sector.
  * @retval IOIF_FLASH_STATUS_OK if successful, IOIF_FLASH_STATUS_ERASEKO otherwise.
  */
IOIF_FLASHState_t IOIF_EraseFlash(uint32_t startSector, bool eraseAll)
{
    uint32_t userStartSector;
    uint32_t sectorError;
    BSP_FLASHEraseInitTypeDef_t eraseInit;

    /* Unlock the Flash for write/erase operations */
    uint8_t status = BSP_UnlockFlash();
    if (status != BSP_OK) {
        return IOIF_FLASH_STATUS_ERASEKO;
    }
    
    IOIF_InitFlash();

    if (eraseAll) {
        eraseInit.TypeErase = BSP_FLASH_TYPEERASE_MASSERASE;
        eraseInit.Banks = BSP_FLASH_BANK_BOTH; // 전체 Bank를 지울 경우 해당 설정 필요
    } else {
        /* Calculate the sector index */
        userStartSector = GetSector(startSector);

        eraseInit.TypeErase = BSP_FLASH_TYPEERASE_SECTORS;
        eraseInit.Sector = userStartSector;
        eraseInit.NbSectors = IOIF_NUM_SECTORS;
        eraseInit.VoltageRange = BSP_FLASH_VOLTAGE_RANGE_3;

        if (startSector < IOIF_FLASH_SECTOR_0_BANK2_ADDR) {
            eraseInit.Banks = BSP_FLASH_BANK_1;
        } else {
            eraseInit.Banks = BSP_FLASH_BANK_2;
        }
    }

    /* Perform the erase operation */
    if (BSP_EraseFlashEx(&eraseInit, &sectorError) != BSP_OK) {
        /* Error occurred while erasing the sector */
        BSP_LockFlash();
        return IOIF_FLASH_STATUS_ERASEKO;
    }

    /* If we're erasing everything, we also need to erase the second bank */
    if (eraseAll) {
        eraseInit.Banks = BSP_FLASH_BANK_2;
        if (BSP_EraseFlashEx(&eraseInit, &sectorError) != BSP_OK) {
            /* Error occurred while erasing the sector */
            BSP_LockFlash();
            return IOIF_FLASH_STATUS_ERASEKO;
        }
    }

    status = BSP_LockFlash();
    if (status != BSP_OK) {
        return IOIF_FLASH_STATUS_ERASEKO;
    }

    return IOIF_FLASH_STATUS_OK;
}

/**
  * @brief  Write a data buffer to Flash memory.
  * @note   Ensure to erase the flash sector before writing to it.
  * @param  flashAddr: Start address in Flash for writing data
  * @param  pData: Pointer to the data buffer to be written
  * @param  length: Actual length of the data to be written
  * @retval None
  */
IOIF_FLASHState_t IOIF_WriteFlash(uint32_t flashAddr, void* pData)
{
    BSP_UnlockFlash();  // Unlock the flash memory for writing
    uint8_t status = BSP_ProgramFlash(FLASH_TYPEPROGRAM_FLASHWORD, flashAddr, (uint32_t)pData);  // Write the data to flash memory
    if (status != IOIF_FLASH_STATUS_OK) {
    	return status;
    }
    BSP_LockFlash();  // Lock the flash memory after writing

    return IOIF_FLASH_STATUS_OK;
}

/**
  * @brief  Read a data buffer from Flash memory.
  * @param  flashAddr: Start address in Flash from where data is to be read
  * @param  pData: Pointer to the buffer where the read data will be stored
  * @param  length: Actual length of the data to be read
  * @retval None
  */
IOIF_FLASHState_t IOIF_ReadFlash(uint32_t flashAddr, void* pData, uint32_t length)
{
    if(length > IOIF_FLASH_BUFFER_SIZE) {
        return IOIF_FLASH_BUFFER_OVERFLOW;
    }

//    memcpy(pData, (uint32_t*)flashAddr, length);

    // Flash 메모리에서 pData 버퍼로 데이터를 복사합니다.
    memcpy(pData, (void*)flashAddr, length);

    // pData 버퍼를 uint32_t 단위로 검사합니다.
    uint32_t* uint32pData = (uint32_t*)pData;
    uint32_t numElements = length / sizeof(uint32_t); // 데이터의 개수를 계산합니다.

    for(uint32_t i = 0; i < numElements; i++) {
        if (uint32pData[i] == UNWRITTEN_VALUE) {
            // 쓰기 전 메모리 값이 발견되면, 해당 위치를 0으로 초기화합니다.
            uint32pData[i] = 0;
        }
    }

    return IOIF_FLASH_STATUS_OK;
}


IOIF_FLASHState_t IOIF_ReadFlash_FF(uint32_t flashAddr, void* pData, uint32_t length)
{
    if(length > IOIF_FLASH_BUFFER_SIZE) {
        return IOIF_FLASH_BUFFER_OVERFLOW;
    }

    memcpy(pData, (uint32_t*)flashAddr, length);
    return IOIF_FLASH_STATUS_OK;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
  * @brief  Determines the sector of a given flash address.
  * @param  addr: Flash address to determine the sector for.
  * @retval The sector corresponding to the given address.
  */
static uint32_t GetSector(uint32_t addr)
{
	if (addr >= IOIF_FLASH_SECTOR_0_BANK1_ADDR && addr < IOIF_FLASH_SECTOR_0_BANK2_ADDR) {
		return (addr - IOIF_FLASH_SECTOR_0_BANK1_ADDR) / IOIF_FLASH_SECTOR_SIZE;
	}
	else if (addr >= IOIF_FLASH_SECTOR_0_BANK2_ADDR && addr < (IOIF_FLASH_SECTOR_0_BANK2_ADDR + (IOIF_FLASH_SECTOR_COUNT * IOIF_FLASH_SECTOR_SIZE))) {
		return (addr - IOIF_FLASH_SECTOR_0_BANK2_ADDR) / IOIF_FLASH_SECTOR_SIZE;
	}

	// 잘못된 주소일 경우 기본적으로 마지막 섹터 반환
	return IOIF_FLASH_SECTOR_COUNT - 1;
}


#endif /* BSP_FLASH_MODULE_ENABLED */
