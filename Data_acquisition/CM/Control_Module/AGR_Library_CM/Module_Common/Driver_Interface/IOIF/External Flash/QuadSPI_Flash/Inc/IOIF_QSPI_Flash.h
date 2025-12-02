/*
 * IOIF_QSPI_Flash.h
 *
 *  Created on: May 22, 2024
 *      Author: Angelrobotics
 */

#ifndef DRIVER_INTERFACE_IOIF_EXTERNAL_FLASH_QUADSPI_FLASH_INC_IOIF_QSPI_FLASH_H_
#define DRIVER_INTERFACE_IOIF_EXTERNAL_FLASH_QUADSPI_FLASH_INC_IOIF_QSPI_FLASH_H_


#include "module.h"

#ifdef IOIF_QSPI_FLASH_ENABLED

#include "W25Q128JV.h"				// SPI, QSPI Flash Driver
#include "BSP_QSPI.h"				// QSPI BSP Driver


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


typedef enum _IOIF_QSPI_StatusTypeDef_t {
    IOIF_QSPI_OK 			= BSP_QSPI_OK,
    IOIF_QSPI_ERROR 		= BSP_QSPI_ERROR,
    IOIF_QSPI_BUSY  		= BSP_QSPI_BUSY,
    IOIF_QSPI_NOT_SUPPORTED = BSP_QSPI_NOT_SUPPORTED,
	IOIF_QSPI_SUSPENDED     = BSP_QSPI_SUSPENDED,
} IOIF_QSPI_StatusTypeDef_t;


typedef enum _IOIF_QSPI_ModeStatusTypeDef_t{
	IOIF_QSPI_MODE_INDIRECT,
	IOIF_QSPI_MODE_MEMMAPPED,
}IOIF_QSPI_ModeStatusTypeDef_t;


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */




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

/* Init. & Deinit.*/
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_Init(void);

/* Read & Write & Erase */
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_BlockErase(uint32_t blockAddr);
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_Erase(uint32_t addr, uint32_t length);
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_EraseAll(void);

/* Sub Functions*/
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_SetMemMapMode(bool enabled);
IOIF_QSPI_ModeStatusTypeDef_t IOIF_qSpiFlash_GetMemMapMode(void);
IOIF_QSPI_StatusTypeDef_t 	  IOIF_qSpiFlash_GetInfo(QSPI_Info_t* pInfo);


#endif /* IOIF_QSPI_FLASH_ENABLED */

#endif /* DRIVER_INTERFACE_IOIF_EXTERNAL_FLASH_QUADSPI_FLASH_INC_IOIF_QSPI_FLASH_H_ */
