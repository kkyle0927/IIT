/*
**
 *-----------------------------------------------------------
 *        W25Q128JV SPI FLASH DRIVER HEADER
 *-----------------------------------------------------------
 * @file W25Q128JV.h
 * @date Created on: December 23, 2023
 * @author AngelRobotics HW Team
 * @brief Register map definitions for W25Q128JV.
 *
 *
 * Refer to the W25Q128JV datasheet and related documents for more information.
 *
 * @ref W25Q128JV Datasheet
 */

#ifndef HW_PERI_W25Q128JV_W25Q128JV_H_
#define HW_PERI_W25Q128JV_W25Q128JV_H_

#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */
/* W25Q128JV Configurations */
#define W25Q128JV_FLASH_SIZE                 (0x1000000)    /* Total Size : 128 MBits => 16MBytes */
#define W25Q128JV_BLOCK_SIZE                 0x10000        /* 1 block (=16 sectors) : 64kBytes -> 256 blocks of 64KBytes */
#define W25Q128JV_SECTOR_SIZE             	 0x1000         /* 1 sectors : 4KBytes -> 4096 sectors of 4kBytes */
#define W25Q128JV_PAGE_SIZE                  0x100          /* 1 page : 256 Bytes -> 65536 pages of 256 bytes */



/* Quad SPI Command */
#define W25Q128JV_DUMMY_CYCLES_READ           8
#define W25Q128JV_DUMMY_CYCLES_READ_QUAD      4
//#define W25Q128JV_DUMMY_CYCLES_READ_QUAD      6

/* Device ID */
#define SPI_FLASH_MANUFACTURER_ID			 0xEF		//Winbond Serial Flash
#define SPI_FLASH_DEVICE_ID15_8				 0x40		//16bit ID Upper Byte, W25QJ128IQ/JQ
#define SPI_FLASH_DEVICE_ID7_0				 0x18		//16bit ID Lower Byte, W25QJ128IQ/JQ
#define QSPI_FLASH_DEVICE_ID_IDX			 10

/* Device Control */
#define W25Q128JV_WRITE_ENABLE_TIMEOUT		 200
#define W25Q128JV_PROGRAM_WAIT_TIME			 2000
#define W25Q128JV_BLOCK_ERASE_MAX_TIME		 3000
#define W25Q128JV_SECTOR_ERASE_MAX_TIME		 1000
#define W25Q128JV_CHIP_ERASE_MAX_TIME        500000
//#define W25Q128JV_CHIP_ERASE_MAX_TIME        3000000


/* Definitions of Standard SPI Flash Instruction Sets */

/* Device controls */
#define WRITE_ENABLE				0x06
#define VOLATILE_SR_WRITE_ENABLE	0x50
#define WRITE_DISABLE				0x04
#define GLOBAL_BLOCK_LOCK			0x7E
#define GLOBAL_BLOCK_UNLOCK			0x98
#define READ_BLOCK_LOCK				0x3D
#define INDIVIDUAL_BLOCK_LOCK		0x36
#define INDIVIDUAL_BLOCK_UNLOCK		0x39
#define ERASE_PROGRAM_SUSPEND		0x75
#define ERASE_PROGRAM_RESUME		0x7A
#define POWER_DOWN					0xB9
#define ENABLE_RESET				0x66
#define RESET_DEVICE				0x99

/* Read & Write Register */
#define READ_STATUS_REG1			0x05
#define WRITE_STATUS_REG1			0x01
#define READ_STATUS_REG2			0x35
#define WRITE_STATUS_REG2			0x31
#define READ_STATUS_REG3			0x15
#define WRITE_STATUS_REG3			0x11
#define READ_SFDP_REG				0x5A
#define ERASE_SECURITY_REG			0x44
#define PROGRAM_SECURITY_REG		0x42
#define READ_SECURITY_REG			0x48
#define READ_FLAG_STATUS_REG_CMD    0x70
#define READ_VOL_CFG_REG_CMD        0x85
#define WRITE_VOL_CFG_REG_CMD       0x81

/* Read & Write(Program) & Erase */
#define RELEASE_POWERDOWN_ID		0xAB
#define MANUFACTURER_DEV_ID			0x90
#define READ_JEDEC_ID				0x9F
#define READ_UNIQUE_ID				0x4B
#define READ_DATA					0x03
#define FAST_READ					0x0B
#define PAGE_PROGRAM				0x02
#define SECTOR_ERASE_4KB			0x20
#define BLOCK_ERASE_32KB			0x52
#define BLOCK_ERASE_64KB			0xD8
#define CHIP_ERASE					0xC7
//#define CHIP_ERASE					0x60
#define DUAL_OUT_FAST_READ_CMD      0x3B		//DUAL SPI FAST READ
#define DUAL_INOUT_FAST_READ_CMD    0xBB		//DUAL SPI FAST READ
#define QUAD_OUT_FAST_READ_CMD      0x6B		//QUAD SPI FAST READ
#define QUAD_INOUT_FAST_READ_CMD    0xEB		//QUAD SPI FAST READ
#define DUAL_IN_FAST_PROG_CMD       0xA2		//DUAL SPI FAST PROGRAM
#define EXT_DUAL_IN_FAST_PROG_CMD   0xD2		//DUAL SPI FAST PROGRAM
#define QUAD_IN_FAST_PROG_CMD       0x32		//QUAD SPI FAST PROGRAM
#define EXT_QUAD_IN_FAST_PROG_CMD   0x12		//QUAD SPI FAST PROGRAM

/* Definitions of Status Register Bit */

#define W25Q128JV_SR_WIP_BIT            	  ((uint8_t)0x01)    /* Write in progress */
#define W25Q128JV_SR_WEL_BIT                  ((uint8_t)0x02)    /* Write enable latch */
#define W25Q128JV_SR_SRWREN_BIT         	  ((uint8_t)0x80)    /* Status register write enable/disable */

/* Nonvolatile Configuration Register */
#define W25Q128JV_NVCR_LOCK                   ((uint16_t)0x0001) /*!< Lock nonvolatile configuration register */
#define W25Q128JV_NVCR_DUAL                   ((uint16_t)0x0004) /*!< Dual I/O protocol */
#define W25Q128JV_NVCR_QUAB                   ((uint16_t)0x0008) /*!< Quad I/O protocol */
#define W25Q128JV_NVCR_RH                     ((uint16_t)0x0010) /*!< Reset/hold */
#define W25Q128JV_NVCR_ODS                    ((uint16_t)0x01C0) /*!< Output driver strength */
#define W25Q128JV_NVCR_XIP                    ((uint16_t)0x0E00) /*!< XIP mode at power-on reset */
#define W25Q128JV_NVCR_NB_DUMMY               ((uint16_t)0xF000) /*!< Number of dummy clock cycles */

/* Volatile Configuration Register */
#define W25Q128JV_VCR_WRAP                    ((uint8_t)0x03)    /*!< Wrap */
#define W25Q128JV_VCR_XIP                     ((uint8_t)0x08)    /*!< XIP */
#define W25Q128JV_VCR_NB_DUMMY                ((uint8_t)0xF0)    /*!< Number of dummy clock cycles */

/* Enhanced Volatile Configuration Register */
#define W25Q128JV_EVCR_ODS                    ((uint8_t)0x07)    /*!< Output driver strength */
#define W25Q128JV_EVCR_VPPA                   ((uint8_t)0x08)    /*!< Vpp accelerator */
#define W25Q128JV_EVCR_RH                     ((uint8_t)0x10)    /*!< Reset/hold */
#define W25Q128JV_EVCR_DUAL                   ((uint8_t)0x40)    /*!< Dual I/O protocol */
#define W25Q128JV_EVCR_QUAD                   ((uint8_t)0x80)    /*!< Quad I/O protocol */

/* Flag Status Register */
#define W25Q128JV_FSR_PRERR                   ((uint8_t)0x02)    /*!< Protection error */
#define W25Q128JV_FSR_PGSUS                   ((uint8_t)0x04)    /*!< Program operation suspended */
#define W25Q128JV_FSR_VPPERR                  ((uint8_t)0x08)    /*!< Invalid voltage during program or erase */
#define W25Q128JV_FSR_PGERR                   ((uint8_t)0x10)    /*!< Program error */
#define W25Q128JV_FSR_ERERR                   ((uint8_t)0x20)    /*!< Erase error */
#define W25Q128JV_FSR_ERSUS                   ((uint8_t)0x40)    /*!< Erase operation suspended */
#define W25Q128JV_FSR_READY                   ((uint8_t)0x80)    /*!< Ready or command in progress */


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */


/* QSPI Flash Info Structure */

typedef struct {
  uint32_t FlashSize;          						/*!< Size of the flash */
  uint32_t EraseSectorSize;    						/*!< Size of sectors for the erase operation */
  uint32_t EraseSectorsNumber; 						/*!< Number of sectors for the erase operation */
  uint32_t ProgPageSize;       						/*!< Size of pages for the program operation */
  uint32_t ProgPagesNumber;    						/*!< Number of pages for the program operation */

  uint8_t  device_id[QSPI_FLASH_DEVICE_ID_IDX];		// flash device id
} QSPI_Info_t;

typedef enum _QSPI_FLASH_StatusDef_t {
    QSPI_FLASH_OK       	 = 0x00,
    QSPI_FLASH_ERROR    	 = 0x01,
    QSPI_FLASH_BUSY     	 = 0x02,
    QSPI_FLASH_NOT_SUPPORTED = 0x04,
	QSPI_FLASH_SUSPENDED     = 0x08,
} QSPI_FLASH_StatusDef_t;


/* Definition of IO Callback Function Pointer */
typedef bool 		(*spiflashTransfer_fptr) (uint8_t *pTxData, uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
typedef bool 		(*spiflashReceive_fptr)  (uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
typedef bool 		(*spiflashSetWP_fptr) 	  (bool gpiostate);
typedef bool 		(*spiflashIsWP_fptr)	  (void);
typedef bool 		(*spiflashIOwait_fptr)	  (uint32_t wait_ms);
typedef uint32_t	(*spiflashGetTick_fptr)	  (void);

/* Definition of IO callback structurer */
typedef struct _W25Q128JV_IOt{
	spiflashTransfer_fptr spiFlashTransfer;
	spiflashReceive_fptr  spiFlashReceive;
	spiflashSetWP_fptr 	  spiFlashSetWP;
	spiflashIsWP_fptr     spiFlashIsWP;
	spiflashIOwait_fptr	  spiFlashIOwait;
	spiflashGetTick_fptr  spiFlashSysGetTick;
}W25Q128JV_IOt;



/* Definition of IO Callback Function Pointer (Quad SPI Mode)*/
typedef uint8_t	(*qSPIInit_fptr)  		(void);
typedef uint8_t	(*qSPIgetID_fptr) 		(QSPI_Info_t* pInfo);
typedef uint8_t (*qSPIreset_fptr) 		(void);
typedef uint8_t (*qSPIRead_fptr)  		(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
typedef bool 	(*qSPIGetMemMapMode_fptr)  (void);
typedef uint8_t (*qSPIWrite_fptr)       (uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
typedef uint8_t (*qSPIBlockErase_fptr)  (uint32_t BlockAddr);
typedef uint8_t (*qSPIEraseChip_fptr)	(void);
typedef uint8_t (*qSPIGetStatue_fptr)	(void);
typedef uint8_t (*qSPISetMemMap_fptr)	(void);
typedef uint8_t	(*qSPIGetInfo_fptr) 	(QSPI_Info_t* pInfo);
typedef uint8_t (*qSPIAbort_fptr)		(void);

/* Definition of IO callback structurer (Quad SPI) */
typedef struct _W25Q128JV_qSPI_IOt{
	qSPIInit_fptr			qspiInit;
	qSPIgetID_fptr  		qspigetID;
	qSPIreset_fptr  		qspiReset;
	qSPIRead_fptr   		qspiRead;
	qSPIGetMemMapMode_fptr  qspiGetMemMapMode;
	qSPIWrite_fptr			qspiWrite;
	qSPIBlockErase_fptr 	qspiBlockErase;
	qSPIEraseChip_fptr		qspiEraseAll;
	qSPIGetStatue_fptr		qspiGetStatus;
	qSPISetMemMap_fptr		qspiSetMemMap;
	qSPIGetInfo_fptr		qspiGetInfo;
	qSPIAbort_fptr			qspiAbort;
}W25Q128JV_qSPI_IOt;




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

/* Standard SPI Functions */
bool SPIFlashInit(void);

bool SPIFlashReset(void);
bool SPIFlashWriteEnable(void);

bool SPIFlashRead(uint32_t addr, uint8_t *p_data, uint32_t length);
bool SPIFlashFastRead(uint32_t addr, uint8_t *p_data, uint32_t length);
bool SPIFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length);
bool SPIFlashErase(uint32_t addr, uint32_t length);
bool SPIFlashBlockErase(uint32_t block_addr);
bool SPIFlashSectorErase(uint32_t sector_addr);


/* Quad SPI Functions */
bool 	 qSPIFlashInit(void);

bool 	 qSPIFlashReset(void);
bool 	 qSPIFlashGetMemMapMode(void);
uint32_t qSPIFlashGetLength(void);
bool	 qSPIFlashGetStatus(void);
bool 	 qSPIFlashSetMemMap(bool onoff);
bool	 qSPIFlashGetInfo(QSPI_Info_t* pInfo);

bool 	 qSPIFlashRead(uint32_t addr, uint8_t *p_data, uint32_t length);
bool     qSPIFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length);
bool     qSPIFlashEraseBlock(uint32_t block_addr);
bool 	 qSPIFlashErase(uint32_t addr, uint32_t length);
bool     qSPIFlashEraseAll(void);


/* Register Callback */
bool W25Q128JV_RegisterIOCallBack		(W25Q128JV_IOt* spiFlashIOCallback);				//standard SPI Callback
bool W25Q128JV_qSPI_RegisterIOCallBack	(W25Q128JV_qSPI_IOt* qspiFlashIOCallback);			//Quad SPI Callback


#endif /* HW_PERI_W25Q128JV_W25Q128JV_H_ */
