/**
 * @file bsp_sdmmc.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for SDMMC functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#ifndef SDMMC_INC_BSP_SDMMC_H_
#define SDMMC_INC_BSP_SDMMC_H_

#include "main.h"

/** @defgroup SDMMC SDMMC
 * @brief SDMMC HAL BSP module driver
 * @
 */
#ifdef HAL_SD_MODULE_ENABLED
#define BSP_SD_MODULE_ENABLED

#include "sdmmc.h"
#include "bsp_common.h"
//#include "bsp_driver_sd.h"
#include "stm32h7xx_hal.h"
#include "fatfs_platform.h"
//#include "sd_diskio.h"


 /**
  *-----------------------------------------------------------
  *              MACROS AND PREPROCESSOR DIRECTIVES
  *-----------------------------------------------------------
  * @brief Directives and macros for readability and efficiency.
  */

 /** @defgroup SDMMC_LL_Bus_Wide Bus Width
   * @{
   */
 #define BSP_SDMMC_BUS_WIDE_1B SDMMC_BUS_WIDE_1B ///< ((uint32_t)0x00000000U)
 #define BSP_SDMMC_BUS_WIDE_4B SDMMC_BUS_WIDE_4B ///< SDMMC_CLKCR_WIDBUS_0
 #define BSP_SDMMC_BUS_WIDE_8B SDMMC_BUS_WIDE_8B ///< SDMMC_CLKCR_WIDBUS_1

 #define BSP_IS_SDMMC_BUS_WIDE(WIDE) IS_SDMMC_BUS_WIDE(WIDE) /* (((WIDE) == SDMMC_BUS_WIDE_1B) ||
                                                                 ((WIDE) == SDMMC_BUS_WIDE_4B) ||
                                                                 ((WIDE) == SDMMC_BUS_WIDE_8B)) */

 /** @defgroup SDMMC_LL_Speed_Mode
   * @{
   */
 #define BSP_SDMMC_SPEED_MODE_AUTO           SDMMC_SPEED_MODE_AUTO          ///< ((uint32_t)0x00000000U)
 #define BSP_SDMMC_SPEED_MODE_DEFAULT        SDMMC_SPEED_MODE_DEFAULT       ///< ((uint32_t)0x00000001U)
 #define BSP_SDMMC_SPEED_MODE_HIGH           SDMMC_SPEED_MODE_HIGH          ///< ((uint32_t)0x00000002U)
 #define BSP_SDMMC_SPEED_MODE_ULTRA          SDMMC_SPEED_MODE_ULTRA         ///< ((uint32_t)0x00000003U)
 #define BSP_SDMMC_SPEED_MODE_ULTRA_SDR104   SDMMC_SPEED_MODE_ULTRA_SDR104  ///< SDMMC_SPEED_MODE_ULTRA
 #define BSP_SDMMC_SPEED_MODE_DDR            SDMMC_SPEED_MODE_DDR           ///< ((uint32_t)0x00000004U)
 #define BSP_SDMMC_SPEED_MODE_ULTRA_SDR50    SDMMC_SPEED_MODE_ULTRA_SDR50   ///< ((uint32_t)0x00000005U)

 #define BSP_IS_SDMMC_SPEED_MODE(MODE) IS_SDMMC_SPEED_MODE(MODE) /* (((MODE) == SDMMC_SPEED_MODE_AUTO)         ||
                                                                     ((MODE) == SDMMC_SPEED_MODE_DEFAULT)      ||
                                                                     ((MODE) == SDMMC_SPEED_MODE_HIGH)         ||
                                                                     ((MODE) == SDMMC_SPEED_MODE_ULTRA)        ||
                                                                     ((MODE) == SDMMC_SPEED_MODE_ULTRA_SDR50)  ||
                                                                     ((MODE) == SDMMC_SPEED_MODE_DDR)) */

 /** @defgroup SD_Exported_Types_Group2 SD Card State enumeration structure
   * @{
   */
 #define BSP_SD_CARD_READY           HAL_SD_CARD_READY          ///< 0x00000001U  /*!< Card state is ready                     */
 #define BSP_SD_CARD_IDENTIFICATION  HAL_SD_CARD_IDENTIFICATION ///< 0x00000002U  /*!< Card is in identification state         */
 #define BSP_SD_CARD_STANDBY         HAL_SD_CARD_STANDBY        ///< 0x00000003U  /*!< Card is in standby state                */
 #define BSP_SD_CARD_TRANSFER        HAL_SD_CARD_TRANSFER       ///< 0x00000004U  /*!< Card is in transfer state               */
 #define BSP_SD_CARD_SENDING         HAL_SD_CARD_SENDING        ///< 0x00000005U  /*!< Card is sending an operation            */
 #define BSP_SD_CARD_RECEIVING       HAL_SD_CARD_RECEIVING      ///< 0x00000006U  /*!< Card is receiving operation information */
 #define BSP_SD_CARD_PROGRAMMING     HAL_SD_CARD_PROGRAMMING    ///< 0x00000007U  /*!< Card is in programming state            */
 #define BSP_SD_CARD_DISCONNECTED    HAL_SD_CARD_DISCONNECTED   ///< 0x00000008U  /*!< Card is disconnected                    */
 #define BSP_SD_CARD_ERROR           HAL_SD_CARD_ERROR          ///< 0x000000FFU  /*!< Card response Error                     */

/* Exported types --------------------------------------------------------*/
/**
  * @brief SD Card information structure
  */
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

/* Exported constants --------------------------------------------------------*/
/**
  * @brief  SD status structure definition
  */
#define MSD_OK                        ((uint8_t)0x00)
#define MSD_ERROR                     ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)

/**
  * @brief  SD transfer state definition
  */
#define SD_TRANSFER_OK                ((uint8_t)0x00)
#define SD_TRANSFER_BUSY              ((uint8_t)0x01)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
#define SD_DATATIMEOUT           ((uint32_t)100000000)

/* USER CODE BEGIN BSP_H_CODE */
#define SD_DetectIRQHandler()             HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8)


/**
  *------------------------------------------------------------
  *                     TYPE DECLARATIONS
  *------------------------------------------------------------
  * @brief Custom data types and structures for the module.
  */

/**
  * @brief Function pointer type for SD callback functions.
  */
typedef void (*BSP_SDCBPtr_t)(void* params);

/** @defgroup SD_Exported_Types_Group1 SD State enumeration structure
  * @{
  */
typedef enum _BSP_SD_StateTypeDef_t {
    BSP_SD_STATE_RESET          = HAL_SD_STATE_RESET,         /*!< SD not yet initialized or disabled  */
    BSP_SD_STATE_READY          = HAL_SD_STATE_READY,         /*!< SD initialized and ready for use    */
    BSP_SD_STATE_TIMEOUT        = HAL_SD_STATE_TIMEOUT,       /*!< SD Timeout state                    */
    BSP_SD_STATE_BUSY           = HAL_SD_STATE_BUSY,          /*!< SD process ongoing                  */
    BSP_SD_STATE_PROGRAMMING    = HAL_SD_STATE_PROGRAMMING,   /*!< SD Programming State                */
    BSP_SD_STATE_RECEIVING      = HAL_SD_STATE_RECEIVING,     /*!< SD Receiving State                  */
    BSP_SD_STATE_TRANSFER       = HAL_SD_STATE_TRANSFER,      /*!< SD Transfer State                   */
    BSP_SD_STATE_ERROR          = HAL_SD_STATE_ERROR          /*!< SD is in error state                */
} BSP_SD_StateTypeDef_t;

typedef enum _BSP_SD_Status_t {
    BSP_MSD_OK    				  = MSD_OK,
    BSP_MSD_ERROR 				  = MSD_ERROR,
    BSP_MSD_ERROR_SD_NOT_PRESENT = MSD_ERROR_SD_NOT_PRESENT,
} BSP_SD_Status_t;

typedef enum _BSP_SD_Transfer_Status_t {
    BSP_SD_TRANSFER_OK   = SD_TRANSFER_OK,
    BSP_SD_TRANSFER_BUSY = SD_TRANSFER_BUSY,
    BSP_SD_PRESENT 	  = SD_PRESENT,
    BSP_SD_NOT_PRESENT   = SD_NOT_PRESENT,
    BSP_SD_DATATIMEOUT   = SD_DATATIMEOUT,
} BSP_SD_Transfer_Status_t;

/**
  * @brief Enumeration for BSP SD operations.
  *
  * Represents the various sd-related operations available
  * within the BSP layer, abstracting HAL functions.
  */
typedef enum _BSP_SDOP_t {
    // SD Blocking Mode
    BSP_SD_READBLOCKS,
    BSP_SD_WRITEBLOCKS,

    // SD Non-Blocking Mode with IT
    BSP_SD_READBLOCKS_IT,
    BSP_SD_WRITEBLOCKS_IT,

    // SD Non-Blocking Mode with DMA
    BSP_SD_READBLOCKS_DMA,
    BSP_SD_WRITEBLOCKS_DMA,

    // SD Erase
    BSP_SD_ERASE,
} BSP_SDOP_t;

/**
  * @brief Enumeration for BSP SD identifiers.
  * Starts from 1 to align with common STM32 naming (SD1, SD2, ...)
  */
typedef enum _BSP_SD_t {
    BSP_SD1 = 1,  ///< SD 1 Identifier
    BSP_SD2,      ///< SD 2 Identifier
    BSP_SD_COUNT
} BSP_SD_t;

/**
  * @enum BSP_SDCBType_t
  * @brief SD callback types for various sd events.
  */
typedef enum _BSP_SDCallbackType_t {
    BSP_SD_TX_CPLT_CALLBACK,
    BSP_SD_RX_CPLT_CALLBACK,
    BSP_SD_ERROR_CALLBACK,
    BSP_SD_ABORT_CALLBACK,
    BSP_SD_CALLBACK_TYPE_COUNT,
} BSP_SDCBType_t;

/**
  * @struct BSP_SDMap_t
  * @brief Maps BSP SD enumerations to their corresponding HAL SD handles.
  */
typedef struct _BSP_SDMap_t {
    BSP_SD_t sd;       ///< Enumeration of the sd
    SD_HandleTypeDef* handle;  ///< Pointer to the HAL SD handle
} BSP_SDMap_t;

/**
 * @struct BSP_SDCB_t
 * @brief Manager BSP SD Custom Callbacks and Parameters.
 */
typedef struct _BSP_SDCallbackManager_t {
    BSP_SDCBPtr_t callbacks[BSP_SD_CALLBACK_TYPE_COUNT];
    void* params[BSP_SD_CALLBACK_TYPE_COUNT];
} BSP_SDCB_t;

/** @defgroup SD_Exported_Types_Group6 SD Card Status returned by ACMD13
  * @{
  */
typedef struct _BSP_SD_CardStatusTypeDef_t {
    __IO uint8_t  DataBusWidth;           /*!< Shows the currently defined data bus width                 */
    __IO uint8_t  SecuredMode;            /*!< Card is in secured mode of operation                       */
    __IO uint16_t CardType;               /*!< Carries information about card type                        */
    __IO uint32_t ProtectedAreaSize;      /*!< Carries information about the capacity of protected area   */
    __IO uint8_t  SpeedClass;             /*!< Carries information about the speed class of the card      */
    __IO uint8_t  PerformanceMove;        /*!< Carries information about the card's performance move      */
    __IO uint8_t  AllocationUnitSize;     /*!< Carries information about the card's allocation unit size  */
    __IO uint16_t EraseSize;              /*!< Determines the number of AUs to be erased in one operation */
    __IO uint8_t  EraseTimeout;           /*!< Determines the timeout for any number of AU erase          */
    __IO uint8_t  EraseOffset;            /*!< Carries information about the erase offset                 */
    __IO uint8_t  UhsSpeedGrade;          /*!< Carries information about the speed grade of UHS card      */
    __IO uint8_t  UhsAllocationUnitSize;  /*!< Carries information about the UHS card's allocation unit size  */
    __IO uint8_t  VideoSpeedClass;        /*!< Carries information about the Video Speed Class of UHS card    */
} BSP_SD_CardStatusTypeDef_t;

/**
  * @brief  SD Card Information Structure definition
  */
typedef struct _BSP_SD_CardInfoTypeDef_t {
    uint32_t CardType;                     /*!< Specifies the card Type                         */
    uint32_t CardVersion;                  /*!< Specifies the card version                      */
    uint32_t Class;                        /*!< Specifies the class of the card class           */
    uint32_t RelCardAdd;                   /*!< Specifies the Relative Card Address             */
    uint32_t BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */
    uint32_t BlockSize;                    /*!< Specifies one block size in bytes               */
    uint32_t LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */
    uint32_t LogBlockSize;                 /*!< Specifies logical block size in bytes           */
    uint32_t CardSpeed;                    /*!< Specifies the card Speed                        */
} BSP_SD_CardInfoTypeDef_t;

/** @defgroup SD_Exported_Types_Group4 Card Specific Data: CSD Register
  * @{
  */
typedef struct _BSP_SD_CardCSDTypeDef_t {
    __IO uint8_t  CSDStruct;            /*!< CSD structure                         */
    __IO uint8_t  SysSpecVersion;       /*!< System specification version          */
    __IO uint8_t  Reserved1;            /*!< Reserved                              */
    __IO uint8_t  TAAC;                 /*!< Data read access time 1               */
    __IO uint8_t  NSAC;                 /*!< Data read access time 2 in CLK cycles */
    __IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency              */
    __IO uint16_t CardComdClasses;      /*!< Card command classes                  */
    __IO uint8_t  RdBlockLen;           /*!< Max. read data block length           */
    __IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed       */
    __IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment              */
    __IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment               */
    __IO uint8_t  DSRImpl;              /*!< DSR implemented                       */
    __IO uint8_t  Reserved2;            /*!< Reserved                              */
    __IO uint32_t DeviceSize;           /*!< Device Size                           */
    __IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min           */
    __IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max           */
    __IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min          */
    __IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max          */
    __IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier                */
    __IO uint8_t  EraseGrSize;          /*!< Erase group size                      */
    __IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier           */
    __IO uint8_t  WrProtectGrSize;      /*!< Write protect group size              */
    __IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable            */
    __IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC              */
    __IO uint8_t  WrSpeedFact;          /*!< Write speed factor                    */
    __IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length          */
    __IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed      */
    __IO uint8_t  Reserved3;            /*!< Reserved                              */
    __IO uint8_t  ContentProtectAppli;  /*!< Content protection application        */
    __IO uint8_t  FileFormatGroup;      /*!< File format group                     */
    __IO uint8_t  CopyFlag;             /*!< Copy flag (OTP)                       */
    __IO uint8_t  PermWrProtect;        /*!< Permanent write protection            */
    __IO uint8_t  TempWrProtect;        /*!< Temporary write protection            */
    __IO uint8_t  FileFormat;           /*!< File format                           */
    __IO uint8_t  ECC;                  /*!< ECC code                              */
    __IO uint8_t  CSD_CRC;              /*!< CSD CRC                               */
    __IO uint8_t  Reserved4;            /*!< Always 1                              */
} BSP_SD_CardCSDTypeDef_t;

/** @defgroup SD_Exported_Types_Group5 Card Identification Data: CID Register
  * @{
  */
typedef struct _BSP_SD_CardCIDTypeDef_t {
    __IO uint8_t  ManufacturerID;  /*!< Manufacturer ID       */
    __IO uint16_t OEM_AppliID;     /*!< OEM/Application ID    */
    __IO uint32_t ProdName1;       /*!< Product Name part1    */
    __IO uint8_t  ProdName2;       /*!< Product Name part2    */
    __IO uint8_t  ProdRev;         /*!< Product Revision      */
    __IO uint32_t ProdSN;          /*!< Product Serial Number */
    __IO uint8_t  Reserved1;       /*!< Reserved1             */
    __IO uint16_t ManufactDate;    /*!< Manufacturing Date    */
    __IO uint8_t  CID_CRC;         /*!< CID CRC               */
    __IO uint8_t  Reserved2;       /*!< Always 1              */
} BSP_SD_CardCIDTypeDef_t;

 /** @defgroup SD_Exported_Types_Group2 SD Card State enumeration structure
   * @{
   */
typedef uint32_t BSP_SD_CardStateTypeDef;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern void BSP_SD_WriteCpltCallback(void);
extern void BSP_SD_ReadCpltCallback(void);


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
BSP_SD_Status_t BSP_InitSD(BSP_SD_t sd);
BSP_StatusTypeDef_t BSP_InitSDCard(BSP_SD_t sd);
BSP_StatusTypeDef_t BSP_DeInitSD(BSP_SD_t sd);

/* ------------------- SD CHECK ------------------- */
BSP_SD_StateTypeDef_t BSP_GetStateSD(BSP_SD_t sd);
uint32_t BSP_SD_GetError(BSP_SD_t sd);

/* ------------------- SD CARD DETECTION ------------------- */
//BSP_SD_Transfer_Status_t BSP_SD_IsDetected(void);

/* ------------------- PERIPHERAL CONTROL FUNCTIONS ------------------- */
BSP_StatusTypeDef_t BSP_GetSDCID(BSP_SD_t sd, BSP_SD_CardCIDTypeDef_t* pCID);
BSP_StatusTypeDef_t BSP_GetSDCSD(BSP_SD_t sd, BSP_SD_CardCSDTypeDef_t* pCSD);
BSP_StatusTypeDef_t BSP_GetSDStat(BSP_SD_t sd, BSP_SD_CardStatusTypeDef_t* pStat);
BSP_StatusTypeDef_t BSP_GetSDInfo(BSP_SD_t sd, BSP_SD_CardInfoTypeDef_t* pCDInfo);
BSP_StatusTypeDef_t BSP_ConfigSDWB(BSP_SD_t sd, uint32_t WideMode);
BSP_StatusTypeDef_t BSP_ConfigSDSB(BSP_SD_t sd, uint32_t SpeedMode);
BSP_SD_CardStateTypeDef BSP_GetSDState(BSP_SD_t sd);
BSP_StatusTypeDef_t BSP_AbortSDOp(BSP_SD_t sd);
BSP_StatusTypeDef_t BSP_AbortSDIT(BSP_SD_t sd);

/* ------------------- BLOCKING MODE ------------------- */
BSP_StatusTypeDef_t BSP_ReadSDBlock(BSP_SD_t sd, uint32_t* pData, uint32_t blockAdd, uint32_t numBlock, uint32_t timeout);
BSP_StatusTypeDef_t BSP_WriteSDBlock(BSP_SD_t sd, const uint32_t* pData, uint32_t blockAdd, uint32_t numBlock, uint32_t timeout);

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
BSP_StatusTypeDef_t BSP_ReadSDBlockIT(BSP_SD_t sd, uint8_t* pData, uint32_t blockAdd, uint32_t numBlock);
BSP_StatusTypeDef_t BSP_WriteSDBlockIT(BSP_SD_t sd, const uint8_t* pData, uint32_t blockAdd, uint32_t numBlock);

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
BSP_StatusTypeDef_t BSP_ReadSDBlockDMA(BSP_SD_t sd, uint32_t* pData, uint32_t blockAdd, uint32_t numBlock);
BSP_StatusTypeDef_t BSP_WriteSDBlockDMA(BSP_SD_t sd, const uint32_t* pData, uint32_t blockAdd, uint32_t numBlock);

/* ------------------- ERASE ------------------- */
BSP_StatusTypeDef_t BSP_EraseSDCard(BSP_SD_t sd, uint32_t blockStartAdd, uint32_t blockEndAdd);

/* ------------------- SD CALLBACKS ------------------- */
void HAL_SD_AbortCallback(SD_HandleTypeDef * hsd);
void HAL_SD_ErrorCallback(SD_HandleTypeDef * hsd);
void HAL_SD_RxCpltCallback(SD_HandleTypeDef * hsd);
void HAL_SD_TxCpltCallback(SD_HandleTypeDef* hsd);

//!The function according to the complete callback uses the function defined in sd_diskio.c.!

//void BSP_SetSDCB(BSP_SD_t sd, BSP_SDCBType_t callbackType, BSP_SDCBPtr_t callback, void* params);


#endif /* HAL_SD_MODULE_ENABLED */

#endif /* SDMMC_INC_BSP_SDMMC_H_ */
