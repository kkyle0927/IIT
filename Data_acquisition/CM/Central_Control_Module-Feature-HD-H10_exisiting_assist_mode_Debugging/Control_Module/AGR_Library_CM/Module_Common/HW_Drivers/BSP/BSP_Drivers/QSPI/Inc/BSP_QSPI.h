/*
 * BSP_QSPI.h
 *
 *  Created on: May 21, 2024
 *      Author: Angelrobotics
 */

#ifndef HW_DRIVERS_BSP_QSPI_INC_BSP_QSPI_H_
#define HW_DRIVERS_BSP_QSPI_INC_BSP_QSPI_H_

/**
 * @file bsp_qspi.h
 * @date Created on: May 21, 2024
 * @author AngelRobotics HW Team
 * @brief Board Support Package for Quad SPI functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */


#include "main.h"
#include "module.h"

/** @defgroup FLASH FLASH
  * @brief FLASH HAL BSP module driver
  * @
  */

#include "W25Q128JV.h"
#include "bsp_common.h"
#include "stm32h7xx_hal_qspi.h"



/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/* Definition for QSPI clock resources */
//#define QSPI_CLK_ENABLE()          __HAL_RCC_QSPI_CLK_ENABLE()
//#define QSPI_CLK_DISABLE()         __HAL_RCC_QSPI_CLK_DISABLE()
//#define QSPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
//#define QSPI_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
//#define QSPI_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
//#define QSPI_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
//#define QSPI_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
//#define QSPI_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
//
//#define QSPI_FORCE_RESET()         __HAL_RCC_QSPI_FORCE_RESET()
//#define QSPI_RELEASE_RESET()       __HAL_RCC_QSPI_RELEASE_RESET()

/* Memory Mapped Address */
#define QSPI_MEMMAPPED_ADDRESS				 0x90000000

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */


typedef enum _BSP_QSPI_StatusTypeDef_t {
    BSP_QSPI_OK       		= 0x00,
    BSP_QSPI_ERROR    		= 0x01,
    BSP_QSPI_BUSY     		= 0x02,
    BSP_QSPI_NOT_SUPPORTED  = 0x04,
	BSP_QSPI_SUSPENDED		= 0x08,
} BSP_QSPI_StatusTypeDef_t;




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

/* Init, De-init */
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Init  	 	  		(void);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_DeInit	 	  		(void);

/* Read, Write, Erase */
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Read  	 	  		(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Write 	 	  		(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Erase_Block	  		(uint32_t BlockAddress);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Erase_Chip 	  		(void);

/* Sub Functions */
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Abort					(void);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_Reset			  		(void);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetStatus  	  		(void);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetID			  		(QSPI_Info_t* pInfo);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetInfo		  		(QSPI_Info_t* pInfo);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_MemoryMappedMode		(void);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_GetMemMapMode      	(void);
BSP_QSPI_StatusTypeDef_t BSP_QSPI_ConfigQuadMode  		(void);


#endif /* HW_DRIVERS_BSP_QSPI_INC_BSP_QSPI_H_ */
