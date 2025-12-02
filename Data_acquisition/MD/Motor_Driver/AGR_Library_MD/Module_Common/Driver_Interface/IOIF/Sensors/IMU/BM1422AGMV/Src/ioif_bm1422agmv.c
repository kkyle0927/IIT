/**
*-----------------------------------------------------------
*                    3AXIS MAG IMU DRIVER
*-----------------------------------------------------------
* @file ioif_bm1422agmv.c
* @date Created on: Jul 28, 2023
* @author AngelRobotics HW Team
* @brief Driver code for the BM1422AGMV magnetometer.
*
* This source file provides functionality to interface
* with the BM1422AGMV magnetometer, including initialization,
* data retrieval, and control register configurations.
*
* Refer to the BM1422AGMV datasheet and related documents for more information.
*
* @ref BM1422AGMV Datasheet
*/

#include "ioif_bm1422agmv.h"

/** @defgroup I2C I2C
  * @brief I2C BM1422AGMV module driver
  * @{
  */
#ifdef IOIF_BM1422AGMV_ENABLED

#ifdef _USE_DEBUG_CLI
#include "cli.h"
#endif /* _USE_DEBUG_CLI */

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

#ifdef _USE_DEBUG_CLI
IOIF_MagData_t cliMagData={0,};
#endif /* _USE_DEBUG_CLI */


/**
*------------------------------------------------------------
*                      STATIC VARIABLES
*------------------------------------------------------------
* @brief Variables local to this module.
*/

#ifdef WALKON5_CM_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};
#endif /* L30_CM_ENABLED */

#ifdef L30_CM_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};
#endif /* L30_CM_ENABLED */

#ifdef L30_MD_REV06_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c1RxBuff"))) = {0}; // not in use
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c1RxBuff"))) = {0};
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c1RxBuff"))) = {0};
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MINICM_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};
#endif /* SUIT_MINICM_ENABLED */

#ifdef SUIT_MD_ENABLED
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c1RxBuff"))) = {0};
#endif /* SUIT_MD_ENABLED */

#ifdef WIDM_ENABLED
// TODO : Port Change!! U5!!
static uint8_t bmDmaRxBuff[IOIF_BM1422AGMV_BUFF_SIZE] __attribute__((section(".i2c3RxBuff"))) = {0};
#endif /* SUIT_WIDM_ENABLED */

static BM1422AGMVObj_t bm1422agmvObj;
static BM1422AGMVIOctx_t bm1422agmvIOctx;
static IOIF_I2C_t i2cHandle;


/**
*------------------------------------------------------------
*                 STATIC FUNCTION PROTOTYPES
*------------------------------------------------------------
* @brief Static Function prototypes for this module.
*/

static uint8_t IsDevReady(uint16_t devAddr);
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);


/**
*------------------------------------------------------------
*                      PUBLIC FUNCTIONS
*------------------------------------------------------------
* @brief Functions that interface with this module.
*/

/**
* @brief Initialize the 6-axis IMU sensor.
* @param i2c Enumeration representing the I2C channel to use.
* @return Status of the initialization operation.
*/
uint8_t IOIF_InitMag(IOIF_I2C_t i2c)
{
    bm1422agmvIOctx.IsDevReady = IsDevReady;
    bm1422agmvIOctx.ReadReg = ReadReg;
    bm1422agmvIOctx.WriteReg = WriteReg;

    i2cHandle = i2c;
    // for DMA Read
    bm1422agmvObj.dataBuff = bmDmaRxBuff;

    uint8_t status = BM1422AGMV_SetIoctx(&bm1422agmvObj, &bm1422agmvIOctx);
    if (status != BM1422AGMV_STATUS_OK) {
        return status; // Error handling
    }

    // Initialize the BM1422AGMV object
    // You should have a function for initialization
    status = BM1422AGMV_Init(&bm1422agmvObj);
    if (status != BM1422AGMV_STATUS_OK) {
        return status; // Error handling
    }

//#ifdef _USE_DEBUG_CLI
//    CLI_CMDAdd("bm1422agmv", CLI_RunBM1422AGMV);// add cli cmd
//#endif

    return status;
}

/**
* @brief Retrieve the current values from the 6-axis IMU sensor.
* @param imuData Pointer to a structure to store the retrieved data.
* @return Status of the data retrieval operation.
*/
uint8_t IOIF_GetMagValue(IOIF_MagData_t* magData)
{
    // Check for NULL pointer
    if (magData == NULL) {
        return IOIF_IMU3AXIS_STATUS_ERROR;
    }

    // Initialize acceleration and gyroscope data
    memset(&bm1422agmvObj.magData, 0, sizeof(bm1422agmvObj.magData));

    // Get the value from the hardware object and check for errors
    uint8_t status = BM1422AGMV_GetValue(&bm1422agmvObj);
    if (status != IOIF_IMU3AXIS_STATUS_OK) {
        return status;
    }

    memcpy(magData, &bm1422agmvObj.magData, sizeof(bm1422agmvObj.magData));

    return status;
}


/**
*------------------------------------------------------------
*                      STATIC FUNCTIONS
*------------------------------------------------------------
* @brief Functions intended for internal use within this module.
*/

// Checks if the IMU 6-Axis device is ready
static uint8_t IsDevReady(uint16_t devAddr)
{
    return BSP_IsDevReadyI2C((BSP_I2C_t)i2cHandle, devAddr, IOIF_BM1422AGMV_TRIALS, IOIF_BM1422AGMV_TIMEOUT);
}

// Reads a register from the IMU 6-Axis device using DMA
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return BSP_I2CMemDMA((BSP_I2C_t)i2cHandle, devAddr, regAddr, BSP_I2C_MEMADD_SIZE_8BIT, pData, size, BSP_I2C_MEM_READ_DMA);
}

// Writes to a register on the IMU 6-Axis device using blocking method
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return BSP_I2CMemBlock((BSP_I2C_t)i2cHandle, devAddr, regAddr, BSP_I2C_MEMADD_SIZE_8BIT, pData, size, IOIF_BM1422AGMV_TIMEOUT, BSP_I2C_MEM_WRITE);
}

/**
*------------------------------------------------------------
*                      CLI FUNCTIONS
*------------------------------------------------------------
* @brief Functions are supported Commmand Line Interface (CLI).
*/

#ifdef _USE_DEBUG_CLI
void CLI_RunBM1422AGMV(cli_args_t *args)
{
    bool ret = false;
    uint32_t duration = 10;                             //default,

    if(args->argc == 2 && args->cmpStr(0, "getdata") == true)
    {
        duration = (uint32_t) args->getData(1);         // param. 1

        if(duration < 10)   ret = false;
        else                ret = true;

        while(ret && CLI_KeepLoop())
        {
            IOIF_GetMagValue(&cliMagData);              // testing ioif functions
            /* Output to String */
            CLI_Printf( "X: %f, Y: %f, Z: %f \r\n", cliMagData.magX, cliMagData.magY, cliMagData.magZ);
            /* Duration */
            CLI_Delay(duration);
        }
    }

    if(ret == false)        //help
    {
        CLI_Printf(" - Usage : bm1422agmv getdata [duration(ms) (min. 10ms)] \r\n");
        CLI_Printf(" - Get bm1422agmv magnet sensor data at a specified period of time[ms]. \r\n");
        CLI_Printf(" - Example : bm1422agmv getdata 100 \r\n");
    }
}

#endif /* _USE_DEBUG_CLI */

#endif /* IOIF_BM1422AGMV_ENABLED */
