/**
 *-----------------------------------------------------------
 *       LTC2944 IO INTERFACE BATTERY GAS GAUGE DRIVER
 *-----------------------------------------------------------
 * @file ioif_ltc2944.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief LTC2944 IO Interface Driver Implementation.
 *
 * This file implements the functions required to interact with the LTC2944 Battery Gas Gauge IC
 * at a higher interface level. It provides an abstraction layer for using the hardware and BSP functions
 * to interact with the LTC2944 IC, making it suitable for use at the task level.
 *
 * This driver is built upon the lower level LTC2944 driver and is designed to simplify the interactions
 * with the IC for higher level tasks and applications.
 *
 * @ref LTC2944 Datasheet
 */

#include "ioif_ltc2944.h"

/** @defgroup I2C I2C
  * @brief I2C Battery Monitor module driver
  * @{
  */
#ifdef IOIF_BATMONITOR_ENABLED


#ifdef _USE_DEBUG_CLI
#include "cli.h"
#endif


/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

static IOIF_I2C_t i2c_port = IOIF_I2C4;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

#ifdef SUIT_MINICM_ENABLED
static uint8_t batMonitorDmaRxBuff[IOIF_BATMONITOR_BUFF_SIZE] __attribute__((section(".i2c4RxBuff"))) = {0};
#endif /* SUIT_MINICM_ENABLED */

#ifdef SUIT_MD_ENABLED

#endif /* SUIT_MD_ENABLED */

static LTC2944Obj_t batMonitor2944Obj;
static LTC2944IOctx_t batMonitor2944IOctx;
static IOIF_I2C_t i2cHandle;

#ifdef _USE_DEBUG_CLI
static IOIF_BatMonitor_State_t moduleinit_res = 0;
#endif /*_USE_DEBUG_CLI*/
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


IOIF_BatMonitor_State_t IOIF_BatMonitor_Init(void)
{
	IOIF_BatMonitor_State_t ret = IOIF_BATMONITOR_STATUS_OK;

    batMonitor2944IOctx.IsDevReady = IsDevReady;
    batMonitor2944IOctx.ReadReg = ReadReg;
    batMonitor2944IOctx.WriteReg = WriteReg;
    i2cHandle = i2c_port;
    batMonitor2944Obj.dataBuff = batMonitorDmaRxBuff; // for DMA Read

    LTC29444State_t status = LTC2944_SetIoctx(&batMonitor2944Obj, &batMonitor2944IOctx);
    if (status == LTC2944_STATUS_OK) LTC2944_Init(&batMonitor2944Obj);

    /* recovery */
    if (status != LTC2944_STATUS_OK)
    {
    	for (int trial = 0; trial < LTC2944_TRIALS; trial++)
    	{
    		ret = IOIF_BATMONITOR_STATUS_ERROR;
    		if (IOIF_I2C_InitRecovery(IOIF_I2C4, IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_15,
    				IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_14) == IOIF_I2C_STATUS_OK)
    		{
    			if (LTC2944_SetIoctx(&batMonitor2944Obj, &batMonitor2944IOctx) == LTC2944_STATUS_OK)
    				if (LTC2944_Init(&batMonitor2944Obj) == LTC2944_STATUS_OK)
    				{
    					ret = IOIF_BATMONITOR_STATUS_OK;
    					break;
    				}
    		}
    	}
    }

#ifdef _USE_DEBUG_CLI
    moduleinit_res = status;
#endif /*_USE_DEBUG_CLI*/

    return ret;
}

/**
 * @brief Retrieve the current values from the 6-axis IMU sensor.
 * @param batData Pointer to a structure to store the retrieved data.
 * @return Status of the data retrieval operation.
 */
IOIF_BatMonitor_State_t IOIF_BatMonitor_GetValue(IOIF_BatMonitor_Data_t* batData)
{
    // Check for NULL pointer
    if (batData == NULL) {
        return IOIF_BATMONITOR_STATUS_ERROR;
    }

    // Initialize acceleration and gyroscope data
	memset(&batMonitor2944Obj.batData, 0, sizeof(batMonitor2944Obj.batData));

    // Get the value from the hardware object and check for errors
    uint8_t status = LTC2944_GetValue(&batMonitor2944Obj);
    if (status != IOIF_BATMONITOR_STATUS_OK) {
        return status;
    }

    memcpy(batData, &batMonitor2944Obj.batData, sizeof(batMonitor2944Obj.batData));

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

// Checks if the Battery Monitor Device is ready
static uint8_t IsDevReady(uint16_t devAddr)
{
    return BSP_IsDevReadyI2C((BSP_I2C_t)i2cHandle, devAddr, IOIF_BATMONITOR_TRIALS, IOIF_BATMONITOR_TIMEOUT);
}

// Reads a register from the Battery Monitor Device using DMA
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return BSP_I2CMasterDMA((BSP_I2C_t)i2cHandle, devAddr, pData, size, BSP_I2C_MASTER_RECEIVE_DMA);
}

// Writes to a register on the Battery Monitor Device using blocking method
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
	return BSP_I2CMemBlock((BSP_I2C_t)i2cHandle, devAddr, regAddr, BSP_I2C_MEMADD_SIZE_8BIT, pData, size, IOIF_BATMONITOR_TIMEOUT, BSP_I2C_MEM_WRITE);
}


#ifdef _USE_DEBUG_CLI
/**
*------------------------------------------------------------
*                      CLI FUNCTIONS
*------------------------------------------------------------
* @brief Functions are supported Commmand Line Interface (CLI).
*/

void CLI_RunLtc2944(cli_args_t *args)
{
	bool ret = false;

	IOIF_BatMonitor_Data_t cliBatData;
	char BatDataStr[100];
	uint32_t duration = 10;								//default,


	if (args->cmpStr(0, "isinit") == true)
	{
		IOIF_BatMonitor_State_t state = moduleinit_res;

		const char* batStateStrings[] = {
				"IOIF_BATMONITOR_STATUS_OK",
				"IOIF_BATMONITOR_STATUS_ERROR",
				"IOIF_BATMONITOR_STATUS_BUSY",
				"IOIF_BATMONITOR_STATUS_TIMEOUT"
		};

		CLI_Printf(" * LTC2944 init state : %s * \r\n",batStateStrings[state]);

	} else if(args->argc == 2 && args->cmpStr(0, "getdata") == true) {

		CLI_Printf(" * LTC2944 data gathering started! * \r\n");
		duration = (uint32_t) args->getData(1);			// param. 1 에 대한 string 데이터를 정수로 변환

		if(duration < 10)	ret = false;
		else				ret = true;

		while(ret && CLI_KeepLoop())
		{
			IOIF_BatMonitor_GetValue(&cliBatData);				// testing ioif functions

			/* Output to String */
			sprintf(BatDataStr, "Bat Vol : %f V, Bat Current : %f A, Bat Temp. : %f deg.", cliBatData.batVolt, cliBatData.batCurr, cliBatData.brdTemp);
			CLI_Printf("%s \r\n", BatDataStr);

			/* Duration */
			CLI_Delay(duration);
		}
	} else if(ret == false)		//help
	{
		CLI_Printf(" * Power Monitoring * \r\n");
		CLI_Printf("   Get ltc2944 voltage/current/temperature sensor data at a specified period of time[ms] \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   ltc2944 isinit \r\n");
		CLI_Printf("   ltc2944 getdata [duration(ms) (min. 10ms)]\r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   ltc2944 getdata 100 \r\n");
	}
}
#endif /*_USE_DEBUG_CLI*/
#endif /* IOIF_BATMONITOR_ENABLED */
