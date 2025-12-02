/**
 *-----------------------------------------------------------
 *                  MCP7510T-I/MS RTC driver
 *-----------------------------------------------------------
 * @file ioif_mcp79510.c
 * @date Created on: Nov 29, 2024
 * @author AngelRobotics FW Team
 * @brief Code for the MCP7510T-I/MS RTC driver.
 *
 *
 * Refer to the MCP7510T-I/MS datasheet and related documents for more information.
 * @ref MCP7510T-I/MS Datasheet
 */

#include "ioif_mcp79510.h"

/** @defgroup SPI SPI
 * @brief SPI MCP7510T-I/MS RTC driver
 * @{
 */

#ifdef IOIF_MCP79510_ENABLED

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

rtcTime_t    rtcValue_t;
MCP79510_IOt rtcSPICBStruct;

uint8_t rtcSPIRXDMABuff[RTC_SPI_RX_BUFF_LENGTH] __attribute__((section(".spi2RxBuff"))) = {0,};
uint8_t rtcSPITXDMABuff[RTC_SPI_TX_BUFF_LENGTH] __attribute__((section(".spi2TxBuff"))) = {0,};



/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

#ifdef _USE_DEBUG_CLI
static IOIF_MCP79510_State_t moduleinit_res = IOIF_MCP79510_STATE_ERROR;
#endif

static volatile bool rtcspi_cs_autocontrol = false;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static bool IOIF_rtcSPITransmit(uint8_t *pTxData, uint16_t size, bool cs_autoctrl);
static bool IOIF_rtcSPIReceive(uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
static bool IOIF_rtcSPITransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
static void IOIF_rtcSPICS(uint8_t state);
static void IOIF_rtcSPIIOWait(uint32_t wait_ms);

static void rtcSPITXCB(void* param);
static void rtcSPIRXCB(void* param);
static void rtcSPITXRXCB(void* param);



/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_MCP79510_State_t IOIF_MCP79510_Init(void)
{
	IOIF_MCP79510_State_t ret = IOIF_MCP79510_STATE_OK;

	/* Callback Function Registration */
	rtcSPICBStruct.rtcSPICS 	  		 = IOIF_rtcSPICS;
	rtcSPICBStruct.rtcSPITransmit 		 = IOIF_rtcSPITransmit;
	rtcSPICBStruct.rtcSPIReceive		 = IOIF_rtcSPIReceive;
	rtcSPICBStruct.rtcSPITransmitReceive = IOIF_rtcSPITransmitReceive;
	rtcSPICBStruct.rtcSPICS_IOwait		 = IOIF_rtcSPIIOWait;

	MCP79510_RegisterIOCallBack(&rtcSPICBStruct);

	BSP_SetSPICB(IOIF_RTC_SPI_PORT, BSP_SPI_TX_CPLT_CALLBACK, rtcSPITXCB, NULL);
	BSP_SetSPICB(IOIF_RTC_SPI_PORT, BSP_SPI_RX_CPLT_CALLBACK, rtcSPIRXCB, NULL);
	BSP_SetSPICB(IOIF_RTC_SPI_PORT, BSP_SPI_TXRX_CPLT_CALLBACK, rtcSPITXRXCB, NULL);

	/* CS pin init. */
	IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_SET);

	if(MCP79510_init(&rtcSPITXDMABuff[0], &rtcSPIRXDMABuff[0]) != true)
		return ret;

#ifdef _USE_DEBUG_CLI
	moduleinit_res = ret;
#endif

	return ret;
}

IOIF_MCP79510_State_t IOIF_MCP79510_SetTime(rtcTime_t* rtcValue)
{
	IOIF_MCP79510_State_t ret = IOIF_MCP79510_STATE_OK;

	if(MCP79510_rtc_set_time(rtcValue) != true)
		ret = IOIF_MCP79510_STATE_ERROR;

	return ret;
}

IOIF_MCP79510_State_t IOIF_MCP79510_GetTime(rtcTime_t* rtcValue)
{
	IOIF_MCP79510_State_t ret = IOIF_MCP79510_STATE_OK;

	if(MCP79510_rtc_get_time(rtcValue) != true)
		ret = IOIF_MCP79510_STATE_ERROR;

	return ret;
}


time_t IOIF_MCP79510_convert_to_linuxtime(rtcTime_t *rtcValue) {
    struct tm time_info;
    
    // rtcValue->year에 2000을 더하여 연도를 계산
    time_info.tm_year = rtcValue->year + 100; // tm_year는 1900년을 기준으로 계산
    time_info.tm_mon = rtcValue->mon - 1; // tm_mon은 0부터 시작 (0: January, 11: December)
    time_info.tm_mday = rtcValue->mday;
    time_info.tm_hour = rtcValue->hour;
    time_info.tm_min = rtcValue->min;
    time_info.tm_sec = rtcValue->sec;
    time_info.tm_wday = rtcValue->wday; // tm_wday는 요일 정보 (0: Sunday, 6: Saturday)

    // tm_isdst는 Daylight Saving Time 여부, 기본적으로 -1로 설정 (자동 계산)
    time_info.tm_isdst = -1;

    time_t timestamp = mktime(&time_info);

    // KST로 변환, 9시간은 9*60*60 초입니다.
    timestamp -= 9 * 60 * 60;

    return timestamp;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static bool IOIF_rtcSPITransmit(uint8_t *pTxData, uint16_t size, bool cs_autoctrl)
{
	bool ret = true;

	/* use DMA */
//	if(cs_autoctrl == true)
//		rtcspi_cs_autocontrol = true;
//	else
//		rtcspi_cs_autocontrol = false;
//
//	IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_RESET);	//CS low
//
//	if(BSP_RunSPIDMA(IOIF_RTC_SPI_PORT, pTxData, NULL, size, BSP_SPI_TRANSMIT_DMA) != BSP_OK)
//		ret = false;

	/* Normal */
	if(BSP_RunSPIBlock(IOIF_RTC_SPI_PORT, pTxData, NULL, size, 1000, BSP_SPI_TRANSMIT) != BSP_OK)
		ret = false;

	return ret;
}


static bool IOIF_rtcSPIReceive(uint8_t *pRxData, uint16_t size, bool cs_autoctrl)
{
	bool ret = true;

//	/* use DMA */
//	IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_RESET);	//CS low
//
//	if(BSP_RunSPIDMA(IOIF_RTC_SPI_PORT, NULL, pRxData, size, BSP_SPI_RECEIVE_DMA) != BSP_OK)
//		ret = false;
//
//	if(cs_autoctrl == true)
//		rtcspi_cs_autocontrol = true;
//	else
//		rtcspi_cs_autocontrol = false;

	/* Normal */
	if(BSP_RunSPIBlock(IOIF_RTC_SPI_PORT, NULL, pRxData, size, 1000, BSP_SPI_RECEIVE) != BSP_OK)
		ret = false;

	return ret;
}


static bool IOIF_rtcSPITransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size, bool cs_autoctrl)
{
	bool ret = true;

	if(cs_autoctrl == true)
		rtcspi_cs_autocontrol = true;
	else
		rtcspi_cs_autocontrol = false;

	IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_RESET);	//CS low

	if(BSP_RunSPIDMA(IOIF_RTC_SPI_PORT, pTxData, pRxData, size, BSP_SPI_TRANSMIT_RECEIVE_DMA) != BSP_OK)
		ret = false;

	return ret;
}


static void IOIF_rtcSPICS(uint8_t state)
{
	if(state == IOIF_SPI_CS_LOW)
		IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_RESET);
	else
		IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
}


static void IOIF_rtcSPIIOWait(uint32_t wait_ms)
{
#ifdef _USE_OS_RTOS
	osDelay(wait_ms);
#else
	HAL_Delay(wait_ms);
#endif
}

/* SPI TX,RX, TX/RX Callback */

static void rtcSPITXCB(void* param)
{
	if(rtcspi_cs_autocontrol == true)
		IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
}

static void rtcSPIRXCB(void* param)
{
	if(rtcspi_cs_autocontrol == true)
		IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
}

static void rtcSPITXRXCB(void* param)
{
	if(rtcspi_cs_autocontrol == true)
		IOIF_WriteGPIOPin(IOIF_RTC_SPI_CS_PORT, IOIF_RTC_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
}


#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

void CLI_RunMCP79510(cli_args_t *args)
{
	bool ret = false;

	rtcTime_t clirtcgettime = {0,};
	rtcTime_t clirtcsettime = {0,};

	/* Init Check. */
	if(args->argc == 1 && args->cmpStr(0, "isinit") == true)
	{
		if(moduleinit_res != IOIF_MCP79510_STATE_OK)
			CLI_Printf(" * MCP79510 init fail! * \r\n");
		else
			CLI_Printf(" * MCP79510 init success. * \r\n");

		ret = true;
	}

	/* Get Time. */
	else if(args->argc == 2 && args->cmpStr(0, "gettime") == true)
	{
		uint32_t duration = args->getData(1);

		if(!duration)
		{
			if(IOIF_MCP79510_GetTime(&clirtcgettime) == IOIF_MCP79510_STATE_OK)
			{
				CLI_Printf(" * %d[ss] %d[mm] %d[hh] %d[weekday(1-7)] %d[monthday(01-31)] %d[month(01-12)] %d[year(00-99)] \r\n", clirtcgettime.sec, clirtcgettime.min, clirtcgettime.hour, clirtcgettime.wday, clirtcgettime.mday, clirtcgettime.mon, clirtcgettime.year);
			}
			else
				CLI_Printf(" * MCP79510 get time is fail! * \r\n");
		}
		else
		{
			while(CLI_KeepLoop())
			{
				if(IOIF_MCP79510_GetTime(&clirtcgettime) == IOIF_MCP79510_STATE_OK)
				{
					CLI_Printf(" * %d[ss] %d[mm] %d[hh] %d[weekday(1-7)] %d[monthday(01-31)] %d[month(01-12)] %d[year(00-99)] \r\n", clirtcgettime.sec, clirtcgettime.min, clirtcgettime.hour, clirtcgettime.wday, clirtcgettime.mday, clirtcgettime.mon, clirtcgettime.year);
				}
				else
					CLI_Printf(" * MCP79510 get time is fail! * \r\n");

				osDelay(duration);
			}
		}
		ret = true;
	}

	/* Set Time. */
	else if(args->argc == 8 && args->cmpStr(0, "settime") == true)
	{
		clirtcsettime.sec = args->getData(1);
		clirtcsettime.min = args->getData(2);
		clirtcsettime.hour = args->getData(3);
		clirtcsettime.wday = args->getData(4);
		clirtcsettime.mday = args->getData(5);
		clirtcsettime.mon = args->getData(6);
		clirtcsettime.year = args->getData(7);

		if(IOIF_MCP79510_SetTime(&clirtcsettime) == IOIF_MCP79510_STATE_OK)
		{
			CLI_Printf(" * MCP79510 set time success. * \r\n");
		}
		else
			CLI_Printf(" * MCP79510 get time is fail! * \r\n");

		ret = true;
	}

	else if(ret == false) //help
	{
		CLI_Printf(" * RTC IC MCP79510* \r\n");
		CLI_Printf("   Load and store current RTCC(Real Time Clock and Calendar) \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   mcp79510 isinit \r\n");
		CLI_Printf("   mcp79510 gettime [duration(ms)] \r\n");
		CLI_Printf("   mcp79510 settime [ss(0-59)] [mm(0-59)] [hh(00-23)] [weekday(1-7)] [monthday(1-31)] [month(1-12)] [year(00-99)] \r\n");

		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   mcp79510 gettime 1000 \r\n");
		CLI_Printf("   mcp79510 settime 53 52 21 3 23 11 24 \r\n");
	}
}


#endif /*_USE_DEBUG_CLI*/
#endif /* IOIF_MCP79510_ENABLED */
