/*
 * MCP79510T.c
 *
 *  Created on: Dec 2, 2024
 *      Author: Angelrobotics
 */


#include "MCP79510T.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
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

MCP79510_IOt MCP79510_SPICB;

uint8_t* spitxdmabuff;
uint8_t* spirxdmabuff;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                     STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static uint8_t bcd2bin(char val);
static char	bin2bcd(uint8_t val);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool MCP79510_init(uint8_t* tx_buff, uint8_t* rx_buff)
{
	bool ret = true;
	uint8_t control_reg_init = 0;

	/* dma buffer copy */
	spitxdmabuff = tx_buff;
	spirxdmabuff = rx_buff;

	/* 1. Init. Control Register ensuring external osc. disabled */
	MCP79510_rtc_write(MCP79510_CONTROL, &control_reg_init, 1);

	/* 2. Start Osc. without EXTOSC bit */
	MCP79510_rtc_startoscillator(NULL);

	/* 3. Backup Battery Enable */
	if(MCP79510_rtc_vbat_enable() != true)
		ret = false;

	/* 4. Clear 12/24 hour mode : set 24 hour mode */
	if(MCP79510_rtc_setbit(MCP79510_RTCHOUR, RTC_1224, 0) != true)
		ret = false;

	return ret;
}


bool MCP79510_rtc_read(uint8_t addr, uint8_t* readbuf, uint32_t length)
{
	bool ret = true;

	uint8_t txData[2] = {0,};

	txData[0] = MCP79510_READ;
	txData[1] = addr;

	/* copy to txdata to dma buff. */
	memcpy(spitxdmabuff, txData, sizeof(txData));

	MCP79510_SPICB.rtcSPICS(CS_LOW);
	if(MCP79510_SPICB.rtcSPITransmit(spitxdmabuff, 2, false) == true)
	{
		if(MCP79510_SPICB.rtcSPIReceive(spirxdmabuff, length, true) != true)
			ret = false;
	}
	else ret = false;
	MCP79510_SPICB.rtcSPICS(CS_HIGH);


	/* insert data to read buffer */
	if(ret != false)
		for(int i =0; i < length; i++)	readbuf[i] = spirxdmabuff[i];

	return ret;
}


bool MCP79510_rtc_write(uint8_t addr, uint8_t* writebuf, uint32_t length)
{
	bool ret = true;

	uint8_t txData[257] = {0,};

	txData[0] = MCP79510_WRITE;
	txData[1] = addr;

	memcpy(&txData[2], writebuf, 2 + length);
	memcpy(spitxdmabuff, txData, 2 + length);

	MCP79510_SPICB.rtcSPICS(CS_LOW);
	if(MCP79510_SPICB.rtcSPITransmit(spitxdmabuff, 2 + length, true) != true)
		ret = false;
	MCP79510_SPICB.rtcSPICS(CS_HIGH);

	return ret;
}


bool MCP79510_rtc_setbit(uint8_t addr, uint8_t maskbit, uint8_t state)
{
	bool ret = true;

	uint8_t temp;

	if(MCP79510_rtc_read(addr, &temp, 1) != true)
		return ret = false;

	if((temp & maskbit) != state)
	{
		temp = (temp & ~maskbit) | state;
		if(MCP79510_rtc_write(addr, &temp, 1) != true)
			ret = false;
	}

	return ret;
}


bool MCP79510_rtc_startoscillator(bool *extosc)
{
	bool ret = true;
	uint8_t retry = 10;

	if(extosc)
	{
		uint8_t temp = *extosc ? RTC_EXTOSC : 0;
		bool check;

		check = MCP79510_rtc_setbit(MCP79510_CONTROL, RTC_EXTOSC, temp);
		if(!check)
			return ret = false;
	}
	else
		ret = MCP79510_rtc_setbit(MCP79510_RTCSEC, RTC_ST, RTC_ST);

	/* Wait for OSCRUN bit is set */
	do
		{
			uint8_t temp = RTC_OSCRUN;
			MCP79510_SPICB.rtcSPICS_IOwait(1);			//waiting bit is clear
			if(MCP79510_rtc_read(MCP79510_RTCWKDAY, &temp, 1) == true)
			{
				if((temp & RTC_OSCRUN))
					break;
			}

		}while(--retry);

	if(retry == 0)
		ret = false;

	return ret;
}


bool MCP79510_rtc_stoposcillator(bool *extosc)
{
	bool ret = true;
	uint8_t retry = 10;

	uint8_t temp;

	/* 1. Oscillation Stop */
	if(MCP79510_rtc_setbit(MCP79510_RTCSEC, RTC_ST, 0) != true)
		return ret = false;

	/* 2. Read External Osc. used */
	if(MCP79510_rtc_read(MCP79510_CONTROL, &temp, 1) != true)
		return ret = false;

	/* 3. Store current EXTOSC bit */
	*extosc = !!(temp & RTC_EXTOSC);

	/* 4. clear EXTOSC bit */
	if(MCP79510_rtc_setbit(MCP79510_CONTROL, RTC_EXTOSC, 0) != true)
		return ret = false;

	/* 5. wait for OSCRUN bit to clear */
	do
	{
		MCP79510_SPICB.rtcSPICS_IOwait(1);			//waiting bit is clear
		if(MCP79510_rtc_read(MCP79510_RTCWKDAY, &temp, 1) == true)
		{
			if(!(temp & RTC_OSCRUN))
				break;
		}
	}while(--retry);

	if(retry == 0)
		ret = false;

	return ret;
}


bool MCP79510_rtc_vbat_enable(void)
{
	bool ret = true;

	uint8_t temp;

	/* 1. PWRFAIL BIT is set when VBAT powered up before VCC */
	if(MCP79510_rtc_read(MCP79510_RTCWKDAY, &temp, 1) != true)
		return ret = false;

	if(!!(temp & RTC_PWRFAIL))
	{
		if(MCP79510_rtc_setbit(MCP79510_RTCWKDAY, RTC_PWRFAIL, 0) != true)
			return ret = false;
	}

	/* 2. set VBAT Flag to high */
	if(MCP79510_rtc_setbit(MCP79510_RTCWKDAY, RTC_VBATEN, RTC_VBATEN) != true)
		ret = false;

	return ret;
}


bool MCP79510_rtc_get_time(rtcTime_t* rtcTim)
{
	bool ret = true;

	uint8_t data[7] = {0,};

	if(MCP79510_rtc_read(MCP79510_RTCSEC, data, sizeof(data)) != true)
		return ret = false;

	rtcTim->sec  = bcd2bin(data[0] & 0x7f);
	rtcTim->min  = bcd2bin(data[1] & 0x7f);
	rtcTim->hour = bcd2bin(data[2] & 0x3f);
	rtcTim->wday = bcd2bin(data[3] & 0x07);
	rtcTim->mday = bcd2bin(data[4] & 0x3f);
	rtcTim->mon  = bcd2bin(data[5] & 0x1f);
	rtcTim->year = bcd2bin(data[6]);

	return ret;
}


bool MCP79510_rtc_set_time(rtcTime_t* rtcTim)
{
	bool ret = true;

	uint8_t data[7] = {0,};
	bool extosc;

	/* 1. Stop RTC and store value */
	if(MCP79510_rtc_stoposcillator(&extosc) != true)
		return ret = false;

	/* 2. Read clock and calendar */
	if(MCP79510_rtc_read(MCP79510_RTCSEC, data, sizeof(data)) != true)
		return ret = false;

	data[0] = (data[0] & 0x80) | bin2bcd(rtcTim->sec);
	data[1] = (data[1] & 0x80) | bin2bcd(rtcTim->min);
	data[2] = bin2bcd(rtcTim->hour);
	data[3] = (data[3] & 0xf8) | bin2bcd(rtcTim->wday);
	data[4] = bin2bcd(rtcTim->mday);
	data[5] = (data[5] & RTC_AMPM) | bin2bcd(rtcTim->mon);
	data[6] = bin2bcd(rtcTim->year);

	/* 3. Write time and date */
	if(MCP79510_rtc_write(MCP79510_RTCSEC, &data[0], 4) != true)
		ret = false;
	if(MCP79510_rtc_write(MCP79510_RTCDATE, &data[4], 3) != true)
		ret = false;

	/* 4. Re-start Ext.Osc */
	if(MCP79510_rtc_startoscillator(extosc ? &extosc : NULL) != true)
		ret = false;

	return ret;
}


/* Callback Registration Function */

bool MCP79510_RegisterIOCallBack(MCP79510_IOt* rtcIOCallback)
{
	bool ret = true;

	if(!rtcIOCallback->rtcSPICS || !rtcIOCallback->rtcSPITransmit || !rtcIOCallback->rtcSPIReceive || !rtcIOCallback->rtcSPITransmitReceive || !rtcIOCallback->rtcSPICS_IOwait)
		return ret = false;

	MCP79510_SPICB.rtcSPICS 	  		 = rtcIOCallback->rtcSPICS;
	MCP79510_SPICB.rtcSPITransmit 		 = rtcIOCallback->rtcSPITransmit;
	MCP79510_SPICB.rtcSPIReceive		 = rtcIOCallback->rtcSPIReceive;
	MCP79510_SPICB.rtcSPITransmitReceive = rtcIOCallback->rtcSPITransmitReceive;
	MCP79510_SPICB.rtcSPICS_IOwait		 = rtcIOCallback->rtcSPICS_IOwait;

	return ret;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static uint8_t bcd2bin(char val)
{
	return (val & 0x0f) + (val >> 4) *10;
}

static char	bin2bcd(uint8_t val)
{
	return ((val / 10) << 4) + val % 10;
}


