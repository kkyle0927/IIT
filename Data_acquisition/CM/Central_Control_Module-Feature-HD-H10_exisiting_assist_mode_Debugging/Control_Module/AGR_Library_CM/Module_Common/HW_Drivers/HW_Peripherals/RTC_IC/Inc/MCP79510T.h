/*
 * MCP79510T.h
 *
 *  Created on: Dec 2, 2024
 *      Author: Angelrobotics
 */

#ifndef MODULE_COMMON_HW_DRIVERS_HW_PERIPHERALS_RTC_IC_INC_MCP79510T_H_
#define MODULE_COMMON_HW_DRIVERS_HW_PERIPHERALS_RTC_IC_INC_MCP79510T_H_

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* Definitions of Instruction Sets */

#define MCP79510_EEREAD   0b00000011 // Read from EEPROM starting at selected address
#define MCP79510_EEWRITE  0b00000010 // Write to EEPROM starting at selected address
#define MCP79510_EEWRDI   0b00000100 // Reset the write enable latch (disable write operations)
#define MCP79510_EEWREN   0b00000110 // Set the write enable latch (enable write operations)
#define MCP79510_SRREAD   0b00000101 // Read STATUS register
#define MCP79510_SRWRITE  0b00000001 // Write STATUS register
#define MCP79510_READ     0b00010011 // Read data from RTCC/SRAM array beginning at selected address
#define MCP79510_WRITE    0b00010010 // Write data to RTCC/SRAM array beginning at selected address
#define MCP79510_UNLOCK   0b00010100 // Unlock the protected EEPROM block for a write operation
#define MCP79510_IDWRITE  0b00110010 // Write data to the protected EEPROM block beginning at selected address
#define MCP79510_IDREAD   0b00110011 // Read data from the protected EEPROM block beginning at the selected address
#define MCP79510_CLRRAM   0b01010100 // Clear all SRAM data to ‘0’

/* Definitions of Register Address */
/* Sec5.3, "Time-Keeping" */
#define MCP79510_RTCHSEC		0x00
#define MCP79510_RTCSEC			0x01
#define MCP79510_RTCMIN			0x02
#define MCP79510_RTCHOUR		0x03
#define MCP79510_RTCWKDAY		0x04
#define MCP79510_RTCDATE		0x05
#define MCP79510_RTCMTH			0x06
#define MCP79510_RTCYEAR		0x07
#define MCP79510_CONTROL		0x08
#define MCP79510_OSCTRIM		0x09
/* Sec5.4, "Alarms" */
#define MCP79510_ALM0SEC		0x0c
#define MCP79510_ALM0MIN		0x0d
#define MCP79510_ALM0HOUR		0x0e
#define MCP79510_ALM0WKDAY		0x0f
#define MCP79510_ALM0DATE		0x10
#define MCP79510_ALM0MTH		0x11
#define MCP79510_ALM1HSEC		0x12
#define MCP79510_ALM1SEC		0x13
#define MCP79510_ALM1MIN		0x14
#define MCP79510_ALM1HOUR		0x15
#define MCP79510_ALM1WKDAY		0x16
#define MCP79510_ALM1DATE		0x17
/* Sec5.7, "Power-Fail Time-stamp" */
#define MCP79510_PWRDNMIN		0x18
#define MCP79510_PWRDNHOUR		0x19
#define MCP79510_PWRDNDATE		0x1a
#define MCP79510_PWRDNMTH		0x1b
#define MCP79510_PWRUPMIN		0x1c
#define MCP79510_PWRUPHOUR		0x1d
#define MCP79510_PWRUPDATE		0x1e
#define MCP79510_PWRUPMTH		0x1f

/* Definitions of Control Bits */
#define RTC_ST					(1<<7)
#define RTC_1224				(1<<6)
#define RTC_AMPM				(1<<5)
#define RTC_EXTOSC				(1<<3)
#define RTC_OSCRUN				(1<<5)
#define RTC_VBATEN				(1<<3)
#define RTC_PWRFAIL				(1<<4)



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


typedef enum _MCP79510_CS{
	CS_LOW = 0,
	CS_HIGH,
} MCP79510_CS;

typedef enum _MCP79510_EXTOSC_USED{
	EXTOSC_UNUSED = 0,
	EXTOSC_USED,
} MCP79510_EXTOSC_USED;

typedef struct _rtcTime_t {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t wday;
	uint8_t mday;
	uint8_t mon;
	uint8_t year;

} rtcTime_t;



/* Definition of IO Callback Function Pointer */
typedef bool (*rtcSPITransmit_fptr) 		(uint8_t *pTxData, uint16_t size, bool cs_autoctrl);
typedef bool (*rtcSPIReceive_fptr)			(uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
typedef bool (*rtcSPITransmitReceive_fptr)  (uint8_t *pTxData, uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
typedef void (*rtcSPICS_fptr) 				(uint8_t state);
typedef void (*rtcSPICS_IOwait_fptr)		(uint32_t wait_ms);

/* Definition of IO callback structurer */
typedef struct _MCP79510_IOt{
	rtcSPITransmit_fptr 		rtcSPITransmit;
	rtcSPIReceive_fptr			rtcSPIReceive;
	rtcSPITransmitReceive_fptr  rtcSPITransmitReceive;
	rtcSPICS_fptr				rtcSPICS;
	rtcSPICS_IOwait_fptr		rtcSPICS_IOwait;
} MCP79510_IOt;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

bool MCP79510_init(uint8_t*, uint8_t*);

bool MCP79510_rtc_read(uint8_t addr, uint8_t* readbuf, uint32_t length);
bool MCP79510_rtc_write(uint8_t addr, uint8_t* writebuf, uint32_t length);
bool MCP79510_rtc_setbit(uint8_t addr, uint8_t maskbit, uint8_t state);

bool MCP79510_rtc_startoscillator(bool* extosc);
bool MCP79510_rtc_stoposcillator(bool* extosc);
bool MCP79510_rtc_vbat_enable(void);

bool MCP79510_rtc_get_time(rtcTime_t* rtcTim);
bool MCP79510_rtc_set_time(rtcTime_t* rtcTim);


bool MCP79510_RegisterIOCallBack(MCP79510_IOt* rtcIOCallback);


#endif /* MODULE_COMMON_HW_DRIVERS_HW_PERIPHERALS_RTC_IC_INC_MCP79510T_H_ */
