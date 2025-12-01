/**
 *-----------------------------------------------------------
 *                MDBT48Q-AT BLE Slave Device
 *-----------------------------------------------------------
 * @file mdbt42q-at.c
 * @date Created on: July 23, 2023
 * @author AngelRobotics HW Team
 * @brief Code for the MDBT48Q-AT BLE Slave Device.
 *
 *
 * Refer to the MDBT48Q-AT datasheet and related documents for more information.
 * @ref MDBT48Q-AT Datasheet
 */

#include "mdbt42q-at.h"

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

static MDBT42QAT_CallbackStruct mdbt42qat_callback;


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

static bool BLE_IO_RegisterCallback(MDBT42QAT_CallbackStruct* BLE_callback);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool MDBT42Q_Init(MDBT42QAT_CallbackStruct* BLE_callback)
{
	bool ret = false;
	ret = BLE_IO_RegisterCallback(BLE_callback);
	return ret;
}

bool MDBT42Q_Deinit(void)
{
	bool ret = false;
	return ret;
}

bool MDBT42Q_AT_ReadCMD(uint8_t* AT_READ_CMD, uint32_t size)
{
	bool ret = false;

	if (mdbt42qat_callback.mdbt42qat_connected() == true) return false; 		//BLE connect 시 AT command 비활성화

	mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_LOW);		//UART PD is low
	mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_50MS);					//UART PD is low 이후 반드시 50ms delay 이후 전송할 것! (이하 시 BLE module 에서 ack 를 제대로 보내지 않을때가 있음!)

	if(mdbt42qat_callback.mdbt42qat_write(&AT_READ_CMD[0], size) == true){
		ret = true;
		mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_250MS);				//at least 250ms before new command. reference from datasheet
		mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_HIGH);
	} else
		ret = false;

	return ret;
}


bool MDBT42Q_AT_WriteCMD(uint8_t *AT_WRITE_CMD, uint32_t size,  SettingSave_Option save_option)
{
	bool ret = false;

	if (mdbt42qat_callback.mdbt42qat_connected() == true) return false; 		//BLE connect 시 AT command 비활성화

	mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_LOW);		//UART PD is low
	mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_100MS);					//UART PD is low 이후 반드시 50ms delay 이후 전송할 것! (이하 시 BLE module 에서 ack 를 제대로 보내지 않을때가 있음!)

	if(mdbt42qat_callback.mdbt42qat_write(AT_WRITE_CMD, size) == true){
		ret = true;
		mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_250MS);

		if(save_option == Save_Active){
			if(mdbt42qat_callback.mdbt42qat_write((uint8_t*)"AT+RESET", 8) == true){
				mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_1000MS);
				mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_HIGH);
			} else
				ret = false;
		} else
			mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_HIGH);

	} else
		ret =  false;

	return ret;

}

bool MDBT42Q_AT_WriteCMDwhenConnected(uint8_t *AT_WRITE_CMD, uint32_t size,  SettingSave_Option save_option)
{
	bool ret = false;

	mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_LOW);			//UART PD low
	mdbt42qat_callback.mdbt42qat_hw_default(MDBT42Q_GPIO_STATUS_LOW);			//flash default low

	mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_100MS);

	if(mdbt42qat_callback.mdbt42qat_write(AT_WRITE_CMD, size) == true){
		ret = true;
		mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_250MS);

		if(save_option == Save_Active){
			if(mdbt42qat_callback.mdbt42qat_write((uint8_t*)"AT+RESET", 8) == true){
				mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_1000MS);
			} else
				ret = false;
		}

	} else
		ret =  false;

	mdbt42qat_callback.mdbt42qat_uart_enable(MDBT42Q_GPIO_STATUS_HIGH);
	mdbt42qat_callback.mdbt42qat_hw_default(MDBT42Q_GPIO_STATUS_HIGH);

	return ret;

}

uint32_t MDBT42Q_ReceivedData(uint8_t uart_ch, uint8_t* AT_data, uint32_t length)
{
	uint32_t ret = 0;
	uint32_t buffer_length = 0;

	buffer_length = mdbt42qat_callback.mdbt42qat_data_length();			//UART RX ring buffer 에 읽어올 데이터가 있는지 확인
	if(buffer_length <= 0)
		return 0;

	for (uint32_t index = 0; index < length; index++ ) {
		AT_data[index] = mdbt42qat_callback.mdbt42qat_read();
	}

//	/* burst read */ //Todo: burst 함수 검증 후 이 함수로 대체
//	mdbt42qat_callback.mdbt42qat_read(AT_data, buffer_length);

	return ret = buffer_length;										//현재 사용가능한 buffer index 를 반환
}

bool MDBT42Q_TransmitData(uint8_t* data, uint32_t length)
{
	return mdbt42qat_callback.mdbt42qat_write(data, length);
}


void MDBT42Q_SetNodeName(uint8_t* name, uint8_t* data)
{
	const char BLE_NAME[] = BLE_SLAVE_WRITE_NAME_COMMAND;

	uint32_t length = sizeof(BLE_SLAVE_WRITE_NAME_COMMAND) - 1;

	if(!name || !data)
		return;

	if(length + strlen((char*)name) > BLE_NAME_LENGTH_MAX)
		return;

	strncpy((char*)data, BLE_NAME, length);						//NodeName : AT+NAME
	strcpy((char*)data+length, (char*)name);					//NodeName : AT+NAME+BLE_SLAVE_NODENAME
}


void MDBT42Q_SetNodeSerial(uint8_t* serialno, uint8_t* data)
{
	const char BLE_SERIALNO[] = BLE_SLAVE_WRITE_SERIALNO;

	uint32_t length = sizeof(BLE_SLAVE_WRITE_SERIALNO) -1;

	if(!serialno || !data)
		return;

	//if(length + strlen((char*)serialno) > BLE_SERIAL_LENGTH_MAX)
	if(length + 8 > BLE_SERIAL_LENGTH_MAX)
		return;

	strncpy((char*)data, BLE_SERIALNO, length);					//NodeName : AT+SERIALNO
	strcpy((char*)data+length, (char*)serialno);					//NodeName : AT+NAME+BLE_SLAVE_SERIAL
}

void MDBT42Q_SetNodeMac(uint8_t* macad, uint8_t* data)
{

	const char BLE_MAC[] = BLE_SLAVE_WRITE_MACADDR;
	char buff[3] = {0};
	char hex_byte[6] = {0};

	uint32_t length = sizeof(BLE_SLAVE_WRITE_MACADDR) - 1;

	if(length + 6 > BLE_MAC_LENGTH_MAX)
		return;

	strncpy((char*)data, BLE_MAC, length);						//NodeName : AT+MACADDR

	/* 문자열 mac address 를 hex 값으로 변환*/
	for(int i = 0; i< 6; i++)
	{
		strncpy(buff, (char*)macad +2 * i, 2);
		buff[2] = '\0';
		hex_byte[i] = (unsigned char) strtol(buff, NULL, 16);

		/* 변환된 6바이트 hex 값을 mac 에 추가*/
		data[i + length] = hex_byte[i];
	}
}

void MDBT42Q_HW_Default(void)
{
		mdbt42qat_callback.mdbt42qat_hw_default(MDBT42Q_GPIO_STATUS_LOW);					//UART PD is low
		mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_600MS);

		mdbt42qat_callback.mdbt42qat_hw_default(MDBT42Q_GPIO_STATUS_HIGH);
		mdbt42qat_callback.mdbt42qat_IOwait(BLE_CMD_WAIT_1000MS);
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static bool BLE_IO_RegisterCallback(MDBT42QAT_CallbackStruct* BLE_callback)	//Callback 함수 등록
{
	bool ret = true;

	if(!BLE_callback->mdbt42qat_data_length || !BLE_callback->mdbt42qat_connected || !BLE_callback->mdbt42qat_uart_enable \
		|| !BLE_callback->mdbt42qat_read || !BLE_callback->mdbt42qat_write || !BLE_callback->mdbt42qat_hw_default /*|| !BLE_callback->mdbt42qat_IOwait*/ ) //디버깅 필요 : IOwait Null 반환
		return false;

	mdbt42qat_callback.mdbt42qat_data_length	= BLE_callback->mdbt42qat_data_length;
	mdbt42qat_callback.mdbt42qat_connected		= BLE_callback->mdbt42qat_connected;
	mdbt42qat_callback.mdbt42qat_read			= BLE_callback->mdbt42qat_read;
	mdbt42qat_callback.mdbt42qat_write			= BLE_callback->mdbt42qat_write;
	mdbt42qat_callback.mdbt42qat_hw_default		= BLE_callback->mdbt42qat_hw_default;
	mdbt42qat_callback.mdbt42qat_uart_enable 	= BLE_callback->mdbt42qat_uart_enable;
	mdbt42qat_callback.mdbt42qat_IOwait			= BLE_callback->mdbt42qat_IOwait;

	return ret;
}
