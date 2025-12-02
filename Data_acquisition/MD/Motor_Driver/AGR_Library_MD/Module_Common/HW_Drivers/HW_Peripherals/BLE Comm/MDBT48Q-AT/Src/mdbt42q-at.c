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

static MDBT42QAT_CallbackStruct mdbt42qt_callback;


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

	mdbt42qt_callback.mdbt42qat_GPIO_PDpin_write(MDBT42Q_GPIO_STATUS_LOW);				//UART PD is low
	osDelay(BLE_CMD_WAIT_50MS);												//UART PD is low 이후 반드시 50ms delay 이후 전송할 것! (이하 시 BLE module 에서 ack 를 제대로 보내지 않을때가 있음!)

	if(mdbt42qt_callback.mdbt42qat_write(&AT_READ_CMD[0], size) == true){
		ret = true;
		osDelay(BLE_CMD_WAIT_250MS);										//at least 250ms before new command. reference from datasheet
		mdbt42qt_callback.mdbt42qat_GPIO_PDpin_write(MDBT42Q_GPIO_STATUS_HIGH);
	} else
		ret = false;

	return ret;
}


bool MDBT42Q_AT_WriteCMD(uint8_t *AT_WRITE_CMD, uint32_t size,  SettingSave_Option save_option)
{
	bool ret = false;

	mdbt42qt_callback.mdbt42qat_GPIO_PDpin_write(MDBT42Q_GPIO_STATUS_LOW);			//UART PD is low
	osDelay(BLE_CMD_WAIT_50MS);											//UART PD is low 이후 반드시 50ms delay 이후 전송할 것! (이하 시 BLE module 에서 ack 를 제대로 보내지 않을때가 있음!)

	if(mdbt42qt_callback.mdbt42qat_write(AT_WRITE_CMD, size) == true){
		ret = true;
		osDelay(BLE_CMD_WAIT_250MS);

		if(save_option == Save_Active){
			if(mdbt42qt_callback.mdbt42qat_write((uint8_t*)BLE_SLAVE_WRITE_RESET, size) == true){
				osDelay(BLE_CMD_WAIT_250MS);
				mdbt42qt_callback.mdbt42qat_GPIO_PDpin_write(MDBT42Q_GPIO_STATUS_HIGH);
			} else
				ret = false;
		} else
			mdbt42qt_callback.mdbt42qat_GPIO_PDpin_write(MDBT42Q_GPIO_STATUS_HIGH);

	} else
		ret =  false;

	return ret;

}

uint32_t MDBT42Q_ReceivedData(uint8_t uart_ch, uint8_t* AT_data, uint32_t length)
{
	uint32_t ret = 0;
	static uint32_t buffer_length = 0;

	buffer_length = mdbt42qt_callback.mdbt42qat_ready();			//UART RX ring buffer 에 읽어올 데이터가 있는지 확인
	if(buffer_length < 0)
		return false;

	for (uint32_t index = 0; index < length; index++ ) {
		AT_data[index] = mdbt42qt_callback.mdbt42qat_read();
	}

	return ret = buffer_length;										//현재 사용가능한 buffer index 를 반환
}

bool MDBT42Q_TransmitData(uint8_t* data, uint32_t length)
{
	bool ret = false;

	ret = mdbt42qt_callback.mdbt42qat_write(data, length);

	return ret;
}


void MDBT42Q_SetNodeName(uint8_t* name, uint8_t* data)
{
	uint32_t length = sizeof(BLE_SLAVE_WRITE_NAME_COMMAND);

	strncpy((char*)data, &BLE_SLAVE_WRITE_NAME_COMMAND[0], length);			//NodeName : AT+NAME
	strcpy((char*)data+(length-1), (char*)name);					//NodeName : AT+NAME+BLE_SLAVE_NODENAME


}


void MDBT42Q_SetNodeSerial(uint8_t* serialno, uint8_t* data)
{
	uint32_t length = sizeof(BLE_SLAVE_WRITE_SERIALNO);

	strncpy((char*)data, &BLE_SLAVE_WRITE_SERIALNO[0], length);		//NodeName : AT+SERIALNO
	strcpy((char*)data+(length-1), (char*)serialno);				//NodeName : AT+NAME+BLE_SLAVE_SERIAL

}

void MDBT42Q_SetNodeMac(uint8_t* macad, uint8_t* data)
{
	uint32_t length = sizeof(BLE_SLAVE_WRITE_MACADDR);

	strncpy((char*)data, &BLE_SLAVE_WRITE_MACADDR[0], length);		//NodeName : AT+SERIALNO
	strcpy((char*)data+(length-1), (char*)macad);				//NodeName : AT+NAME+BLE_SLAVE_SERIAL

}


void MDBT42Q_HW_Default(void)
{
		mdbt42qt_callback.mdbt42qat_HW_default(MDBT42Q_GPIO_STATUS_LOW);					//UART PD is low
		osDelay(BLE_CMD_WAIT_600MS);

		mdbt42qt_callback.mdbt42qat_HW_default(MDBT42Q_GPIO_STATUS_HIGH);
		osDelay(BLE_CMD_WAIT_1000MS);
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

	if(!BLE_callback->mdbt42qat_ready || !BLE_callback->mdbt42qat_baudrate_get || !BLE_callback->mdbt42qat_GPIO_PDpin_write \
			|| !BLE_callback->mdbt42qat_read ||  !BLE_callback->mdbt42qat_baudrate_set || !BLE_callback->mdbt42qat_write || !BLE_callback->mdbt42qat_HW_default )
		return false;

	mdbt42qt_callback.mdbt42qat_ready			= BLE_callback->mdbt42qat_ready;
	mdbt42qt_callback.mdbt42qat_baudrate_get	= BLE_callback->mdbt42qat_baudrate_get;
	mdbt42qt_callback.mdbt42qat_GPIO_PDpin_write= BLE_callback->mdbt42qat_GPIO_PDpin_write;
	mdbt42qt_callback.mdbt42qat_read			= BLE_callback->mdbt42qat_read;
	mdbt42qt_callback.mdbt42qat_baudrate_set	= BLE_callback->mdbt42qat_baudrate_set;
	mdbt42qt_callback.mdbt42qat_write			= BLE_callback->mdbt42qat_write;
	mdbt42qt_callback.mdbt42qat_HW_default		= BLE_callback->mdbt42qat_HW_default;

	return ret;
}

