/*
 * AS_wifi_comm_hdlr.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Angelrobotics
 */


#include "AS_wifi_comm_hdlr.h"


#ifdef SUIT_MINICM_ENABLED

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

TaskObj_t WiFiCommTask;

uint32_t WiFiCommTask_looptime;

/* WIFI TEST */


uint8_t SPI3_DMA_TX_BUFF_TEST[2]  __attribute__((section(".spi3TxBuff"))) = {0,0};

uint8_t esp32_tx_test_flag = 0;
uint8_t spi_dummy_tx[2] = {0,0};
uint16_t send_buffer_size = 256;

extern uint32_t wholeBodyCtrlLoopCnt;

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
 * @brief Function prototypes for this module.
 */
static void StateOff_Ent(void);
static void StateStandby_Ent(void);
static void StateEnable_Ent(void);
static void StateStandby_Run(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);
static void StateError_Run(void);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */


void InitWiFiComm(void)
{
	/* Init Task */
	InitTask(&WiFiCommTask);
	DOPC_AssignTaskID(&WiFiCommTask, TASK_IDX_WIFI_COMM_HDLR);

	memset(SPI3_DMA_RX_BUFF, 0, sizeof(SPI3_DMA_RX_BUFF));

	/* State Definition */
	TASK_CREATE_STATE(&WiFiCommTask, TASK_STATE_OFF,     StateOff_Ent,     NULL,            NULL,               false);
	TASK_CREATE_STATE(&WiFiCommTask, TASK_STATE_STANDBY, StateStandby_Ent, StateStandby_Run,NULL,               true);
	TASK_CREATE_STATE(&WiFiCommTask, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, StateEnable_Ext,    false);
	TASK_CREATE_STATE(&WiFiCommTask, TASK_STATE_ERROR,   NULL,             StateError_Run,  NULL,               false);
}


void RunWiFiComm(void)
{
	RunTask(&WiFiCommTask);
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Static Functions that interface with this module.
 */

static void StateOff_Ent(void)
{
	StateTransition(&WiFiCommTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Ent(void)
{

}

static void StateStandby_Run(void)
{
	if(wholeBodyCtrlLoopCnt > 10000) {
		memset(SPI3_DMA_TX_BUFF, 99, IOIF_ESP32_SPI_TX_BUFF_LENGTH);
		StateTransition(&WiFiCommTask.stateMachine, TASK_STATE_ENABLE);

	}
}

static void StateEnable_Ent(void)
{
	memset(SPI3_DMA_TX_BUFF, 99, IOIF_ESP32_SPI_TX_BUFF_LENGTH);
}

uint32_t esp32_spi_rx_error_cnt = 0;
uint32_t esp32_spi_rx_cnt = 0;
uint32_t esp32_spi_ack_cnt = 0;
uint32_t esp32_spi_rx_success_trial = 0;


IOIF_GPIOPinState_t spi_rdy = IOIF_GPIO_PIN_RESET;
IOIF_GPIOPinState_t TS_mode;
static void StateEnable_Run(void)
{
//	if(esp32_tx_test_flag == 1)
//	{
//		if(ESP32_WIFI_SPI_TX(SPI3_DMA_TX_BUFF, NULL, 2, false) == IOIF_ESP32_SPI_COMM_OK)
//		{
//			while(!esp32_spitx_cplt);
//			ESP32_WIFI_SPI_TX(&spi_dummy_tx[0], SPI3_DMA_RX_BUFF, 2, true);
//		}
//
//		esp32_tx_test_flag = 0;
//	}
//
//	if(esp32_tx_test_flag == 2)
//	{
//		ESP32_WIFI_SPI_TX(SPI3_DMA_TX_BUFF, SPI3_DMA_RX_BUFF, 64, true);
//		esp32_tx_test_flag = 0;
//	}
//
//	if(esp32_tx_test_flag == 3)
//	{
//		ESP32_WIFI_SPI_TX(SPI3_DMA_TX_BUFF, NULL, 2, true);
//		esp32_tx_test_flag = 0;
//	}

	if(SPI_test_sw ==0 ){
		static uint32_t cnt = 1;

		SPI3_DMA_TX_BUFF[0] = (cnt >> 24) & 0xFF;
		SPI3_DMA_TX_BUFF[1] = (cnt >> 16) & 0xFF;
		SPI3_DMA_TX_BUFF[2] = (cnt >> 8) & 0xFF;
		SPI3_DMA_TX_BUFF[3] = cnt & 0xFF;

		SPI3_DMA_TX_BUFF[IOIF_ESP32_SPI_TX_BUFF_LENGTH - 4] = (cnt >> 24) & 0xFF;
		SPI3_DMA_TX_BUFF[IOIF_ESP32_SPI_TX_BUFF_LENGTH - 3] = (cnt >> 16) & 0xFF;
		SPI3_DMA_TX_BUFF[IOIF_ESP32_SPI_TX_BUFF_LENGTH - 2] = (cnt >> 8) & 0xFF;
		SPI3_DMA_TX_BUFF[IOIF_ESP32_SPI_TX_BUFF_LENGTH - 1] = cnt & 0xFF;

		uint8_t test_buff[IOIF_ESP32_SPI_TX_BUFF_LENGTH] = {0,};
		uint32_t esp32_spi_rx_error_count1 = 0;
		//uint8_t test_buff_tx[2] = {0,0};

		/* Wifi TX,RX */
		if(IOIF_ESP32_SPI_TX(SPI3_DMA_TX_BUFF, SPI3_DMA_RX_BUFF, 40, true) == IOIF_ESP32_SPI_COMM_OK)
		{
			while(!esp32_spitxrx_cplt);
			memcpy(test_buff, SPI3_DMA_RX_BUFF, IOIF_ESP32_SPI_RX_BUFF_LENGTH);

			for(uint i = 0; i < IOIF_ESP32_SPI_TX_BUFF_LENGTH; i++)
			{
				if(test_buff[i] != 0x96)
					esp32_spi_rx_error_count1++;
			}
			if(esp32_spi_rx_error_count1 == 0)
				esp32_spi_rx_success_trial++;

			esp32_spi_rx_error_cnt += esp32_spi_rx_error_count1;

			esp32_spi_rx_cnt++;
		}

		cnt++;
		if (cnt == 60001) cnt = 1;

	} else if( SPI_test_sw ==1){

		SPI3_DMA_TX_BUFF[0] = 255;
		SPI3_DMA_TX_BUFF[1] = 0;

		for(int i = 1; i<256; i++){
			SPI3_DMA_TX_BUFF[i+1] = i;
		}
		SPI3_DMA_TX_BUFF[256]=255;
		IOIF_ESP32_SPI_TX(SPI3_DMA_TX_BUFF, SPI3_DMA_RX_BUFF, 260, true);

		if(SPI3_DMA_RX_BUFF[0] == 255 && SPI3_DMA_RX_BUFF[1] == 1){
			for(int i = 1; i<256; i++){
				if(SPI3_DMA_RX_BUFF[i+1]!= 256-i)
					break;
				if(i ==255){
					SPI_test_result = 1;
				}
			}
		}

	}

//
//	/* WiFi Loopback Ack. */
//	memcpy(&SPI3_DMA_TX_BUFF_TEST[0], &test_buff[0], 2);
//
//	if(ESP32_WIFI_SPI_TX(SPI3_DMA_TX_BUFF_TEST, NULL, 2, true) == IOIF_ESP32_SPI_COMM_OK)
//	{
//		while(!esp32_spitx_cplt);
//		esp32_spi_ack_cnt++;
//		memset(SPI3_DMA_TX_BUFF_TEST, 0, 2);
//	}

	WiFiCommTask_looptime++;
}

static void StateEnable_Ext(void)
{

}

static void StateError_Run(void)
{

}



#endif /* SUIT_MINICM_ENABLED */

