/*
 * AS_system_log.h
 *
 *  Created on: May 14, 2025
 *      Author: Angelrobotics
 */

#ifndef APPS_TASKS_ANGELSUIT_RISKMNGT_HDLR_INC_AS_SYSTEM_LOG_H_
#define APPS_TASKS_ANGELSUIT_RISKMNGT_HDLR_INC_AS_SYSTEM_LOG_H_

#include "stdio.h"
#include "string.h"

#include "ioif_fatfs.h"
#include "ring_buffer.h"

/* for data log */
#include "AS_dev_mngr.h"
#include "AS_whole_body_ctrl.h"
#include "AS_ble_comm_hdlr.h"
#include "ioif_ltc2944.h"


typedef enum _USB_LOG_STATE {
	USB_LOG_OK = 0,
	USB_LOG_EMPTY,
	USB_LOG_FAIL,
} USB_LOG_STATE;



#define LOG_SIZE			100		// max log size
#define LOG_BUFF_SIZE 		LOG_SIZE * 11 * 5			// LOG_SIZE 의 11배(LOG 가 10ms 주기이므로, 10배는 100ms, 11배는 1배의 여유공간을 위한 것, 그리고 버퍼 크기는 실제 저장할 버퍼의 5~6배 크기가 적당.

#define LOG_PREFIX_SIZE		7		// 7 : no. of prefix for time stamp, 'mmddss:'



void 	log_init(void);
//void 	log_push_to_buffer(uint8_t* data);
void 	log_push_to_buffer(const uint8_t* data, size_t length);
int8_t  log_write_to_sdcard(FIL* file_object, uint32_t* writesize);
void 	build_log_string(uint8_t* buffer, uint8_t* log_data, uint8_t hour, uint8_t min, uint8_t sec);
void 	set_syslog_data(uint8_t* log_data);
void 	InitFaultSave(uint8_t* log_data, uint8_t cat, uint8_t id);

/* log printf via USB */
void 	serial_log_init(void);
bool 	push_log_printf(uint8_t* data, ...);			// Variable Arguments
uint8_t print_log_to_USB(void);

#endif /* APPS_TASKS_ANGELSUIT_RISKMNGT_HDLR_INC_AS_SYSTEM_LOG_H_ */
