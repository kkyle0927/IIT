/*
 * AS_system_log.c
 *
 *  Created on: May 14, 2025
 *      Author: Angelrobotics
 */


#include "AS_system_log.h"

/* global variable */
uint8_t test_data_log[2][LOG_BUFF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));		//double buffer
size_t  write_offset[2] = {0, 0};  					// 각 버퍼의 현재 쓰기 위치

/* Serial log buffer */
RingBufferStruct serial_log_ringbuf;
uint8_t serial_log_buffer[1024] = {0,};


volatile uint8_t active_buffer = 0;       			// 현재 쓰고 있는 버퍼
volatile uint8_t buffer_ready_to_write = 0;


uint8_t syslog_hexDataCursor = 0;

/* extern append data list */
//extern osSemaphoreId_t sdio_sync_semaphore;

extern AS_BLE_CtrlObj_t  BLECtrlObj;					// for User ID
extern FSM_Mngr    	  	 FSMMngrObj;					// for FSM state
extern IOIF_BatMonitor_Data_t batData;						// battery data
extern AssistStage 	     assistStage;					// assist level
extern uint8_t 		  	 errorMSGcode;					// error message code
extern uint8_t 		  	 assistmode;					// assist mode
extern uint8_t 		  	 manu_mode;						// manufactoring(with Angel'a TS) mode
extern AS_BLECommState_t isBLEConnect;					// BLE is connected?
extern float 			 RH_IMU_BodyAngle_Sagittal;		// TVCF RH
extern float 			 LH_IMU_BodyAngle_Sagittal;		// TVCF LH
extern AS_CtrlObj_t 	 userCtrlObj[AS_DEV_IDX_MAX];	// SAM10 data
extern float 			 RH_ThighNeutralSagittalAngle;	// Netrual Posture Angle
extern float 			 LH_ThighNeutralSagittalAngle;  // Netrual Posture Angle
extern float 			 RH_PositionAct;				// SAM10 position actual
extern float 			 LH_PositionAct;				// SAM10 position actual
extern float 			 RH_accXGlobal[2];				// IMU acc.x global
extern float 			 RH_accYGlobal[2];				// IMU acc.y global
extern float 			 LH_accXGlobal[2];				// IMU acc.x global
extern float 			 LH_accYGlobal[2];				// IMU acc.y global


/* static function */
static char Num2Str(uint8_t num);
static void Append_SD_Data_Float16(uint8_t* buffer, uint8_t t_size, float t_data, float conversion_constant);
static void Append_SD_Data_Uint8(uint8_t* buffer, uint8_t t_size, uint8_t t_data);
static void Append_SD_Data_Uint16(uint8_t* buffer, uint8_t t_size, uint16_t t_data);
static void Append_SD_Data_Uint32(uint8_t* buffer, uint8_t t_size, uint32_t t_data);


/* function descripted */


void log_init(void)
{
	syslog_hexDataCursor = 0;

	memset(test_data_log[0], 0, LOG_BUFF_SIZE);
	memset(test_data_log[1], 0, LOG_BUFF_SIZE);
}


void log_push_to_buffer(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0) return;

    // 공간이 부족한 경우, 버퍼를 스위칭
    if (write_offset[active_buffer] + length >= LOG_BUFF_SIZE) {
        buffer_ready_to_write = 1;
        active_buffer ^= 1;  // 버퍼 전환

        // 새 버퍼 초기화
        write_offset[active_buffer] = 0;
        memset(test_data_log[active_buffer], 0, LOG_BUFF_SIZE);
    }
    // 안전하게 복사
    memcpy(&test_data_log[active_buffer][write_offset[active_buffer]], data, length);
    write_offset[active_buffer] += length;
}


int8_t log_write_to_sdcard(FIL* file_object, uint32_t* writesize)
{
	if (!buffer_ready_to_write) return 0;

	uint8_t buffer_to_write = active_buffer ^ 1;
	size_t length_to_write = write_offset[buffer_to_write];

	FRESULT res1 = IOIF_fWrite((IOIF_FILE_t*)file_object, test_data_log[buffer_to_write], length_to_write, writesize);

	FRESULT res2 = IOIF_FileSync((IOIF_FILE_t*)file_object);

	if (res1 == FR_OK && res2 == FR_OK) {
		buffer_ready_to_write = 0;
		write_offset[buffer_to_write] = 0;  // 기록 완료 후 리셋
		return 0;
	}
	return -1;		//return error
}



void build_log_string(uint8_t* buffer, uint8_t* log_data, uint8_t hour, uint8_t min, uint8_t sec)
{
    uint8_t* ptr = buffer;

    // 시각 숫자 직접 삽입 (ASCII 변환)
    *ptr++ = '0' + (hour / 10);
    *ptr++ = '0' + (hour % 10);

    *ptr++ = '0' + (min / 10);
    *ptr++ = '0' + (min % 10);

    *ptr++ = '0' + (sec / 10);
    *ptr++ = '0' + (sec % 10);
    *ptr++ = ':';

    // 나머지 문자열 복사
    const char* fixed_str = (const char*)log_data;
    while (*fixed_str && ((ptr - buffer) < LOG_BUFF_SIZE - 1)) {
        *ptr++ = *fixed_str++;
    }

    // 널 종료
    *ptr = '\0';
}

void InitFaultSave(uint8_t* log_data, uint8_t cat, uint8_t id)
{
	/* 0. Clear previous Data, cursor */
	memset(log_data, 0, 20);
	syslog_hexDataCursor = 0;

	Append_SD_Data_Uint8(log_data, CONV_UINT8, cat);
	Append_SD_Data_Uint8(log_data, CONV_UINT8, id);

	/* Including carriage return, line feed */
	log_data[syslog_hexDataCursor++] = '\r';
	log_data[syslog_hexDataCursor++] = '\n';
}


void set_syslog_data(uint8_t* log_data)
{
	/* 0. Clear previous Data, cursor */
	memset(log_data, 0, LOG_SIZE - LOG_PREFIX_SIZE);
	syslog_hexDataCursor = 0;
	/* Append log data */
	/* 0. User ID, Pro User ID */
	//Append_SD_Data_Uint32(log_data, CONV_UINT32, BLECtrlObj.data.UserID);
	Append_SD_Data_Uint32(log_data, CONV_UINT32, BLECtrlObj.data.ProUserID);
	/* 1. Battery Data : Voltage, Current, Temp. */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, batData.batVolt, VOLT_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, batData.batCurr, CURRENT_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, batData.brdTemp, TEMP_CONSTANT);
	/* 2. RM, MD Error Code */
	Append_SD_Data_Uint8(log_data, CONV_UINT8, errorMSGcode);
	/* 3. FSM Current State */
	Append_SD_Data_Uint8(log_data, CONV_UINT8, FSMMngrObj.state_curr);
	/* 4. Assist Level */
	Append_SD_Data_Uint8(log_data, CONV_UINT8, assistStage);
	/* 5. Current assist mode */
	Append_SD_Data_Uint8(log_data, CONV_UINT8, assistmode);
	/* 6. Manufactoring mode */
//	Append_SD_Data_Uint8(log_data, CONV_UINT8, manu_mode);
	/* 7. BLE is connected? */
	Append_SD_Data_Uint8(log_data, CONV_UINT8, isBLEConnect.current_state);
	/* 8. SAM10 Sagittal degree (R/H) */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_Sagittal.ThighSagittalAngle, DEG_ENC_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_Sagittal.ThighSagittalAngle, DEG_ENC_CONSTANT);
	/* 9. SAM10 TVCF degree (R/H) */
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_IMU_BodyAngle_Sagittal, DEG_ENC_RANGE_MAX);
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_IMU_BodyAngle_Sagittal, DEG_ENC_RANGE_MAX);
	/* 10. SAM10 Incremental Encoder Degree (R/H)*/
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, userCtrlObj[2].data.inc_enc_velo/1.875, DEG_ENC_RANGE_MAX);
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, userCtrlObj[3].data.inc_enc_velo/1.875, DEG_ENC_RANGE_MAX);
	/* 11. SAM10 Position Actual */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_Sagittal.PositionAct, DEG_ENC_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_Sagittal.PositionAct, DEG_ENC_CONSTANT);
	/* 12. SAM10 Absolute Encodee Degree (R/H) */
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, userCtrlObj[2].data.abs1_enc_velo/1.875, DEG_ENC_RANGE_MAX);
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, userCtrlObj[3].data.abs1_enc_velo/1.875, DEG_ENC_RANGE_MAX);
	/* 13. SAM10 Neutral Posture Degree (R/H) */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_ThighNeutralSagittalAngle, DEG_ENC_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_ThighNeutralSagittalAngle, DEG_ENC_CONSTANT);
	/* 14. SAM10 Actual Voltage */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_Sagittal.MotorActVoltage, VOLT_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_Sagittal.MotorActVoltage, VOLT_CONSTANT);
	/* 15. SAM10 Actual Current */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_Sagittal.MotorActCurrent, CURRENT_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_Sagittal.MotorActCurrent, CURRENT_CONSTANT);
	/* 16. SAM10 Motor Temp */
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_Sagittal.MotorTemp, TEMP_CONSTANT);
	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_Sagittal.MotorTemp, TEMP_CONSTANT);
	/* 17. SAM10 Acc. X Global */
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_accXGlobal[0], ACC_RANGE_MAX);
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_accXGlobal[0], ACC_RANGE_MAX);
//	/* 18. SAM10 Acc. Y Global */
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, RH_accYGlobal[0], ACC_RANGE_MAX);
//	Append_SD_Data_Float16(log_data, CONV_FLOAT16, LH_accYGlobal[0], ACC_RANGE_MAX);
	/* 19. SAM10 P-Vector Reference */
	/* 20. SAM10 I-Vector Reference */
	/* 21. SAM10 F-Vector Reference */

	/* Including carriage return, line feed */
	log_data[syslog_hexDataCursor++] = '\r';
	log_data[syslog_hexDataCursor++] = '\n';
}


void serial_log_init(void)
{
	RingBufferCreate(&serial_log_ringbuf, &serial_log_buffer[0], 1024);
}


bool push_log_printf(uint8_t* data, ...)			// Variable Arguments
{
	bool ret = false;
	uint8_t msg_buf[256];

	int32_t length;

	va_list args;

	va_start(args, data);

	length = vsnprintf((char*)msg_buf, 256, (char*)data, args);
	ret = RingBufferPush(&serial_log_ringbuf, msg_buf, length);

	va_end(args);

	return ret;
}


uint8_t print_log_to_USB(void)
{
	uint8_t ret = USB_LOG_OK;
	uint8_t temp_buf[512] = {0,};

	/* 1. buffer 가 available 하면 */
	uint32_t length = RingBufferIsAvailable(&serial_log_ringbuf);

	/* 2. length check, 보낼게 없으면 종료 */
	if(length == 0)
		return ret = USB_LOG_EMPTY;

	/* 3. pop from buffer 후 */
	RingBufferPop(&serial_log_ringbuf, &temp_buf[0], length);

	/* 4. print out via USB*/
	if(IOIF_USBD_Write(&temp_buf[0], length, 100) != USBD_OK)
		ret = USB_LOG_FAIL;

	return ret;
}





static void Append_SD_Data_Float16(uint8_t* buffer, uint8_t t_size, float t_data, float conversion_constant) // for float
{
	int16_t conv_data = 0;
	conv_data = (int16_t)(t_data * conversion_constant);

	if (syslog_hexDataCursor + t_size - 1 < LOG_SIZE) {
		buffer[syslog_hexDataCursor++]   = Num2Str((conv_data & 0xF000) >> 12);
		buffer[syslog_hexDataCursor++]   = Num2Str((conv_data & 0x0F00) >> 8);
		buffer[syslog_hexDataCursor++]   = Num2Str((conv_data & 0x00F0) >> 4);
		buffer[syslog_hexDataCursor++]   = Num2Str((conv_data & 0x000F));
	}
}

static void Append_SD_Data_Uint8(uint8_t* buffer, uint8_t t_size, uint8_t t_data)
{
	if (syslog_hexDataCursor + t_size - 1 < LOG_SIZE) {
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0xF0) >> 4);
		buffer[syslog_hexDataCursor++]   = Num2Str(t_data & 0x0F);
	}
}

static void Append_SD_Data_Uint16(uint8_t* buffer, uint8_t t_size, uint16_t t_data) // for uint16
{
	if (syslog_hexDataCursor + t_size - 1 < LOG_SIZE) {
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0xF000) >> 12);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x0F00) >> 8);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x00F0) >> 4);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x000F));
	}
}

static void Append_SD_Data_Uint32(uint8_t* buffer, uint8_t t_size, uint32_t t_data) // for uint32
{
	if (syslog_hexDataCursor + t_size - 1 < LOG_SIZE) {
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0xF0000000) >> 28);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x0F000000) >> 24);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x00F00000) >> 20);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x000F0000) >> 16);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x0000F000) >> 12);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x00000F00) >> 8);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x000000F0) >> 4);
		buffer[syslog_hexDataCursor++]   = Num2Str((t_data & 0x0000000F));
	}
}



static char Num2Str(uint8_t num)
{
	switch (num) {
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	case 10:
		return 'A';
	case 11:
		return 'B';
	case 12:
		return 'C';
	case 13:
		return 'D';
	case 14:
		return 'E';
	case 15:
		return 'F';
	default:
		break;
	}
	return 0;
}
