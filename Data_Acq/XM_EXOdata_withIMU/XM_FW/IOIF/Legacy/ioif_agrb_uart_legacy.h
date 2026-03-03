#ifndef _IOIF_AGRB_UART_EX_H_
#define _IOIF_AGRB_UART_EX_H_

#include "ioif_agrb_defs.h"

#if defined(AGRB_IOIF_UART_ENABLE)

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#if defined(USE_FREERTOS_DMA)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

/**
 * @file ioif_agrb_uart_ex.h
 * @author AngelRobotics FW
 */

#if defined(USE_FREERTOS_DMA)
#define IOIF_UART_TX_DMA_BUFFER_SIZE        (256)   //UART 송신용 DMA 버퍼 크기
#define IOIF_UART_RX_DMA_BUFFER_SIZE        (2048)  //UART 수신용 DMA 버퍼 크기
#define IOIF_UART_TASK_PRIORITY             (osPriorityNormal) //UART 수신 처리 태스크 우선순위

#endif

#define IOIF_UART_MAX_INSTANCES      (3) //Max 16 devices can be handled

#define IOIF_UART_ID_NOT_ALLOCATED  (0xFFFFFFFF)

typedef uint32_t IOIF_UARTx_t;

typedef enum {
  IOIF_UART_Mode_Polling, //주기적으로 쌓인 데이터를 확인하여 알아서 처리할 것임, 별도로 인터럽트 콜백은 발생하지 않음
  IOIF_UART_Mode_EventDriven, //수신완료 이벤트가 발생한 경우, 데이터에 대한 콜백을 수행함

} IOIF_UART_Mode_e;

typedef enum {
    IOIF_UART_Baudrate_9600  ,
    IOIF_UART_Baudrate_19200 ,
    IOIF_UART_Baudrate_38400 ,
    IOIF_UART_Baudrate_57600 ,
    IOIF_UART_Baudrate_115200,
    IOIF_UART_Baudrate_230400,
    IOIF_UART_Baudrate_460800,
    IOIF_UART_Baudrate_921600,

} IOIF_UART_Baudrate_e;

//UART Receive Event Callback function prototype)
//DMA에서 데이터를 수신 완료한 후 호출되는 콜백 함수의 형식 (내부 태스크에서 동작함)
#if defined(USE_FREERTOS_DMA)
typedef void (*IOIF_UART_RxEventCallback_t)(uint8_t* rx_buf, uint32_t size, uint32_t id);
#endif

typedef struct {
    IOIF_UART_Baudrate_e baudrate;
    #if defined(USE_FREERTOS_DMA)
    uint32_t bounce_buffer_size;
    IOIF_UART_RxEventCallback_t rx_event_callback;
    #endif

} IOIF_UART_Config_t;

AGRBStatusDef ioif_uart_assign_instance(IOIF_UARTx_t* id, UART_HandleTypeDef* huart, IOIF_UART_Config_t* config);

AGRBStatusDef ioif_uart_start(IOIF_UARTx_t id);

AGRBStatusDef ioif_uart_update_rx_callback(IOIF_UARTx_t id, IOIF_UART_RxEventCallback_t callback);

AGRBStatusDef ioif_uart_reset_baudrate(IOIF_UARTx_t id, IOIF_UART_Baudrate_e baudrate);

AGRBStatusDef ioif_uart_write(IOIF_UARTx_t id, uint8_t* tx_buf, uint32_t size);
AGRBStatusDef ioif_uart_read(IOIF_UARTx_t id, uint8_t* rx_buf, uint32_t size); //사실 필요없음

AGRBStatusDef ioif_uart_flush(IOIF_UARTx_t id);
AGRBStatusDef ioif_uart_reset(IOIF_UARTx_t id);
AGRBStatusDef ioif_uart_close(IOIF_UARTx_t id);

#endif // AGRB_IOIF_UART_ENABLE
#endif /* _IOIF_AGRB_UART_EX_H_ */
