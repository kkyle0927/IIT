/**
 * @file risk_mngr.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief 
 * @ref
 */

#ifndef RISK_MNGR_INC_RISK_MNGR_H_
#define RISK_MNGR_INC_RISK_MNGR_H_

#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define FAULT_TASK_BIT	(29U)
#define FAULT_TYPE_BIT		(28U)

#define FAULT_QUEUE_SIZE_MAX	10


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _FaultMsg_t {
	uint8_t  fault_id;
	uint8_t   fault_cat;
	uint8_t   fault_priority;		 		//priority : 0(Urgent), 5(High), 10(Mid), 15(Low)
	void 	 (*fault_handler)(void*); 		// 등록 시 처리할 함수
	void*	  handler_param;
}FaultMsg_t;

typedef struct _FaultTaskQueue {
    FaultMsg_t buffer[FAULT_QUEUE_SIZE_MAX]; // 정적 배열로 큐 저장소 할당
    int front;
    int rear;
    int size; // 현재 큐에 저장된 요소의 개수
} FaultTaskQueue;

typedef enum _FaultMsgState_t {
	FaultMsgState_OK = 0,
	FaultMsgState_ERROR,
	FaultMsgState_BUSY,
} FaultMsgState_t;

typedef enum _FaultPriority {
	FAULT_PRIORITY_REALTIME = 0,
	FAULT_PRIORITY_HIGH		= 5,
	FAULT_PRIORITY_MID 		= 10,
	FAULT_PRIORITY_LOW 		= 15,
}_FaultPriority;

typedef enum _FaultType {
	FaultType_Default = 0,
	FaultType_Fatal,	// 발생 후 system off
	FaultType_Warning	// 치명적이지 않은 오류. 대기상태로 빠지거나 recovery를 기다리는 case
} FaultType;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern FaultTaskQueue faultQueue;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

FaultMsgState_t FaultTaskQueueInit(void);
FaultMsgState_t FaultTaskCreate	  ( uint8_t cat, uint8_t id, uint8_t priority, void (*handler)(void*), void* handler_param, uint32_t param_size);
FaultMsgState_t FaultTaskRun      (void);


#endif /* RISK_MNGR_INC_RISK_MNGR_H_ */
