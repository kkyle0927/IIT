/*
 * risk_mngr.c
 *
 *  Created on: Aug 21, 2023
 *      Author: HyundoKim
 */

#include "risk_mngr.h"

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

FaultTaskQueue faultQueue;
FaultTaskQueue waitingFaultQueue;  // 대기 큐 추가


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

volatile static bool IsFaultTaskQueueBusy = false;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static bool 			FaultTaskQueueIsEmpty(FaultTaskQueue* queue);
static bool 			FaultTaskQueueIsFull (FaultTaskQueue* queue);
static FaultMsgState_t  FaultTaskQueueIn(FaultTaskQueue* queue, uint8_t id, uint8_t cat, uint8_t priority, void (*FaultHandlerFnc)(void*), void* handler_param);
static FaultMsg_t 		FaultTaskQueueOut(FaultTaskQueue* queue);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */



FaultMsgState_t FaultTaskQueueInit(void)
{
	faultQueue.front = 0;
	faultQueue.rear = -1;
	faultQueue.size = 0;

	waitingFaultQueue.front = 0;
	waitingFaultQueue.rear = -1;
	waitingFaultQueue.size = 0;  // 대기 큐 초기화

    return FaultMsgState_OK;
}


FaultMsgState_t FaultTaskCreate(uint8_t cat, uint8_t num, uint8_t priority, void (*handler)(void*), void* handler_param, uint32_t param_size)
{
    void* data_copy = NULL;
    if (param_size > 0) {
        data_copy = malloc(param_size);
        if (data_copy == NULL) {
            return FaultMsgState_ERROR; // 메모리 할당 실패 시 오류 반환
        }
        memcpy(data_copy, handler_param, param_size);
    }

    // Queue가 처리 중일 경우 대기 큐에 추가
    if (IsFaultTaskQueueBusy) {
        if (FaultTaskQueueIsFull(&waitingFaultQueue)) {
            if (data_copy != NULL) {
                free(data_copy); // 대기 큐가 가득 찬 경우 메모리 해제
            }
            return FaultMsgState_BUSY;  // 대기 큐도 가득 차 있는 경우
        }
        return FaultTaskQueueIn(&waitingFaultQueue, num, cat, priority, handler, data_copy);
    }
    // 메인 큐에 추가
    return FaultTaskQueueIn(&faultQueue, num, cat, priority, handler, data_copy);
}


FaultMsgState_t FaultTaskRun(void)
{
    while (!FaultTaskQueueIsEmpty(&faultQueue)) {
        IsFaultTaskQueueBusy = true;        // Queue 처리 시작
        FaultMsg_t faultmsg = FaultTaskQueueOut(&faultQueue);
        // fault 메시지를 처리하는 로직을 여기에 추가
        if (faultmsg.fault_handler != NULL) {
            faultmsg.fault_handler(faultmsg.handler_param); // 함수 포인터 호출
            if (faultmsg.handler_param != NULL) {
                free(faultmsg.handler_param);
            }
        }
    }

    /* Main Queue 초기화 */
	faultQueue.front = 0;
	faultQueue.rear = -1;
	faultQueue.size = 0;

    IsFaultTaskQueueBusy = false;
    // 대기 큐에 있는 모든 항목을 메인 큐로 이동
    while (!FaultTaskQueueIsEmpty(&waitingFaultQueue)) {
        FaultMsg_t faultmsg = FaultTaskQueueOut(&waitingFaultQueue);
        FaultTaskQueueIn(&faultQueue, faultmsg.fault_id, faultmsg.fault_cat, faultmsg.fault_priority,
        		faultmsg.fault_handler, faultmsg.handler_param);
    }

    /* Waiting Queue 초기화 */
	waitingFaultQueue.front = 0;
	waitingFaultQueue.rear = -1;
	waitingFaultQueue.size = 0;

    return FaultMsgState_OK;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static FaultMsgState_t FaultTaskQueueIn(FaultTaskQueue* queue, uint8_t id, uint8_t cat,
		uint8_t priority, void (*FaultHandlerFnc)(void*), void* handler_param)
{
    if (FaultTaskQueueIsFull(queue) || FaultHandlerFnc == NULL) {
        return FaultMsgState_ERROR; // 큐가 가득 찬 경우 오류 반환
    }

    FaultMsg_t faultmsg;
    faultmsg.fault_id = id;
    faultmsg.fault_cat = cat;
    faultmsg.fault_priority = priority;
    faultmsg.fault_handler = FaultHandlerFnc;
    faultmsg.handler_param = handler_param;

    int i;
    // 우선 순위 정렬 : 우선순위에 맞는 위치를 찾기 위해 뒤에서부터 앞으로 이동
    for (i = queue->size - 1; i >= 0; i--) {
        if (queue->buffer[i].fault_priority <= faultmsg.fault_priority) {
            break;
        }
        queue->buffer[i + 1] = queue->buffer[i]; // 정렬 (Priority 가 낮은 순으로)
    }

    queue->buffer[i + 1] = faultmsg; // 새로운 메시지 삽입
    queue->size++;
    queue->rear = (queue->rear + 1) % FAULT_QUEUE_SIZE_MAX;

    return FaultMsgState_OK;
}

static FaultMsg_t FaultTaskQueueOut(FaultTaskQueue* queue)
{
    FaultMsg_t faultmsg = {0, 0, 0, NULL, NULL};

    if (FaultTaskQueueIsEmpty(queue)) {
        return faultmsg; // 큐가 비어있으면 빈 메시지 반환
    }

    faultmsg = queue->buffer[queue->front];
    queue->front = (queue->front + 1) % FAULT_QUEUE_SIZE_MAX;
    queue->size--;

    return faultmsg;
}


static bool FaultTaskQueueIsEmpty(FaultTaskQueue* queue) {
    return (queue->size == 0);
}

static bool FaultTaskQueueIsFull(FaultTaskQueue* queue) {
    return (queue->size == FAULT_QUEUE_SIZE_MAX);
}
