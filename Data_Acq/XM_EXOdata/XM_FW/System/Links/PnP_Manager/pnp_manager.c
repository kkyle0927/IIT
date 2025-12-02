/**
 ******************************************************************************
 * @file    pnp_manager.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Oct 14, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "pnp_manager.h"
#include "module.h"
#include "cm_xm_link.h"
#include "grf_xm_link.h"
#include "xsens_imu_xm_link.h" // [신규] IMU 링크 모듈
#if defined(USE_FREERTOS_DMA)
#include "cmsis_os.h"
#endif // USE_FREERTOS_DMA

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define MAX_LINK_MODULES         7  // XM에 연결될 모듈 최대 개수 (CM, EMG, FSR, GRF, IMU, FES, 인공근육 모터드라이버-기타..)

/* ------------------- Task Definition ------------------- */
#if defined(USE_FREERTOS_DMA)
static osThreadId_t PnPManager_TaskHandle;
static const osThreadAttr_t PnPManager_Task_attributes = {
  .name = "PnPManager_Task",
  .stack_size = TASK_STACK_PNP_MANAGER,
  .priority = (osPriority_t) TASK_PRIO_PNP_MANAGER,
};
#endif // USE_FREERTOS_DMA

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

static LinkModule_t* s_linkModules[MAX_LINK_MODULES];
static uint8_t s_moduleCount = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void PnPManager_Task(void* argument);
static void PnPManager_RegisterModule(LinkModule_t* module);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void PnPManager_Init(void)
{
    // 1. 관리할 모든 링크 모듈을 등록합니다.
    //    새로운 모듈이 추가되면 아래에 한 줄만 추가하면 됩니다.
    PnPManager_RegisterModule(CM_XM_Link_GetModule());
    PnPManager_RegisterModule(GRF_XM_Link_GetModule());
    PnPManager_RegisterModule(XsensIMU_XM_Link_GetModule()); // [신규] IMU 모듈 등록

     // 2. 등록된 모든 모듈의 Init() 함수를 호출하여 초기화합니다.
    for (int i = 0; i < s_moduleCount; i++) {
        s_linkModules[i]->Init();
    }

#if defined(USE_FREERTOS_DMA)
    // 3. 모든 모듈을 주기적으로 관리할 '슈퍼 관리자' 태스크를 생성합니다.
    PnPManager_TaskHandle = osThreadNew(PnPManager_Task, NULL, &PnPManager_Task_attributes);
#endif // USE_FREERTOS_DMA
}

// CAN 라우터가 메시지를 전달할 함수(PDO 제외 메세지들 처리)
void PnPManager_RouteMessage(uint16_t canId, uint8_t* data, uint8_t len)
{
    // CAN ID에서 발신자 Node ID를 추출합니다.
    uint8_t sourceNodeId = (canId & 0x0F0) >> 4;

    // 메시지를 보낸 Node ID를 담당하는 모듈을 찾아 ProcessMessage 함수 호출
    for (int i = 0; i < s_moduleCount; i++) {
        if (s_linkModules[i]->nodeId == sourceNodeId) {
            // 담당자를 찾았으면 메시지를 전달하고 함수를 종료합니다.
            if (s_linkModules[i]->ProcessMessage != NULL) {
                s_linkModules[i]->ProcessMessage(canId, data, len);
            }
            return;
        }
    }
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

#if defined(USE_FREERTOS_DMA)
/**
 * @brief 등록된 모든 링크 모듈을 주기적으로 관리하는 '슈퍼 관리자' 태스크 함수입니다.
 * @details PnPManager_Init()에서 생성된 태스크로 TAST_PERIOD_MS_PNP_MANAGER주기로 동작합니다.
 * @param[in] argument .
 */
static void PnPManager_Task(void* argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TAST_PERIOD_MS_PNP_MANAGER);

    for (;;) {
        // 등록된 모든 모듈의 주기적 관리 함수(RunPeriodic)를 순서대로 호출합니다.
        for (int i = 0; i < s_moduleCount; i++) {
            if (s_linkModules[i]->RunPeriodic != NULL) {
                s_linkModules[i]->RunPeriodic();
            }
        }

        // 정해진 주기(10ms)에 맞춰 대기합니다.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
#endif // USE_FREERTOS_DMA

/**
 * @brief PnP 매니저에 새로운 링크 모듈을 등록합니다.
 * @details PnPManager_Init() 내부에서 각 링크 모듈을 등록할 때 사용됩니다.
 * @param[in] module 등록할 링크 모듈의 LinkModule_t 구조체 포인터.
 */
static void PnPManager_RegisterModule(LinkModule_t* module)
{
    if (s_moduleCount < MAX_LINK_MODULES) {
        s_linkModules[s_moduleCount++] = module;
    }
}
