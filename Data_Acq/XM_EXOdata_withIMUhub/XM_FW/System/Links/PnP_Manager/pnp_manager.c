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
#include "agr_dop_node_id.h"     /* ✅ AGR_NODE_ID_XM */
// #include "agr_pnpmgr.h"  /* AGR PnP Manager (DOP V1 전용) */
#include "cm_xm_link.h"
#include "grf_xm_link.h"
#include "xsens_imu_xm_link.h"
#include "imu_hub_drv.h"          /* ✅ V3.0: IMU Hub Device Driver (System Link 제거) */
#include "canfd_rx_handler.h"    /* PnP V2 Queue 가져오기 */
#include "ioif_agrb_fdcan.h"     /* IOIF_FDCAN_Msg_t */
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
static osThreadId_t PnPManager_TaskHandle;
static const osThreadAttr_t PnPManager_Task_attributes = {
  .name = "PnPManager_Task",
  .stack_size = TASK_STACK_PNP_MANAGER,
  .priority = (osPriority_t) TASK_PRIO_PNP_MANAGER,
};

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

/* ===== Master PnP 인스턴스 (DOP V2 전용) ===== */
static AGR_PnP_Inst_t s_master_pnp;  /**< ✅ XM10 Master PnP (모든 DOP V2 Device 관리) */
static bool s_master_pnp_initialized = false;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void PnPManager_Task(void* argument);
static void PnPManager_RegisterModule(LinkModule_t* module);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS - Master PnP 접근
 *------------------------------------------------------------
 */

AGR_PnP_Inst_t* PnPManager_GetMasterPnP(void)
{
    return s_master_pnp_initialized ? &s_master_pnp : NULL;
}

void PnPManager_RunPeriodic(void)
{
    if (s_master_pnp_initialized) {
        AGR_PnP_RunPeriodic(&s_master_pnp);  /* ✅ 인자 1개만 (inst) */
    }
}

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void PnPManager_Init(AGR_TxFunc_t tx_func)
{
    /* ===== 1. Master PnP 초기화 (DOP V2 전용, XM10 전체) ===== */
    if (!s_master_pnp_initialized) {
        AGR_PnP_Config_t pnp_config = {
            .role = AGR_PNP_ROLE_MASTER,
            .my_node_id = AGR_NODE_ID_XM,
            .tx_func = tx_func,  /* ✅ System Layer에서 제공 */
            .callbacks = {
                /* ✅ Master 공통 콜백 (Device별 콜백은 Device 등록 시 설정) */
                .on_connected = NULL,
                .on_disconnected = NULL,
                .on_bootup = NULL,
                .on_nmt_change = NULL,
                .on_run_pre_op = NULL,
                .on_error = NULL
            }
        };
        AGR_PnP_Init(&s_master_pnp, &pnp_config, HAL_GetTick);
        s_master_pnp_initialized = true;
    }
    
    /* ===== 2. DOP V1 Legacy 모듈 등록 (CM, GRF, XSENS) ===== */
    // 관리할 모든 링크 모듈을 등록합니다 (DOP V1 전용).
    // 새로운 모듈이 추가되면 아래에 한 줄만 추가하면 됩니다.
    PnPManager_RegisterModule(CM_XM_Link_GetModule());           // DOP V1 (Legacy)
    PnPManager_RegisterModule(GRF_XM_Link_GetModule());
    PnPManager_RegisterModule(XsensIMU_XM_Link_GetModule());

    // 등록된 모든 모듈의 Init() 함수를 호출하여 초기화합니다.
    for (int i = 0; i < s_moduleCount; i++) {
        s_linkModules[i]->Init();
    }
    
    /* ✅ V3.0: DOP V2 Device Driver 초기화는 system_startup.c에서 수행 */
    // ImuHub_Drv_Init(&s_master_pnp) - system_startup.c에서 호출
    // EMG_Drv_Init(&s_master_pnp) - 추후 추가
    
    /* ===== 3. PnP Manager 태스크 생성 ===== */
    // 모든 모듈을 주기적으로 관리할 '슈퍼 관리자' 태스크를 생성합니다.
    PnPManager_TaskHandle = osThreadNew(PnPManager_Task, NULL, &PnPManager_Task_attributes);
}

/**
 * @brief SDO Queue에서 호출되는 메시지 라우터 (DOP V1 전용)
 * @details 현재는 CM ↔ XM SDO만 Queue를 사용합니다.
 *          PDO는 ISR에서 직접 처리되므로 Queue를 거치지 않습니다.
 * 
 * [설계 의도]
 * - DOP V1 SDO (0x200): CM ↔ XM 설정 교환 (PnP 단계)
 * - IMU Hub Module과 달리, XM은 여러 Link Module을 관리하므로
 *   향후 확장성을 위해 라우팅 구조 유지
 * 
 * @param canId CAN ID (DOP V1 형식)
 * @param data  수신된 데이터
 * @param len   데이터 길이
 */
void PnPManager_RouteMessage(uint16_t canId, uint8_t* data, uint8_t len)
{
    /* 
     * [DOP V1 전용]
     * - 현재 Queue로 들어오는 메시지: CM SDO만 (0x212, 0x221 등)
     * - Oriented Node (발신자) = Bit 4-7
     * - Destination Node (수신자) = Bit 0-3
     */
    uint8_t oriented_node = (canId & 0x0F0) >> 4;  // Bit 4-7: Oriented (발신자)
    
    /* 발신자 Node ID로 담당 Link Module 찾기 */
    for (int i = 0; i < s_moduleCount; i++) {
        if (s_linkModules[i]->nodeId == oriented_node) {
            /* 담당자를 찾았으면 메시지를 전달 */
            if (s_linkModules[i]->ProcessMessage != NULL) {
                s_linkModules[i]->ProcessMessage(canId, data, len);
            }
            return;
        }
    }
    
    /* 
     * [향후 확장]
     * - DOP V2 SDO도 Queue를 사용하려면:
     *   - Function Code로 DOP V1/V2 구분 (0x200 vs 0x600)
     *   - DOP V2는 Node ID 추출 방식 다름 (msg->id & 0x7F)
     */
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief 등록된 모든 링크 모듈을 주기적으로 관리하는 '슈퍼 관리자' 태스크 함수입니다.
 * @details PnPManager_Init()에서 생성된 태스크로 TAST_PERIOD_MS_PNP_MANAGER주기로 동작합니다.
 * @param[in] argument .
 */
static void PnPManager_Task(void* argument)
{
    /* [주기 작업만 수행, Queue는 _CanFdRxQueueRouter_Task에서 처리] */
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS_PNP_MANAGER);

    for (;;) {
        // 정해진 주기(100ms)에 맞춰 대기합니다.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* ✅ Master PnP 주기 실행 (DOP V2 전용) */
        PnPManager_RunPeriodic();

        // 1. 등록된 모든 모듈의 주기적 관리 함수(RunPeriodic)를 순서대로 호출합니다 (DOP V1).
        for (int i = 0; i < s_moduleCount; i++) {
            if (s_linkModules[i]->RunPeriodic != NULL) {
                s_linkModules[i]->RunPeriodic();
            }
        }
        
        /* ===== 주기 작업 (Timeout 체크, State Machine) ===== */
        /* ✅ V3.0: IMU Hub Device Driver 주기 실행 (Pre-Op Timeout 체크) */
        ImuHub_Drv_RunPeriodic();
        
        /* 참고: Queue 처리는 _CanFdRxQueueRouter_Task에서 이벤트 기반으로 즉시 처리됨 */
    }
}

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
