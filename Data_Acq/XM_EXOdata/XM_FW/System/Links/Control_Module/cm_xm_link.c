/**
 ******************************************************************************
 * @file    cm_xm_link.c
 * @author  HyundoKim
 * @brief   [System/Links] CM <-> XM 링크 관리
 * @version 1.1 (Refactored LED control)
 * @date    Nov 12, 2025
 ******************************************************************************
 */

#include "cm_drv.h"
#include "cm_xm_link.h"
#include "cmsis_os.h"
#include "system_startup.h"
#include "ioif_agrb_tim.h"
#include "led_manager.h" // [신규] LED 매니저 헤더 포함

/**
 *-----------------------------------------------------------
 *              PRIVATE DEFINITIONS AND TYPES
 *-----------------------------------------------------------
 */

// --- PRE_OPERATIONAL 하위 상태 정의 ---
typedef enum {
    PRE_OP_IDLE,
    PRE_OP_WAIT_PDO_MAP_ACK,
    PRE_OP_SEND_SYNC_STATES,    // CM과 동기화를 위한 명령 신호 전송
    PRE_OP_WAIT_SYNC_STATES,    // CM이 상태 SDO를 보내주기를 대기(상태 동기화)
    PRE_OP_WAIT_OPERATIONAL_HB  // 최종 확인(첫 Heartbeat)을 기다리는 상태
} PreOpState_t;

// 링크 관리자의 내부 상태 변수를 담는 구조체
typedef struct {
    volatile LinkNmtState_t     nmtState;
    volatile PreOpState_t       preOpState;
    volatile LinkNmtState_t     lastKnownCmState; // CM의 마지막 상태 저장
    volatile bool               isCmBooted;
    volatile bool               isCmConnected;
    volatile bool               isCmSynced;         // OPERATIONAL 이후, 유효한 PDO 스트림이 시작되었는지 여부
    volatile uint32_t           lastCmHeartbeatMs;
    volatile uint32_t           lastSdoSentMs;
    volatile uint8_t            sdoRetryCount;
    volatile uint32_t           initEntryTime;

    // --- 상태 동기화 플래그 ---
    volatile uint32_t           lastSyncStatesTime;
    volatile bool               isAllStatesSynced;
} LinkManagerInst_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 */
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 */

static LinkManagerInst_t s_cm_xm_link = { .nmtState = NMT_STATE_INITIALISING };

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

// --- Private 콜백 함수 (cm_api로부터 호출될) ---
static void _Link_OnCmBootup(void);
static void _Link_OnCmHeartbeat(uint8_t cmNmtState);
static void _Link_OnSdoResponse(uint8_t dictId, uint8_t sdoId, int8_t isSuccess);
static void _Link_OnCmSyncStates(void);
static void _Link_onSuitModeChanged(uint8_t newMode);
static void _Link_onAssistLevelChanged(uint8_t newLevel);
static void _Link_onPVectorCompletedRH(uint8_t isCompleted);
static void _Link_onPVectorCompletedLH(uint8_t isCompleted);

static void _Link_GoToStopped(void);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief CM-XM 링크 관리자 모듈을 초기화하고 관련 태스크를 생성합니다.
 * @details BSP 계층의 메인 초기화 함수(BSP_Init)에서 한 번만 호출되어야 합니다.
 * 이 함수는 PnP 상태 머신을 관리하는 백그라운드 태스크를 생성합니다.
 */
void CM_XM_Link_Init(void)
{
    memset(&s_cm_xm_link, 0, sizeof(LinkManagerInst_t));
    s_cm_xm_link.nmtState = NMT_STATE_INITIALISING;

    // cm_api 모듈이 SDO 이벤트를 상위 계층(이곳)으로 전달할 때 사용할 콜백 함수들
    CM_EventCallbacks_t api_callbacks = {
        .onCmBootup = _Link_OnCmBootup,
        .onCmHeartbeat = _Link_OnCmHeartbeat,
        .onSdoResponse = _Link_OnSdoResponse,
        .onCmSyncStates = _Link_OnCmSyncStates,
        .onSuitModeChanged = _Link_onSuitModeChanged,
        .onAssistLevelChanged = _Link_onAssistLevelChanged,
        .onPVectorCompletedRH = _Link_onPVectorCompletedRH,
        .onPVectorCompletedLH = _Link_onPVectorCompletedLH,
    };
    
    // Devices 계층의 CM_API 초기화 (의존성 주입)
    CM_Init(System_Fdcan1_Transmit, &api_callbacks, SYS_NODE_ID_XM, SYS_NODE_ID_CM);
}

/**
 * @brief CM-XM 링크의 주기적 상태 관리를 수행합니다.
 * @details PnP 매니저 태스크에 의해 10ms 주기로 호출됩니다.
 * Heartbeat 전송, 타임아웃 검사 등 CM과의 연결 상태를 관리하는
 * 메인 상태 머신 로직을 포함합니다.
 */
void CM_XM_Link_RunPeriodic(void)
{
    // 모든 '현재 시간'은 함수 시작 시 한 번만 읽는 것이 아니라,
    // 각 case 내부에서 '공유 변수'를 읽은 직후에 읽어야 합니다.
    switch (s_cm_xm_link.nmtState) {
        case NMT_STATE_INITIALISING:
            // 이 상태에 처음 진입했다면, 전체 타임아웃을 위한 시작 시간 기록
            static uint32_t last_bootup_sent_ms = 0;
            // 1. 공유 변수(s_cm_xm_link.initEntryTime)를 먼저 읽습니다.
            uint32_t initTime = s_cm_xm_link.initEntryTime;
            // 2. 현재 시간을 마지막에 읽습니다.
            uint32_t initCurrentTime = IOIF_GetTick();

            if (initTime == 0) {
                s_cm_xm_link.initEntryTime = initCurrentTime;
                initTime = initCurrentTime;
            }

            // 1. 1s마다 주기적으로 Boot-up 메시지 전송 (메시지 유실 방지)
            if (initCurrentTime - last_bootup_sent_ms > 1000) {
                CM_SendBootup();
                last_bootup_sent_ms = initCurrentTime;
            }

            // 2. 5초가 지나도 CM의 응답이 없으면 최종 실패 처리
            if (initCurrentTime - s_cm_xm_link.initEntryTime > 5000) {
                _Link_GoToStopped(); // 타임아웃 에러 코드와 함께 정지
                s_cm_xm_link.initEntryTime = 0; // 다음 시도를 위해 초기화
            }
            
            // 참고: CM의 Boot-up을 받으면 콜백 함수에서 상태를 PRE_OP로 바꾸고,
            // s_cm_xm_link.initEntryTime = 0; 으로 리셋해주어야 합니다.
            break;

        case NMT_STATE_PRE_OPERATIONAL:
            // 1. 모든 공유 변수를 먼저 읽습니다.
            uint32_t preOpLastHeartbeatTime = s_cm_xm_link.lastCmHeartbeatMs;
            uint32_t preOplastSdoTime = s_cm_xm_link.lastSdoSentMs;

            // 2. 현재 시간을 마지막에 읽습니다.
            uint32_t preOpCurrentTime = IOIF_GetTick();

            // PRE_OP 중에도 CM이 살아있는지 최소한의 감시는 필요
            if (preOpCurrentTime - preOpLastHeartbeatTime > 5000) {
                _Link_GoToStopped();
                break;
            }

            // PRE_OPERATIONAL 하위 상태 머신 실행
            switch (s_cm_xm_link.preOpState) {
                case PRE_OP_IDLE:
                    CM_SendSetPDOMapping();
                    s_cm_xm_link.lastSdoSentMs = preOpCurrentTime;
                    s_cm_xm_link.sdoRetryCount = 1;
                    s_cm_xm_link.preOpState = PRE_OP_WAIT_PDO_MAP_ACK;
                    break;
                
                case PRE_OP_WAIT_PDO_MAP_ACK:
                    if (preOpCurrentTime - preOplastSdoTime > 1000) { // 1초 타임아웃
                        if (s_cm_xm_link.sdoRetryCount < 3) {
                            s_cm_xm_link.preOpState = PRE_OP_IDLE; // 재전송
                            s_cm_xm_link.sdoRetryCount++;
                        } else {
                            _Link_GoToStopped(); // 3초 실패 시 정지
                        }
                    }
                    break;

                // --- 상태 동기화 대기 로직 ---
                case PRE_OP_SEND_SYNC_STATES:
                    CM_SendSyncStates();
                    s_cm_xm_link.lastSyncStatesTime = preOpCurrentTime;
                    s_cm_xm_link.preOpState = PRE_OP_WAIT_SYNC_STATES;
                case PRE_OP_WAIT_SYNC_STATES:
                    uint32_t preOpLastSyncStatesTime = s_cm_xm_link.lastSyncStatesTime;
                    bool preOpIsAllStatesSynced = s_cm_xm_link.isAllStatesSynced;
                    // 3초 이내에 모든 동기화 SDO가 도착해야 함
                    if (preOpCurrentTime - preOpLastSyncStatesTime > 3000) {
                        _Link_GoToStopped(); // 동기화 타임아웃
                        break;
                    }

                    // 모든 동기화 플래그가 true가 되었는지 확인
                    if (preOpIsAllStatesSynced) {
                        // 1. 동기화 완료! CM에게 OPERATIONAL로 전환하라고 명령
                        CM_SendSetNMTState(NMT_STATE_OPERATIONAL);
                        // 2. 타임아웃 타이머 리셋
                        s_cm_xm_link.lastSdoSentMs = preOpCurrentTime;
                        s_cm_xm_link.lastCmHeartbeatMs = preOpCurrentTime;
                        // 3. 다음 상태로 전환
                        s_cm_xm_link.preOpState = PRE_OP_WAIT_OPERATIONAL_HB;
                    }
                    break;
                
                case PRE_OP_WAIT_OPERATIONAL_HB:
                    if (preOpCurrentTime - preOpLastHeartbeatTime > 3000) { // 3초 이내에 HB가 안오면 실패
                        _Link_GoToStopped();
                    }
                    break;
            }
            break;

        case NMT_STATE_OPERATIONAL:
            // 1. 모든 공유 변수(경쟁 대상)를 먼저 읽습니다.
            uint32_t opLastHeartbeatTime = s_cm_xm_link.lastCmHeartbeatMs;
            LinkNmtState_t opLastCmState = s_cm_xm_link.lastKnownCmState;

            // 2. (static 변수 읽기)
            static uint32_t last_xm_hb_time = 0;
            uint32_t last_xm_hb_time_local = last_xm_hb_time;

            // 3. 현재 시간을 가장 마지막에 읽습니다.
			uint32_t opCurrentTime = IOIF_GetTick();

            // CM Heartbeat 타임아웃 검사
            if ((opCurrentTime - opLastHeartbeatTime > 1000) ||
                (opLastCmState != NMT_STATE_OPERATIONAL)) { //  상태 불일치 감지
                _Link_GoToStopped();
                break;
            }

            // 주기적인 XM Heartbeat 전송
            if (opCurrentTime - last_xm_hb_time_local > 200) {
                CM_SendHeartbeat(s_cm_xm_link.nmtState);
                last_xm_hb_time = opCurrentTime;
            }

            // 데이터 동기화 검사
            if (CM_IsDataReady()) {
                s_cm_xm_link.isCmSynced = true;
            }
            break;

        case NMT_STATE_STOPPED:
            static uint32_t stopped_entry_time = 0;
            static bool is_reset_cmd_sent = false;

            // 1. 현재 시간을 읽습니다. (여기서는 static 변수만 사용하므로 순서 무관)
            uint32_t stopCurrentTime = IOIF_GetTick();

            // 이 상태에 처음 진입했다면, 시작 시간 기록
            if (stopped_entry_time == 0) {
                stopped_entry_time = stopCurrentTime;
                is_reset_cmd_sent = false;
            }

            // 1. 재연결 시도 1초 전, CM에게 리셋 명령을 미리 보낸다
            if (!is_reset_cmd_sent && (stopCurrentTime - stopped_entry_time > 2000)) {
                CM_SendSetNMTState(NMT_STATE_STOPPED); // 또는 NMT_CMD_RESET_COMMUNICATION

                is_reset_cmd_sent = true;
            }

            // 2. 총 3초 후, 재연결을 시작한다
            if (stopCurrentTime - stopped_entry_time > 3000) {
                s_cm_xm_link.nmtState = NMT_STATE_INITIALISING;
                stopped_entry_time = 0; // 다음을 위해 초기화
            }
            break;
    }

    /**
     * 10ms마다 현재 NMT 상태를 LED 매니저에게 전달합니다.
     * NMT 상태가 변경되었을 때만 GPIO를 1회 조작합니다.
     */
    LedManager_SetLinkState(s_cm_xm_link.nmtState);
}

/**
 * @brief CM-XM 링크 모듈의 LinkModule_t 인터페이스를 반환합니다.
 * @details PnP 매니저가 이 모듈을 등록하고 제어할 수 있도록,
 * 자신의 정보와 함수 포인터들이 담긴 구조체를 외부에 제공합니다.
 * @return 이 모듈의 LinkModule_t 구조체 포인터.
 */
LinkModule_t* CM_XM_Link_GetModule(void)
{
    static LinkModule_t module = {
        .nodeId = SYS_NODE_ID_CM,
        .Init = CM_XM_Link_Init,
        .RunPeriodic = CM_XM_Link_RunPeriodic,
        .ProcessMessage = CM_XM_Link_ProcessMessage,
        .IsConnected = CM_XM_Link_IsConnected
    };
    return &module;
}

/**
 * @brief 이 링크가 담당하는 CAN 메시지를 처리합니다.
 * @details CAN 라우터에서 CM으로부터 온 메시지일 경우 호출됩니다.
 * 수신된 메시지를 하위 cm_api 모듈로 전달하는 역할을 합니다.
 * @param[in] canId   수신된 메시지의 CAN ID.
 * @param[in] data    수신된 데이터 버퍼 포인터.
 * @param[in] len     수신된 데이터의 길이.
 */
void CM_XM_Link_ProcessMessage(uint16_t canId, uint8_t* data, uint8_t len)
{
    // 이 링크는 CM(Node 1)으로부터 온 메시지만 처리
    if ((canId & 0x0F0) >> 4 == SYS_NODE_ID_CM) {
        CM_ProcessCANMessage(canId, data, len);
    }
}

/**
 * @brief CM과의 PnP 통신 및 데이터 스트림 동기화가 모두 완료되었는지 반환합니다.
 * @return 모든 연결 및 동기화 절차가 완료되었으면 `true`, 그렇지 않으면 `false`.
 */
bool CM_XM_Link_IsConnected(void)
{
    // isCmConnected는 PnP 연결의 성공을,
    // isCmSynced는 그 이후의 데이터 스트림 안정화를 의미합니다.
    return s_cm_xm_link.isCmConnected && s_cm_xm_link.isCmSynced;
}

/**
 * @brief 현재 XM의 PnP 네트워크 관리 상태(NMT)를 반환합니다.
 * @return 현재 XM NMT 상태 값 (LinkNmtState_t).
 */
LinkNmtState_t CM_XM_Link_GetXMNmtState(void)
{
    return s_cm_xm_link.nmtState;
}

/**
 *------------------------------------------------------------
 *                      PRIVATE FUNCTIONS
 *------------------------------------------------------------
 */

static void _Link_GoToStopped(void)
{
    // TODO: EMCY 메시지 전송 Risk Manangement 필요
    s_cm_xm_link.isCmBooted = false;
    s_cm_xm_link.isCmConnected = false;
    s_cm_xm_link.isCmSynced = false;
    s_cm_xm_link.isAllStatesSynced = false;
    s_cm_xm_link.nmtState = NMT_STATE_STOPPED;
    // TODO: 음성 안내 등 안전 관련 로직 추가 필요
    // 데이터 캐시를 리셋하여 오래된(Stale) 데이터가 사용되는 것을 방지합니다.
    CM_ResetRxData();
}

// --- SDO 콜백 함수 ---
static void _Link_OnCmBootup(void)
{
    if (s_cm_xm_link.nmtState == NMT_STATE_INITIALISING) {
        s_cm_xm_link.isCmBooted = true;
        s_cm_xm_link.lastCmHeartbeatMs = IOIF_GetTick();
        s_cm_xm_link.nmtState = NMT_STATE_PRE_OPERATIONAL;
        s_cm_xm_link.preOpState = PRE_OP_IDLE; // 이제 PDO 설정을 시작
        // INITIALISING 상태를 성공적으로 벗어났으므로, 타임아웃 타이머를 리셋
        s_cm_xm_link.initEntryTime = 0;
    }
}

static void _Link_OnCmHeartbeat(uint8_t cmNmtState)
{
    s_cm_xm_link.lastCmHeartbeatMs = IOIF_GetTick();
    s_cm_xm_link.lastKnownCmState = (LinkNmtState_t)cmNmtState; // CM의 현재 상태를 저장

    if (s_cm_xm_link.nmtState == NMT_STATE_PRE_OPERATIONAL && s_cm_xm_link.preOpState == PRE_OP_WAIT_OPERATIONAL_HB) {
        // NMT 명령 후 첫 Heartbeat를 받으면 최종적으로 OPERATIONAL 상태로 전환
        s_cm_xm_link.isCmConnected = true;
        s_cm_xm_link.nmtState = NMT_STATE_OPERATIONAL;
    } else if (s_cm_xm_link.nmtState == NMT_STATE_INITIALISING) {
        // [예외 처리] 초기화 중인데 CM이 Heartbeat을 보낸다는 것은 CM이 과거 상태에 갇혔다는 의미.
        // 연결을 리셋하기 위해 STOPPED 상태로 전환.
        _Link_GoToStopped();
    }
}

static void _Link_OnSdoResponse(uint8_t dictId, uint8_t sdoId, int8_t isSuccess)
{
    // CM으로부터 온 비동기적 요청 처리
    if (dictId == CM_DICT_ID_CM_TO_XM) {
        // 필요시 작성
    }
    // SCM이 보낸 요청에 대한 동기적 응답 처리
    else if (dictId == CM_DICT_ID_XM_TO_CM) {
        if (s_cm_xm_link.nmtState == NMT_STATE_PRE_OPERATIONAL) {
            if (s_cm_xm_link.preOpState == PRE_OP_WAIT_PDO_MAP_ACK && sdoId == CM_SDO_ID_X2C_SET_PDO_MAPPING) {
                if (isSuccess == DOP_SDO_SUCC || isSuccess == DOP_SDO_REQU) { // TODO : CM에서 Req를 SUCC로 변경하고 보내야함
                    // PDO 매핑 응답 성공 -> NMT 상태 변경 명령 전송
                    // CM_SendSetNMTState(NMT_STATE_OPERATIONAL);
                    s_cm_xm_link.lastSdoSentMs = IOIF_GetTick();
                    s_cm_xm_link.lastCmHeartbeatMs = IOIF_GetTick();
                    // s_cm_xm_link.preOpState = PRE_OP_WAIT_OPERATIONAL_HB;
                    s_cm_xm_link.preOpState = PRE_OP_SEND_SYNC_STATES;
                    // 동기화 플래그 리셋
                    s_cm_xm_link.isAllStatesSynced = false;
                } else {
                    _Link_GoToStopped();
                }
            }
        }
    }
}

static void _Link_OnCmSyncStates(void)
{
    s_cm_xm_link.isAllStatesSynced = true;
}

static void _Link_onSuitModeChanged(uint8_t newMode)
{
    CM_UpdateSuitMode((CM_SuitMode_t)newMode);
}

static void _Link_onAssistLevelChanged(uint8_t newLevel)
{
    CM_UpdateAssistLevel(newLevel);
}

static void _Link_onPVectorCompletedRH(uint8_t isCompleted)
{
    CM_UpdatePVectorCompletedRH(isCompleted);
}

static void _Link_onPVectorCompletedLH(uint8_t isCompleted)
{
    CM_UpdatePVectorCompletedLH(isCompleted);
}
