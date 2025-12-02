/**
 ******************************************************************************
 * @file    grf_xm_link.c
 * @author  HyundoKim
 * @brief   [System/Links] GRF (FSR) 모듈과의 '가상 PnP' 링크 관리
 * @version 0.1
 * @date    Nov 6, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "grf_xm_link.h"
#include "mdaf-25-6850.h"  // Device Layer
#include "ioif_agrb_tim.h" // IOIF_GetTick()
#include "cmsis_os2.h"
#include <string.h>
#include <stdatomic.h> // C11 원자적 연산을 위해 추가

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define GRF_PACKET_TIMEOUT_MS   (500) // 500ms간 패킷이 없으면 연결 끊김 간주

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

// NMT 상태 (cm_xm_link.h와 동일하게 정의)
typedef enum {
    NMT_STATE_STOPPED,
    NMT_STATE_OPERATIONAL,
} LinkNmtState_t;

// 링크 관리자의 내부 상태 변수를 담는 구조체
typedef struct {
    volatile LinkNmtState_t nmtState;
    volatile uint32_t       lastPacketTimeMs; // 마지막 패킷 수신 시간
} LinkManagerInst_t;

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

static LinkManagerInst_t s_grf_xm_link[MAX_SENSOR_NUM]; // (0 index)1 = 왼발, (1 index)2 = 오른발

// 핑퐁 버퍼 (이중 버퍼)
// [0] = Read Buffer (UserTask가 읽는 버퍼)
// [1] = Write Buffer (UartRxTask가 쓰는 버퍼)
static MarvelDex_packet_t s_grf_buffers_L[2];
static MarvelDex_packet_t s_grf_buffers_R[2];

// "현재 최신 데이터가 저장된 인덱스" (0 or 1)
// atomic_int를 사용하여 읽기/쓰기가 단일 명령어로 처리됨을 보장
static atomic_int s_read_index_L = 0;
static atomic_int s_read_index_R = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _Link_Init(void);
static void _Link_RunPeriodic(void);
static bool _Link_IsConnected(void);

/* --- Interface Object --- */
static LinkModule_t s_grf_interface = {
    .nodeId = 0, // UART는 CAN Node ID가 없음
    .Init = _Link_Init,
    .RunPeriodic = _Link_RunPeriodic,
    .ProcessMessage = NULL, // CAN 미사용
    .IsConnected = _Link_IsConnected
};

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/***
 * @brief PnP 매니저에게 등록할 인터페이스 반환
 */
LinkModule_t* GRF_XM_Link_GetModule(void)
{
    return &s_grf_interface;
}

/**
 * @brief 연결 상태 확인
 */
bool GRF_XM_Link_IsConnected(uint8_t sensorSpace)
{
    if (sensorSpace == SENSOR_SPACE_LEFT) {
        return (s_grf_xm_link[0].nmtState == NMT_STATE_OPERATIONAL);
    } else if (sensorSpace == SENSOR_SPACE_RIGHT) {
        return (s_grf_xm_link[1].nmtState == NMT_STATE_OPERATIONAL);
    }
    return false;
}

// Rx Task가 호출하는 함수 (Writer)
/**
 * @brief [Writer] UartRxTask에서 호출
 * 인터럽트를 끄지 않고, '쓰기용 버퍼'에 쓰고 나서 인덱스만 바꿉니다.
 */
void GRF_XM_Link_UpdateData(const MarvelDex_packet_t* packet)
{
    if (packet == NULL) return;

    int32_t idx = -1;
    if (packet->sensorSpace == SENSOR_SPACE_LEFT) idx = 0;
    else if (packet->sensorSpace == SENSOR_SPACE_RIGHT) idx = 1;
    if (idx == -1) return;

    // [가상 PnP] 패킷이 수신되었으므로 상태와 시간 갱신
    // (이 변수들은 10ms PnP 태스크만 읽으므로 Mutex 불필요)
    s_grf_xm_link[idx].nmtState = NMT_STATE_OPERATIONAL;
    s_grf_xm_link[idx].lastPacketTimeMs = IOIF_GetTick(); // 1ms마다 갱신됨

    int read_idx;
    int write_idx;

    if (packet->sensorSpace == MARVELDEX_SENSOR_SPACE_LEFT) {
        // 1. 현재 Reader가 보고 있는 인덱스 확인
        read_idx = atomic_load(&s_read_index_L);
        
        // 2. Reader가 안 보는 쪽(반대편)을 쓰기 버퍼로 설정
        write_idx = (read_idx == 0) ? 1 : 0;
        
        // 3. [Slow Operation] 데이터 복사 (인터럽트 켜진 상태로 진행!)
        // 이 동안 UserTask가 read_idx(0)를 읽어도 안전함
        memcpy(&s_grf_buffers_L[write_idx], packet, sizeof(MarvelDex_packet_t));
        
        // 4. [Atomic Operation] 다 썼으니 "이제 최신 데이터는 write_idx야"라고 공표
        // 포인터(인덱스) 교체는 CPU 한 클럭에 끝남 -> Tearing 없음
        atomic_store(&s_read_index_L, write_idx);
    } 
    else if (packet->sensorSpace == MARVELDEX_SENSOR_SPACE_RIGHT) {
        read_idx = atomic_load(&s_read_index_R);
        write_idx = (read_idx == 0) ? 1 : 0;
        
        memcpy(&s_grf_buffers_R[write_idx], packet, sizeof(MarvelDex_packet_t));
        atomic_store(&s_read_index_R, write_idx);
    }
}

// Core Process가 호출할 Getter (Reader)
/**
 * @brief [Reader] Core Process에서 호출
 * Lock 없이 최신 인덱스의 데이터를 가져갑니다.
 */
bool GRF_XM_Link_GetLatest(MarvelDex_packet_t* out_L, MarvelDex_packet_t* out_R)
{
    if (out_L) {
        // 1. 현재 최신 데이터가 있는 인덱스를 확인 (Atomic)
        int idx = atomic_load(&s_read_index_L);
        // 2. 데이터 복사
        // (복사 도중 Writer가 새 데이터를 써도, Writer는 반대편 버퍼에 쓰므로 안전함)
        memcpy(out_L, &s_grf_buffers_L[idx], sizeof(MarvelDex_packet_t));
    }
    if (out_R) {
        int idx = atomic_load(&s_read_index_R);
        memcpy(out_R, &s_grf_buffers_R[idx], sizeof(MarvelDex_packet_t));
    }
    return true;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief 링크 모듈 초기화 (PnP 매니저가 호출)
 */
static void _Link_Init(void)
{
    memset(&s_grf_xm_link, 0, sizeof(s_grf_xm_link));
    memset(&s_grf_buffers_L, 0, sizeof(s_grf_buffers_L));
    memset(&s_grf_buffers_R, 0, sizeof(s_grf_buffers_R));
    s_grf_xm_link[0].nmtState = NMT_STATE_STOPPED;
    s_grf_xm_link[1].nmtState = NMT_STATE_STOPPED;
    
    atomic_store(&s_read_index_L, 0);
    atomic_store(&s_read_index_R, 0);
}

/**
 * @brief PnP 매니저(10ms)가 호출하는 PnP 상태 검사 함수
 */
static void _Link_RunPeriodic(void)
{
    // [삭제] 'while (marvelDexFSR.get(&packet))' 루프 전체 삭제
    //       (이 역할은 UartRxTask가 1ms 주기로 수행함)
    
    uint32_t currentTime = IOIF_GetTick();

    // [수정] 타임아웃 검사 로직만 남김
    for (int i = 0; i < MAX_SENSOR_NUM; i++) {
        if (s_grf_xm_link[i].nmtState == NMT_STATE_OPERATIONAL) {
            // UartRxTask가 갱신한 lastPacketTimeMs 값을 읽기만 함
            if (currentTime - s_grf_xm_link[i].lastPacketTimeMs > GRF_PACKET_TIMEOUT_MS) {
                // [가상 PnP] 타임아웃 -> STOPPED 상태로 전환
                s_grf_xm_link[i].nmtState = NMT_STATE_STOPPED;
            }
        }
    }
}

/**
 * @brief (PnP 매니저용) 하나라도 연결되면 true
 */
static bool _Link_IsConnected(void)
{
    // 두 센서 중 하나라도 OPERATIONAL 상태이면 true 반환
    return (s_grf_xm_link[0].nmtState == NMT_STATE_OPERATIONAL) || 
           (s_grf_xm_link[1].nmtState == NMT_STATE_OPERATIONAL);
}
