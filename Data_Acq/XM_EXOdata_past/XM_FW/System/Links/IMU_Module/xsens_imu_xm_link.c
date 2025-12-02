/**
 ******************************************************************************
 * @file    xsens_imu_xm_link.c
 * @author  HyundoKim
 * @brief   [System/Links] XSENS IMU(mti-630) 센서 데이터 링크 (Lock-Free Double Buffering)
 * @details Atomic 연산을 이용한 이중 버퍼링으로 데이터 찢어짐(Tearing)을 방지합니다.
 * @version 0.1
 * @date    Nov 12, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xsens_imu_xm_link.h"
#include "ioif_agrb_tim.h" // IOIF_GetTick()
#include "cmsis_os2.h"
#include <string.h>
#include <stdatomic.h> // C11 원자적 연산

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define IMU_PACKET_TIMEOUT_MS   (500) // 500ms간 패킷이 없으면 연결 끊김 간주

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

// NMT 상태
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

static LinkManagerInst_t s_imu_xm_link; // IMU는 1개이므로 배열이 아님

// [스레드 안전] 핑퐁 버퍼 (이중 버퍼)
// [0] = Read Buffer (UserTask가 읽는 버퍼)
// [1] = Write Buffer (UartRxTask가 쓰는 버퍼)
// 1. 실제 데이터 저장소 (버퍼 2개)
static XsensMTi_packet_t s_imu_buffers[2];
// 2. "현재 최신 데이터가 저장된 인덱스" (0 or 1)
// atomic_int를 사용하여 읽기/쓰기가 단일 명령어로 처리됨을 보장 (Tearing 방지)
static atomic_int s_read_index = 0;

// PnP Manager와 공유할 상태 변수들
static volatile uint32_t s_last_packet_tick = 0; 

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _Link_Init(void);
static void _Link_RunPeriodic(void);
static bool _Link_IsConnected(void); // PnPManager용

/* --- Interface Object --- */
static LinkModule_t s_imu_interface = {
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

/**
 * @brief PnP 매니저에게 등록할 인터페이스 반환
 */
LinkModule_t* XsensIMU_XM_Link_GetModule(void)
{
    return &s_imu_interface;
}

/**
 * @brief 연결 상태 확인
 */
bool XsensIMU_XM_Link_IsConnected(void)
{
    return _Link_IsConnected();
}

/**
 * @brief [Writer] UartRxTask(1kHz)에서 호출
 * @details 인터럽트를 끄지 않고(Critical Section 없음), 반대편 버퍼에 쓰고 인덱스를 스왑합니다.
 */
void XsensIMU_XM_Link_UpdateData(const XsensMTi_packet_t* packet)
{
    if (packet == NULL) return;

    // 1. 현재 Reader(Core Process)가 보고 있는 인덱스 확인
    int current_read_idx = atomic_load(&s_read_index);
    
    // 2. Reader가 안 보는 쪽(반대편)을 쓰기 버퍼로 설정 (0 <-> 1)
    int write_idx = (current_read_idx == 0) ? 1 : 0;
    
    // 3. [Slow Operation] 데이터 복사
    // 인터럽트가 켜져 있으므로 FDCAN/USB 등이 이 도중에 치고 들어올 수 있음 (지연 Zero)
    // Reader는 current_read_idx를 읽고 있으므로 안전함
    memcpy(&s_imu_buffers[write_idx], packet, sizeof(XsensMTi_packet_t));
    
    // 4. [Atomic Operation] 다 썼으니 "이제 최신 데이터는 write_idx야"라고 공표
    // 포인터(인덱스) 교체는 CPU 한 클럭에 끝나므로 Tearing이 발생하지 않음
    atomic_store(&s_read_index, write_idx);

    // 5. [중요] PnP를 위해 생존 신고 (Tick 갱신)
    // 32bit 변수 대입은 Atomic하므로 Lock 불필요
    s_imu_xm_link.lastPacketTimeMs = IOIF_GetTick();
    // 6. [가상 PnP] 패킷이 수신되었으므로 상태와 시간 갱신
    // 상태 갱신 (데이터가 들어오면 Operational)
    s_imu_xm_link.nmtState = NMT_STATE_OPERATIONAL;
}

/**
 * @brief [Reader] Core Process(User Task)에서 호출
 */
bool XsensIMU_XM_Link_GetLatest(XsensMTi_packet_t* data)
{
    if (data == NULL) return false;

    // 연결이 끊겼으면 false 반환
    if (s_imu_xm_link.nmtState != NMT_STATE_OPERATIONAL) {
        return false;
    }

    // 1. 현재 최신 데이터가 있는 인덱스를 확인 (Atomic Load)
    int idx = atomic_load(&s_read_index);

    // 2. 데이터 복사
    // (복사 도중 Writer가 새 데이터를 써도, Writer는 반대편 버퍼에 쓰므로 안전함)
    memcpy(data, &s_imu_buffers[idx], sizeof(XsensMTi_packet_t));

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
    memset(&s_imu_xm_link, 0, sizeof(s_imu_xm_link));
    memset(&s_imu_buffers, 0, sizeof(s_imu_buffers));
    s_imu_xm_link.nmtState = NMT_STATE_STOPPED;
    atomic_store(&s_read_index, 0); // Read 인덱스 0으로 초기화
}

/**
 * @brief PnP 매니저(10ms 주기)가 호출하는 PnP 타임아웃 검사 함수
 */
static void _Link_RunPeriodic(void)
{
    // UartRxTask가 1ms마다 데이터를 갱신하므로,
    // 이 10ms 태스크는 타임아웃 검사만 수행
    if (_Link_IsConnected()) {
        // UartRxTask가 갱신한 lastPacketTimeMs 값을 읽기만 함
        if (IOIF_GetTick() - s_imu_xm_link.lastPacketTimeMs > IMU_PACKET_TIMEOUT_MS) {
            // [가상 PnP] 타임아웃 -> STOPPED 상태로 전환
            s_imu_xm_link.nmtState = NMT_STATE_STOPPED;
        }
    }
}

/**
 * @brief (PnP 매니저용) 연결 상태 확인
 */
static bool _Link_IsConnected(void)
{
    return (s_imu_xm_link.nmtState == NMT_STATE_OPERATIONAL);
}
