/**
 ******************************************************************************
 * @file    data_logger.c
 * @author  HyundoKim
 * @brief   [System Layer] 저순위 USB MSC 로깅 태스크 (Consumer) 구현
 * @details
 * [Rev2.0 Phase 1 — ACTIVE] Lock-Free SPSC Ring Buffer (D2 128KB Hot Buffer · Hot-only)
 * - UserTask(Producer): D2 Hot Buffer에 memcpy + atomic_store(head) — 비차단
 * - DataLoggerTask(100ms, Prio16): Hot Buffer 직접 read → f_write USB
 *
 * [Rev2.0 Phase 2 — DEPRECATED 2026-04-18 C안] PSRAM Cold Buffer 제거됨
 * - 사유: 복잡도 ↑ vs GC 마진 이득 미미. Rev1.1 방식 (Hot-only) 로 회귀 결정.
 * - 이전 안: PSRAM 4MB Cold + OffloadTask(20ms, Prio32) Dual-Buffer
 * - 현재 안: Hot-only · GC 마진 0.85초 (Phase 1 그대로)
 * @version 0.1
 * @date    Nov 10, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "data_logger.h"
#include "usb_mode_handler.h" // USB 상태 플래그(g_isUsbHostReady) 사용
#include "ioif_agrb_fs.h"     // IOIF 파일 시스템 API
/* ioif_agrb_dwt.h 제거: f_sync DWT 측정 불필요 (Pre-allocation 전환) */
#include <stdio.h>
#include <string.h>
#include <stdatomic.h> // [신규] 원자적 연산 사용
/* [2026-04-18 C안] PSRAM Cold Buffer 제거. Rev1.1 방식 Hot-only 아키텍처로 전환. */
#include "xm_api_rtc.h"       // [Phase 3b] RTC → Cached FatTime
#include "stm32h7xx_hal_crc.h" // [Phase 3d] HW CRC32 블록 무결성
#include "usbh_diskio.h"       // [Diag 2026-04-17] USBH_DiskIO_GetDiag/ResetDiag
#include "cm_drv.h"            // [Diag 2026-04-17] CM_GetPdoDiag/ResetPdoDiag
#include "ioif_agrb_fdcan.h"  // [2026-06-24] FDCAN 세션 통계 IOIF 캡슐화 (GetBusStatus/RxFifo0/ErrorCounters)
#include "system_startup.h"   // [2026-06-24] System_GetFDCAN1_Id (IOIF 인스턴스 핸들)
#include "diag_perf.h"           // [Step 0 2026-04-18] UserTask jitter histogram + kill-switch
#include "ioif_agrb_dwt.h"       // [2026-04-18] OffloadTask stall timing (DWT cycles)
#include "hardfault_dump.h"      // [Step B 재설계 #1] 부팅 시 pending fault dump → /LOGS/hardfault_*.txt

/* [2026-06-24] hfdcan1 extern 제거 — FDCAN 세션 통계는 IOIF(System_GetFDCAN1_Id) 경유. */

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

// --- 1. RAM 링 버퍼 (댐) 설정 ---
// [2026-04-18 C안] D2 SRAM 256KB Hot-only Buffer (Cold Buffer 제거, Rev1.1 방식)
// Producer: UserTask(1kHz, 122B/sample) = 122KB/s
// Consumer: DataLoggerTask(100ms, time-bounded drain) — 직접 Hot → USB
// Hot margin: 256KB / 122KB/s ≈ 2.1초
// 관측 USB burst worst = 716ms (145115 세션) ≪ 2.1초 margin
#define LOG_BUFFER_SIZE         (256 * 1024)

// --- 2. 로깅 태스크 주기 ---
/* [Revert 2026-04-17] 50→100ms 원복: 16KB/50ms 실측 결과 pdo_gap 오히려 증가 (+29%).
 *   USB GC가 chunk size/주기와 무관하게 tight spin을 지배하는 것으로 확인.
 *
 * [후보2 2026-04-18] 100 → 500ms 실험: 반대 방향 (짧게 실패 → 길게 시도).
 *   Producer: 122KB/s × 500ms = 61KB/drain (Hot 256KB 중 24%, margin 충분)
 *   기대: drain 1회에 USB burst 집중 → 나머지 450ms 는 UserTask 무방해 시간
 *        → UserTask 1ms 주기 놓치는 확률 감소 → pdo_gap 감소
 *   리스크: burst 길이 자체는 증가 (하지만 UserTask 가 그 동안 깨어있지 않아도 됨
 *         — drain 끝난 후 한참 조용). drain budget = period / 2 = 250ms.
 *
 * [옵션A 2026-04-19] 100/200/300/500 전 구간 비교 결과:
 *   - 100ms: ratio 2.07, 3ms+/s 0.29, burst 70-130ms, hot 6%
 *   - 200ms: ratio 1.00, 3ms+/s 0.36, burst 94-781ms, hot 10-28%
 *   - 300ms: ratio 1.00, 3ms+/s 0.39, burst 165-522ms, hot 14%
 *   - 500ms: ratio 1.00, 3ms+/s 1.02, burst 264ms, hot 20%
 *
 * 재분석 결론:
 *   - SCSI cmd/s 총 빈도 37-48 범위 (period 무관 saturation)
 *   - disk_write ratio 1.00 은 지표 효과일 뿐 실제 CPU/대역폭 부담 동등
 *   - burst_max 는 GC spike 에 지배 — drain budget 제한 불가
 *   → 100ms 복귀가 가장 합리적 (변경 최소 + critical gap 최저 + hot 여유 최대).
 *   남은 개선은 SDMMC 전환 또는 USBH_MSC event-driven 재도전 영역. */
#define LOGGER_TASK_PERIOD_MS   (100)

// --- 3. 링 버퍼 위험 수위 (에러 감지용) ---
#define BUFFER_WARNING_THRESHOLD_PERCENT (90) // 90%
#define BUFFER_STOP_THRESHOLD_PERCENT    (98) // 98% (오버플로우 직전)

// --- 4. 배치 쓰기(f_write) 단위 ---
/* [Revert 2026-04-17] 16→32KB 원복: 실측 결과 chunk 축소 효과 미미 (max 118ms 유지).
 * USB GC가 tight spin 시간을 지배. 32KB가 FAT32 32KB cluster와 1:1 매칭이라 더 안정적. */
#define MAX_WRITE_CHUNK_SIZE    (32 * 1024)

// 2단계 큐 (API -> DataLoggerTask)
#define CMD_QUEUE_SIZE          (5)     // Start/Stop 명령 5개 버퍼링
#define MAX_METADATA_SIZE       (2048)

// 세션 디렉터리 경로 최대 크기 (LOG_DIR_PATH + "/" + sessionName)
#define MAX_PATH_SIZE           (512)
// 파일명까지 포함한 전체 경로 버퍼 크기 (세션 경로 + "/" + 파일명)
#define MAX_FULL_PATH_SIZE      (MAX_PATH_SIZE + 64)
// End-User가 입력할 세션 이름의 길이를 100자로 제한
#define MAX_SESSION_NAME_SIZE   (100)

#define LOG_FILE_ROLLING_SIZE_MB (10)
#define LOG_DIR_PATH            "/LOGS"
#define WRITE_RETRY_MAX         (2)

/* [Phase 3d] CRC32 블록 무결성: 데이터 블록마다 4-byte CRC32 append.
 * 손상 시 해당 블록만 스킵하고 나머지 복구 가능 (partial recovery).
 *
 * [Step D 2026-04-18] Sector-aligned block (format_version = 2):
 *   - On-disk block = [data CRC_BLOCK_DATA_SIZE B][CRC 4 B] = CRC_BLOCK_TOTAL_SIZE B
 *   - 4096B = 정확히 8 sectors → FatFs partial-sector tail (fp->buf) 제거
 *   - 실측 (S1, 4100B): disk_write ratio = 2.27, Step D 에선 ~1.0 예상
 *
 * 이전 (format_version = 1): data 4096B + CRC 4B = 4100B, 4B 가 fp->buf 경유. */
#define CRC_BLOCK_TOTAL_SIZE    (4096U)
#define CRC_BLOCK_DATA_SIZE     (CRC_BLOCK_TOTAL_SIZE - sizeof(uint32_t))  /* 4092 */

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

// [신규] 링 버퍼에 저장될 패킷 헤더
typedef struct {
    uint32_t packetSize; // 뒤따라오는 실제 데이터의 크기
} LogPacketHeader_t;

// 2단계 큐 (명령 큐) - Start/Stop은 순서 보장이 필요하므로 큐 사용
/**
 * @brief 2단계 큐 (명령 큐)가 사용할 패킷
 */
typedef enum {
    LOG_CMD_START_SESSION,
    LOG_CMD_STOP_SESSION,
} LoggerCommand_e;

/* ===== [Phase 3a] 에러 복구 강화 — 통합 정리 Reason ===== */
/**
 * @brief 로깅 중단 사유 (정상 종료 포함)
 * @details _EmergencyStop(), _WriteSummary_WithStatus()에서 사용.
 *          summary.txt의 status= / error_reason= 필드로 기록됨.
 */
typedef enum {
    STOP_REASON_NORMAL,          /**< 정상 종료 (DataLogger_Stop) */
    STOP_REASON_HOT_OVERFLOW,    /**< Hot Buffer 오버플로우 (UserTask 컨텍스트) */
    STOP_REASON_USB_DISCONNECT,  /**< USB 매체 분리 */
    STOP_REASON_COLD_OVERFLOW,   /**< Cold Buffer 98% 초과 */
    STOP_REASON_WRITE_FAILURE,   /**< f_write 재시도 모두 실패 */
    STOP_REASON_ROLLING_FAILURE, /**< 파일 롤링(새 파트 생성) 실패 */
} StopReason_e;

// 2단계 큐 (명령 큐)
typedef struct {
    LoggerCommand_e command;
    // 세션 이름 버퍼 크기를 명확히 제한
    char sessionName[MAX_SESSION_NAME_SIZE + 1];
    char metadata[MAX_METADATA_SIZE];
} LoggerCommand_t;

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

// data_logger.h가 아닌 usb_mode_handler.h에서 선언됨
extern volatile bool g_isUsbHostReady;
// data_logger.h에서 선언됨
QueueHandle_t g_logCmdQueue; // 2단계 큐 (LoggerCommand_t)

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// 4. 저순위(Prio 16) 로깅 태스크 생성
static osThreadId_t s_loggerTaskHandle;
const osThreadAttr_t s_loggerTask_attributes = {
  .name = "DataLoggerTask",
  .stack_size = TASK_STACK_USB_SAVE, // 4096 (from module.h)
  .priority = (osPriority_t) TASK_PRIO_USB_SAVE, // BelowNormal (16)
};

// --- [Rev2.0 Phase 1] Lock-Free SPSC Hot Ring Buffer ---
// D2 SRAM 128KB Hot Buffer (CPU direct R/W, Zero-Wait-State DMA 접근 가능)
// Phase 2에서 PSRAM 4MB Cold Buffer + Offload Task로 GC 마진 확대 예정
__attribute__((section(".RAM_D2_data")))
static uint8_t s_logBuffer[LOG_BUFFER_SIZE];

// Head: UserTask(Producer)가 씀. Atomic 필수.
static volatile atomic_uint s_logHead = 0; 
// Tail: [Phase 2] OffloadTask(Consumer)가 씀. DataLoggerTask는 Cold Buffer에서 읽음.
// OffloadTask(32)만 쓰고, UserTask(54)가 읽음 (free space 계산). ARM 32-bit write = atomic.
static volatile uint32_t s_logTail = 0;

// --- 상태 변수 ---
static volatile atomic_bool s_isLoggingActive = false;
static volatile atomic_uint s_logStatus = LOG_STATUS_IDLE;

/* [Phase 3a] Emergency Stop: UserTask(Prio 54)에서 FATFS 접근 금지 → flag만 설정.
 * DataLoggerTask(Prio 16) 다음 사이클에서 실제 cleanup 수행. */
static volatile StopReason_e s_pending_stop_reason = STOP_REASON_NORMAL;

/* [2026-04-18] 세션 재시작 체인 추적: 이전 세션 종료 사유를 현재 세션 summary 에 기록.
 * Emergency stop 후 app 이 같은 폴더로 재시작 시 session_index 가 증가 → 이전 세션의
 * 실패 사유가 현재 summary 에 drop 되던 문제 해소. _WriteSummary_WithStatus 마지막에
 * 현재 reason 으로 갱신. 부트 직후 최초 세션은 STOP_REASON_NORMAL ("NONE") 로 표기. */
static StopReason_e s_prev_session_stop_reason = STOP_REASON_NORMAL;

// --- 로깅 통계 ---
static DataLogger_Stats_t s_stats = {0};
/* s_fsync_last_us / s_fsync_max_us 제거: Pre-allocation으로 로깅 중 f_sync 0회 */

/* [Diag 2026-04-17] FDCAN 세션 trace (OffloadTask 20ms 주기로 갱신)
 * - fill_max: RxFIFO0 최대 채움 (overflow 근접 여부)
 * - bus_off_events: BusOff false→true 전이 카운트
 * - was_bus_off: 이전 상태 기억 (전이 감지용)
 */
static volatile uint8_t  s_fdcan_fill_max       = 0;
static volatile uint32_t s_fdcan_bus_off_events = 0;
static volatile bool     s_fdcan_was_bus_off    = false;

// --- 편의 기능 설정 ---
static volatile bool s_auto_timestamp_enabled = true;
static uint32_t s_rolling_size_mb = LOG_FILE_ROLLING_SIZE_MB;
static uint32_t s_user_packet_size = 0;

/* [Phase 3b M1] RTC Cached FatTime: 10초 주기로 RTC 읽어 캐시.
 * get_fattime()은 이 캐시값 즉시 반환 (<1us, SPI 호출 없음). */
static volatile uint32_t s_cached_fattime = 0;
/* 10초 주기 RTC 캐시 — LOGGER_TASK_PERIOD_MS 변경 시 자동 보정 */
#define RTC_SYNC_INTERVAL_CYCLES   (10000U / LOGGER_TASK_PERIOD_MS)
/* FSYNC_INTERVAL_CYCLES 제거: Pre-allocation 전환으로 로깅 중 f_sync 0회.
 * 세션 시작 시 f_lseek(rolling_size) + f_sync 1회만 수행하여
 * FAT+디렉토리 확정 → data gap 0 + 비정상 종료 시 파일 복구 가능. */

static IOIF_FILEx_t s_currentFileId;
static char s_currentSessionPath[MAX_PATH_SIZE];
/* [Option A 2026-04-17] 세션 폴더 이름 (LOG_DIR_PATH 제외한 basename).
 * Emergency Stop 후 재시작 시 예제가 이 이름을 조회해 같은 폴더에 이어쓰기 용. */
static char s_currentSessionName[MAX_SESSION_NAME_SIZE + 1] = {0};
// 세션 인덱스 (data_XXX)
static uint32_t s_currentSessionIndex = 0;
// 파일 분할 인덱스 (part_YYY)
static uint32_t s_currentSplitIndex = 0;

/* [Phase 3b] 파트 파일별 레코드/바이트 추적 (풋터 기록용) */
static uint32_t s_part_record_count = 0;
static uint32_t s_part_data_bytes = 0;

/* [Q6] 파일 open 상태 추적: double-close 방지 (롤링 실패 시 close 중복 호출 방어) */
static bool s_file_open = false;

/* [Phase 3b M5] 세션 시작 RTC 시각 (summary.txt rtc_start/rtc_end 기록용) */
static XmDateTime_t s_session_rtc_start = {0};


/* [Phase 3c] 디스크 모니터링 캐시 (10초 주기 갱신) */
/* 10초 주기 디스크 용량 체크 — LOGGER_TASK_PERIOD_MS 변경 시 자동 보정 */
#define DISK_CHECK_INTERVAL_CYCLES  (10000U / LOGGER_TASK_PERIOD_MS)
#define DISK_LOW_THRESHOLD_MB       (50)
static uint32_t s_cached_disk_free_mb = 0;
static uint32_t s_cached_disk_total_mb = 0;

/* [Phase 3d] CRC32 블록 상태: 현재 불완전 블록의 바이트 수 + HW CRC 핸들.
 * _WriteFileHeader()에서 리셋, _ProcessColdBuffer()에서 블록 경계 추적,
 * 파일 close 전 _FlushPartialBlockCRC()로 마지막 불완전 블록 CRC flush. */
extern CRC_HandleTypeDef hcrc;
static uint32_t s_crc_block_offset = 0;

/* [2026-04-18 C안] OffloadTask + PSRAM Cold Buffer + QSPI Mutex 전부 제거.
 * DataLoggerTask 가 Hot Buffer 를 직접 read → USB write (Rev1.1 Hot-only 패턴). */

/* [Phase 2 Fix] 32-byte 정렬 필수: SCB_CleanDCache_by_Addr 가 cache line 경계에서 동작.
 * s_read_buf: Hot → USB 스테이징 (D1, 32KB, DMA 전 D-Cache Clean 필요). */
__attribute__((aligned(32)))
static uint8_t s_read_buf[MAX_WRITE_CHUNK_SIZE];

/* [Step B v4 S1 + Step D — sector-aligned batching]
 *
 * S1 (2 블록, 8200B) 로 안정 동작 검증 (260418_180656). 효과:
 *   - pdo_gap C안 평균 3.7% → 1.6% (-57%, 단발)
 *   - USB CPU 34.8% → 20.7% (-14.1%p)
 *   - UserTask p99 -20%, bin06 -66%
 *
 * S2 (4 블록, 16400B) 추가 이득 없음 확인 (260418_181547):
 *   - Producer rate (122KB/s × 100ms = 12KB/drain) 가 bottleneck
 *   - write_count 739 ≈ 742, disk_write ratio 2.27 ≈ 2.28
 *
 * [Step D 2026-04-18] S1 유지하면서 **block 크기 자체** 를 4100 → 4096
 *   으로 조정 (data 4092 + CRC 4). FatFs partial-sector tail (fp->buf 경유
 *   4B 추가 disk_write) 완전 제거 → disk_write ratio 2.27 → ~1.0 기대.
 *   파일 포맷 version 1 → 2 (Python decoder 양쪽 지원).
 *
 * Capacity: 2 * CRC_BLOCK_TOTAL_SIZE = 8192B (S1 8200B 대비 -8B) */
#define CRC_STAGE_BLOCKS        (2U)
#define CRC_STAGE_CAPACITY      (CRC_STAGE_BLOCKS * CRC_BLOCK_TOTAL_SIZE)

__attribute__((aligned(32)))
static uint8_t s_crc_stage[CRC_STAGE_CAPACITY];

/** Stage 내 다음 write 위치 (누적 길이). 완성 블록 + CRC tail + 현재 in-progress 합. */
static uint32_t s_crc_stage_pos = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void StartDataLoggerTask(void* argument);
static bool _CheckUsbSpace(void);
static bool _InitSensorLogSession(const char* sessionName, const char* metadata);
static inline AGRBFileSystemStatusDef _log_write(IOIF_FILEx_t id, const uint8_t* data,
                                                 uint32_t size, FRESULT* res);
static bool _CheckFileRolling(void);
static void _HandleCommandQueue(void);
static uint32_t _GetBufferUsedSize(uint32_t head, uint32_t tail);
static void _WriteSummary_WithStatus(StopReason_e reason);
static void _WriteDiagProfile(void);
static void _RingBufferWrite(uint32_t* idx, const uint8_t* data, uint32_t size);

// [Phase 3a] Emergency Stop helpers
static void _EmergencyStop(StopReason_e reason);
static void _RequestEmergencyStop(StopReason_e reason);

// [2026-04-18 C안] Hot-only consumer — DataLoggerTask 가 직접 Hot → USB drain
static void _ProcessRingBuffer(void);

// [Phase 3d] CRC32 블록 무결성 헬퍼
static bool _WriteDataWithBlockCRC(const uint8_t* data, uint32_t size);

// [Diag 2026-04-17] FDCAN 세션 통계 trace (OffloadTask 호출)
static void _ResetFdcanSessionStats(void);
static void _TraceFdcanStats(void);
static void _CollectFdcanFinalStats(void);
static bool _FlushPartialBlockCRC(void);

// [Step B 재설계 #1] 부팅 시 1회 — pending fault dump 를 /LOGS/hardfault_*.txt 로 기록
static void _TryWritePendingFaultDump(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void DataLogger_Init(void)
{
    g_logCmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(LoggerCommand_t));

    atomic_store(&s_logHead, 0);
    s_logTail = 0;
    atomic_store(&s_isLoggingActive, false);
    atomic_store(&s_logStatus, LOG_STATUS_IDLE);
    memset(&s_stats, 0, sizeof(s_stats));

    /* [2026-04-18 C안] PSRAM Cold Buffer / QSPI Mutex / OffloadTask 제거.
     * DataLoggerTask 단독이 Hot 256KB 를 직접 drain → USB. */
    s_loggerTaskHandle = osThreadNew(StartDataLoggerTask, NULL, &s_loggerTask_attributes);
}

bool DataLogger_IsReady(void)
{
    // // usb_mode_handler.c가 관리하는 플래그 + IOIF 계층의 실제 준비 상태
    // return g_isUsbHostReady && ioif_filesystem_is_ready();
    // [수정] g_isUsbHostReady가 아닌,
    // IOIF의 최종 "f_mount 완료" 상태를 반환합니다.
    return (ioif_filesystem_is_ready() & g_isUsbHostReady);
}

bool DataLogger_Start(const char* sessionName, const char* metadata)
{
    if (!g_isUsbHostReady || metadata == NULL) return false;

    if (strlen(metadata) >= MAX_METADATA_SIZE) {
        return false;
    }

    LoggerCommand_t cmd = {0};
    cmd.command = LOG_CMD_START_SESSION;

    /* [Rev2.0] RTC 타임스탬프 기반 자동 세션 넘버링
     * sessionName이 NULL이거나 빈 문자열이면 YYMMDD_HHMMSS 형식 자동 생성.
     * Why: RTC 초 단위 해상도로 유니크 보장, 이진 탐색 불필요. */
    bool is_auto_naming = (sessionName == NULL || sessionName[0] == '\0');
    if (is_auto_naming) {
        /* _HandleCommandQueue에서 _GenerateRtcSessionName으로 처리 */
        cmd.sessionName[0] = '\0';
    } else {
        // 런타임 길이 검증
        if (strlen(sessionName) > MAX_SESSION_NAME_SIZE) {
            return false;
        }
        /* [Phase 3c M4] FATFS 금지문자 + 제어문자 검증 */
        for (const char* p = sessionName; *p != '\0'; p++) {
            if (*p < 0x20 || *p == '<' || *p == '>' || *p == ':' ||
                *p == '"' || *p == '/' || *p == '\\' || *p == '|' ||
                *p == '?' || *p == '*') {
                return false;
            }
        }
        strncpy(cmd.sessionName, sessionName, MAX_SESSION_NAME_SIZE);
    }

    strncpy(cmd.metadata, metadata, MAX_METADATA_SIZE - 1);

    // 2단계 큐에 "로깅 시작" 명령 전송 (100ms 타임아웃)
    return (xQueueSend(g_logCmdQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE);
}

void DataLogger_Stop(void)
{
    /* 즉시 로깅 차단 — DataLogger_Log() 호출을 바로 거부
     * Why: 큐 전송만 하면 DataLoggerTask(100ms)가 처리할 때까지
     *      최대 100ms간 stale 데이터가 링 버퍼에 계속 쌓임.
     *      atomic_store는 UserTask 컨텍스트에서 안전 (Lock-Free). */
    atomic_store(&s_isLoggingActive, false);

    LoggerCommand_t cmd = {0};
    cmd.command = LOG_CMD_STOP_SESSION;
    xQueueSend(g_logCmdQueue, &cmd, 0); // 비차단
}

/**
 * @brief [실시간] 2ms UserTask가 링 버퍼에 직접 쓰기 (Lock-Free)
 * @details auto_timestamp 활성화 시 매 패킷 앞에 4-byte tick 자동 삽입.
 *          File format: [LogPacketHeader_t][timestamp(opt)][user_data]
 */
bool DataLogger_Log(const void* logPacket, uint32_t packetSize)
{
    if (!atomic_load(&s_isLoggingActive)) return false;
    if (packetSize == 0 || packetSize > MAX_LOG_PACKET_SIZE) return false; 

    uint32_t ts_size = s_auto_timestamp_enabled ? LOG_TIMESTAMP_SIZE : 0;
    uint32_t payload_size = ts_size + packetSize;

    uint32_t head = atomic_load(&s_logHead);
    uint32_t tail = s_logTail;
    
    uint32_t used_size = _GetBufferUsedSize(head, tail);
    uint32_t free_space = LOG_BUFFER_SIZE - used_size - 1;
    uint32_t total_write_size = sizeof(LogPacketHeader_t) + payload_size;

    /* [Phase 3c L3] Hot Buffer 피크 사용률 추적 */
    uint8_t usage_pct = (uint8_t)((uint64_t)used_size * 100 / LOG_BUFFER_SIZE);
    if (usage_pct > s_stats.hot_buffer_peak_percent) {
        s_stats.hot_buffer_peak_percent = usage_pct;
    }

    if (total_write_size > free_space) {
        s_stats.dropped_packets++;
        /* [Phase 3a] UserTask 컨텍스트 → FATFS 접근 금지 → flag만 설정 */
        _RequestEmergencyStop(STOP_REASON_HOT_OVERFLOW);
        return false;
    }

    uint32_t idx = head;
    LogPacketHeader_t header = { .packetSize = payload_size };

    _RingBufferWrite(&idx, (const uint8_t*)&header, sizeof(LogPacketHeader_t));

    if (s_auto_timestamp_enabled) {
        uint32_t tick = xTaskGetTickCount();
        _RingBufferWrite(&idx, (const uint8_t*)&tick, sizeof(uint32_t));
    }

    _RingBufferWrite(&idx, (const uint8_t*)logPacket, packetSize);
    
    atomic_store(&s_logHead, idx);
    s_stats.total_packets_logged++;
    s_part_record_count++;  /* [Phase 3b] 파트 파일별 레코드 카운트 */
    return true;
}

/* ===== [Phase 3e] 이벤트 마커 삽입 ===== */
bool DataLogger_InsertMarker(uint8_t marker_type, uint16_t data)
{
    if (!atomic_load(&s_isLoggingActive)) return false;

    /* 마커 레코드: LogPacketHeader_t(4) + LogMarkerRecord_t(12) = 16 bytes */
    LogMarkerRecord_t marker = {
        .marker_header = LOG_MARKER_MAGIC | (uint32_t)marker_type,
        .tick_ms       = xTaskGetTickCount(),
        .marker_data   = data,
        .reserved      = 0,
    };

    uint32_t total_write_size = sizeof(LogPacketHeader_t) + sizeof(LogMarkerRecord_t);
    uint32_t head = atomic_load(&s_logHead);
    uint32_t tail = s_logTail;
    uint32_t free_space = LOG_BUFFER_SIZE - _GetBufferUsedSize(head, tail) - 1;

    if (total_write_size > free_space) {
        return false; /* 마커 drop — 데이터보다 우선순위 낮으므로 emergency stop 안 함 */
    }

    uint32_t idx = head;
    /* packetSize = LOG_MARKER_MAGIC | type → 디코더가 상위 24비트로 마커 식별 */
    LogPacketHeader_t hdr = { .packetSize = LOG_MARKER_MAGIC | (uint32_t)marker_type };
    _RingBufferWrite(&idx, (const uint8_t*)&hdr, sizeof(LogPacketHeader_t));
    _RingBufferWrite(&idx, (const uint8_t*)&marker, sizeof(LogMarkerRecord_t));

    atomic_store(&s_logHead, idx);
    s_stats.total_packets_logged++;
    s_part_record_count++;
    return true;
}

DataLogger_Status_e DataLogger_GetStatus(void)
{
    return (DataLogger_Status_e)atomic_load(&s_logStatus);
}

void DataLogger_GetStats(DataLogger_Stats_t* stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void DataLogger_GetActiveSessionName(char* out_buf, uint32_t buf_size)
{
    if (out_buf == NULL || buf_size == 0) { return; }
    /* s_currentSessionName은 _InitSensorLogSession에서만 쓰고 DataLoggerTask에서만 접근.
     * UserTask(예제)는 세션 종료 후 조회 → race 없음. */
    strncpy(out_buf, s_currentSessionName, buf_size - 1);
    out_buf[buf_size - 1] = '\0';
}

void DataLogger_SetPacketSize(uint32_t size)
{
    s_user_packet_size = size;
}

void DataLogger_SetAutoTimestamp(bool enabled)
{
    s_auto_timestamp_enabled = enabled;
}

void DataLogger_SetRollingSize(uint32_t size_mb)
{
    if (size_mb >= 1 && size_mb <= 100) {
        s_rolling_size_mb = size_mb;
    }
}

/* [Phase 3b M1] Cached FatTime — fatfs.c의 get_fattime()에서 호출 */
uint32_t DataLogger_GetCachedFatTime(void)
{
    return s_cached_fattime;
}

/** @brief RTC → FATFS DWORD 타임스탬프 캐시 갱신 (10초 주기) */
static void _UpdateCachedFatTime(void)
{
    XmDateTime_t dt;
    if (!XM_RTC_GetDateTime(&dt) || dt.year < 1980) return;
    s_cached_fattime = ((uint32_t)(dt.year - 1980) << 25)
                     | ((uint32_t)dt.month << 21)
                     | ((uint32_t)dt.day << 16)
                     | ((uint32_t)dt.hour << 11)
                     | ((uint32_t)dt.minute << 5)
                     | ((uint32_t)(dt.second >> 1));
}

/* [Phase 3c] 디스크 용량 캐시 갱신 (10초 주기) + 공개 조회 함수 */
static void _UpdateDiskFreeCache(void)
{
    /* [Q5] get_space_mb로 free + total 동시 갱신 (f_getfree 1회 호출) */
    uint32_t free_mb = 0;
    uint32_t total_mb = 0;
    if (AGRBFileSystem.get_space_mb(&free_mb, &total_mb) == IOIF_FileSystem_OK) {
        s_cached_disk_free_mb = free_mb;
        s_cached_disk_total_mb = total_mb;
        s_stats.disk_free_mb = free_mb;
        s_stats.disk_total_mb = total_mb;
    }
}

uint32_t DataLogger_GetDiskFreeMB(void)  { return s_cached_disk_free_mb; }
uint32_t DataLogger_GetDiskTotalMB(void) { return s_cached_disk_total_mb; }

uint32_t DataLogger_GetLastFsyncUs(void) { return 0; } /* Pre-allocation: 로깅 중 f_sync 0회 */
uint32_t DataLogger_GetMaxFsyncUs(void)  { return 0; } /* Pre-allocation: 로깅 중 f_sync 0회 */

/* ===== [Phase 3b] 파일 헤더/풋터 쓰기 헬퍼 ===== */

/** @brief 현재 파일에 32-byte 헤더를 기록 */
static bool _WriteFileHeader(void)
{
    uint32_t ts_bytes = s_auto_timestamp_enabled ? LOG_TIMESTAMP_SIZE : 0;
    FRESULT res;

    DataLogFileHeader_t hdr = {
        .magic              = LOG_FILE_MAGIC,
        .format_version     = LOG_FILE_FORMAT_VERSION,
        .flags              = (s_auto_timestamp_enabled ? LOG_FILE_FLAG_AUTO_TIMESTAMP : 0)
                              | LOG_FILE_FLAG_BLOCK_CRC,
        .header_size        = (uint16_t)sizeof(DataLogFileHeader_t),
        .record_total_bytes = (uint32_t)(sizeof(LogPacketHeader_t) + ts_bytes + s_user_packet_size),
        .user_payload_bytes = s_user_packet_size,
        .creation_fattime   = s_cached_fattime,
        .reserved           = {0, 0},
    };

    _log_write(s_currentFileId, (uint8_t*)&hdr, sizeof(hdr), &res);
    s_part_record_count = 0;
    s_part_data_bytes = 0;

    /* [Phase 3d + Step B v4 S1] CRC 블록 / stage 상태 리셋 (새 파일 시작) */
    s_crc_block_offset = 0;
    s_crc_stage_pos    = 0;
    return (res == FR_OK);
}

/** @brief 현재 파일에 12-byte 풋터를 기록 (close 직전 호출) */
static bool _WriteFileFooter(void)
{
    FRESULT res;
    DataLogFileFooter_t ftr = {
        .record_count = s_part_record_count,
        .data_bytes   = s_part_data_bytes,
        .footer_magic = LOG_FILE_FOOTER_MAGIC,
    };

    _log_write(s_currentFileId, (uint8_t*)&ftr, sizeof(ftr), &res);
    return (res == FR_OK);
}

/* [Phase 2A] Logger write 추적 래퍼 — IOIF 층을 수정하지 않고 Logger 경로만 계측.
 * Mutex 보유 컨텍스트에서만 호출됨 (모든 AGRBFileSystem.write 는 FATFS_MUTEX_LOCK_IO 내부).
 * DIAG_PROFILE_ENABLED=OFF 시 DiagPerf_RecordWrite 는 inline no-op → zero-cost. */
static inline AGRBFileSystemStatusDef _log_write(IOIF_FILEx_t id, const uint8_t* data,
                                                 uint32_t size, FRESULT* res)
{
    DiagPerf_RecordWrite(size);
    return AGRBFileSystem.write(id, data, size, res);
}

/* ===== [Phase 3d] CRC32 블록 무결성 헬퍼 =====
 * 4KB 데이터 블록마다 HW CRC32 (STM32H7 CRC peripheral)를 계산하여 4-byte append.
 * 파일 포맷: [Header][Block0 4KB][CRC 4B][Block1 4KB][CRC 4B]...[PartialBlock][CRC 4B][Footer]
 * 손상 시 해당 블록만 스킵하고 나머지 복구 가능 (partial recovery). */

/**
 * @brief 데이터를 4KB 블록 단위로 쓰고, 블록 경계마다 CRC32를 append한다.
 * @param[in] data 쓸 데이터 버퍼 (D-Cache Clean 완료된 상태)
 * @param[in] size 데이터 크기 (bytes)
 * @return true: 전체 쓰기 성공, false: f_write 실패
 */
static bool _WriteDataWithBlockCRC(const uint8_t* data, uint32_t size)
{
#if defined(DIAG_KILL_DRAIN_WRITE)
    /* [Step 0 K2a] Drain write 전면 스킵 — tight spin 기여 격리 측정용.
     * FATFS 에 f_write 호출 자체가 안 가므로 내부 캐시 일관성 유지.
     * metadata/summary/diag_profile write 는 별도 경로이므로 영향 없음. */
    (void)data; (void)size;
    return true;
#else
    /* [Step B v4 S1] Stage 레이아웃: [완성0][CRC0][완성1][CRC1]...[in-progress].
     *   s_crc_stage_pos   = stage 내 다음 쓸 위치 (완성 영역 + 현재 partial 포함)
     *   s_crc_block_offset = 현재 블록에 누적된 바이트 수 (partial 크기)
     *   호출 종료 시: 완성 영역 = [0 .. pos - block_offset) → 단일 f_write
     *                in-progress = stage[pos - block_offset .. pos) → 앞쪽으로 memmove
     *
     * 안전성 점검:
     *   - memcpy 목적지 = stage + pos → pos 증가 후 다음 iter 에 이어 씀
     *   - s_crc_stage_pos 최대값: CRC_STAGE_BLOCKS 개 완성 시 CRC_STAGE_CAPACITY
     *     → overflow 전 while 조건으로 강제 flush 필요. 호출당 size 는
     *       최대 MAX_WRITE_CHUNK_SIZE (32KB) 이므로 CRC_STAGE_CAPACITY (8192B) 를
     *       초과할 수 있음 → 루프 안에서 stage full 시 중간 flush 추가.
     */
    uint32_t offset = 0;
    FRESULT res = FR_OK;

    while (offset < size) {
        uint32_t space_in_block = CRC_BLOCK_DATA_SIZE - s_crc_block_offset;
        uint32_t chunk = (size - offset < space_in_block) ? (size - offset) : space_in_block;

        /* Stage 에 append (pos 위치에) */
        memcpy(s_crc_stage + s_crc_stage_pos, data + offset, chunk);

        /* HW CRC 누적: 블록 첫 chunk 에 Calculate, 이후 Accumulate */
        if (s_crc_block_offset == 0) {
            HAL_CRC_Calculate(&hcrc, (uint32_t*)(data + offset), chunk);
        } else {
            HAL_CRC_Accumulate(&hcrc, (uint32_t*)(data + offset), chunk);
        }

        s_crc_stage_pos    += chunk;
        s_crc_block_offset += chunk;
        offset             += chunk;

        /* 블록 완성 → CRC tail 을 stage 에 append (아직 write 안 함) */
        if (s_crc_block_offset >= CRC_BLOCK_DATA_SIZE) {
            uint32_t crc_val = hcrc.Instance->DR;
            memcpy(s_crc_stage + s_crc_stage_pos, &crc_val, sizeof(crc_val));
            s_crc_stage_pos   += sizeof(crc_val);
            s_crc_block_offset = 0;
        }

        /* Stage 용량 한계 도달 → 완성 영역 flush 후 계속.
         * CRC_STAGE_BLOCKS 개 완성 시 추가 데이터 공간 부족.
         * CRC_BLOCK_TOTAL_SIZE = data + CRC tail 포함 1 블록 크기. */
        if (s_crc_stage_pos + CRC_BLOCK_TOTAL_SIZE > CRC_STAGE_CAPACITY) {
            uint32_t completed_len = s_crc_stage_pos - s_crc_block_offset;
            if (completed_len > 0) {
                _log_write(s_currentFileId, s_crc_stage, completed_len, &res);
                if (res != FR_OK) return false;

                if (s_crc_block_offset > 0) {
                    memmove(s_crc_stage, s_crc_stage + completed_len, s_crc_block_offset);
                }
                s_crc_stage_pos = s_crc_block_offset;
            }
        }
    }

    /* 호출 종료 시 완성 영역 flush. 남은 in-progress 는 stage 앞쪽으로 이동. */
    uint32_t completed_len = s_crc_stage_pos - s_crc_block_offset;
    if (completed_len > 0) {
        _log_write(s_currentFileId, s_crc_stage, completed_len, &res);
        if (res != FR_OK) return false;

        if (s_crc_block_offset > 0) {
            memmove(s_crc_stage, s_crc_stage + completed_len, s_crc_block_offset);
        }
        s_crc_stage_pos = s_crc_block_offset;
    }
    return true;
#endif /* DIAG_KILL_DRAIN_WRITE */
}

/**
 * @brief 마지막 불완전 블록의 CRC를 flush (파일 close/롤링 전 호출)
 * @note 불완전 블록이 없으면 (s_crc_block_offset==0) 아무것도 쓰지 않음.
 */
static bool _FlushPartialBlockCRC(void)
{
    if (s_crc_block_offset > 0) {
        FRESULT res;
        uint32_t crc_val = hcrc.Instance->DR;
        /* [Step B v4 S1] _WriteDataWithBlockCRC 호출 종료 시 memmove 로
         * in-progress 가 stage 앞쪽(0..s_crc_block_offset) 에 위치함이 보장됨.
         * 여기에 CRC tail 을 append 해서 (partial + 4B) 단일 write. */
        memcpy(s_crc_stage + s_crc_block_offset, &crc_val, sizeof(crc_val));
        _log_write(s_currentFileId, s_crc_stage,
                   s_crc_block_offset + sizeof(crc_val), &res);
        s_crc_block_offset = 0;
        s_crc_stage_pos    = 0;
        if (res != FR_OK) return false;
    }
    return true;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief [핵심] 데이터 저장 태스크 (Prio 24, 100ms 주기, time-bounded drain)
 */
static void StartDataLoggerTask(void* argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(LOGGER_TASK_PERIOD_MS);
    uint32_t rtcSyncCounter = 0;   /* [Phase 3b] RTC 캐시 갱신 카운터 */
    uint32_t diskCheckCounter = 0; /* [Phase 3c] 디스크 용량 체크 카운터 */

#if defined(XM_TEST_FAULT_AT_BOOT)
    /* [Step B 재설계 #1 round-trip 검증] boot 후 약 5초 (50 tick × 100ms)
     * 시점에 의도적 UsageFault 발생 → NVIC_SystemReset → 재마운트 시
     * /LOGS/hardfault_<uptime>.txt 생성되는지 확인.
     * 검증 완료 후 module.h 의 XM_TEST_FAULT_AT_BOOT 주석 복귀. */
    uint32_t test_fault_counter = 0;
    #define XM_TEST_FAULT_TICKS 50U
#endif

    for (;;) {
        AGRBFileSystem.process_mount();

#if defined(XM_TEST_FAULT_AT_BOOT)
        /* 2번째 부팅 (pending dump 존재) 에는 fault 재유발 금지 — 파일 기록 기회 확보.
         * HasBootDump() false 인 경우만 카운트 (첫 boot, 또는 dump 소비 후). */
        if (!HardFault_HasBootDump() && test_fault_counter < XM_TEST_FAULT_TICKS) {
            ++test_fault_counter;
            if (test_fault_counter == XM_TEST_FAULT_TICKS) {
                /* UsageFault (DIVBYZERO) 유발 — SCB->CCR.DIV_0_TRP 는
                 * 다음 줄에서 선 설정. volatile 로 컴파일러 상수 접힘 방지. */
                SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
                __DSB();
                volatile int zero = 0;
                volatile int boom = 10 / zero;
                (void)boom;
            }
        }
#endif

        /* [Phase 3a] Deferred emergency stop: UserTask가 flag 설정 → 여기서 cleanup */
        if (s_pending_stop_reason != STOP_REASON_NORMAL) {
            _EmergencyStop(s_pending_stop_reason);
            s_pending_stop_reason = STOP_REASON_NORMAL;
        }

        /* [Phase 3a] USB Disconnect → _EmergencyStop (FATFS 호출 자동 스킵) */
        if (!DataLogger_IsReady()) {
            if (atomic_load(&s_isLoggingActive)) {
                _EmergencyStop(STOP_REASON_USB_DISCONNECT);
            }
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            continue;
        }

        /* [Step B 재설계 #1] USB 마운트 완료 직후 pending HardFault dump 파일 기록.
         * 내부 플래그로 1-shot 보장 — 이미 소비됐으면 HasBootDump() false. */
        _TryWritePendingFaultDump();

        /* [Phase 3b M1] 10초 주기 RTC → FatTime 캐시 갱신 (로깅 여부 무관) */
        if (++rtcSyncCounter >= RTC_SYNC_INTERVAL_CYCLES) {
            rtcSyncCounter = 0;
            _UpdateCachedFatTime();
        }

        /* [Phase 3c] 10초 주기 디스크 용량 캐시 갱신 */
        if (++diskCheckCounter >= DISK_CHECK_INTERVAL_CYCLES) {
            diskCheckCounter = 0;
            _UpdateDiskFreeCache();
            /* 로깅 중 디스크 부족 경고 + 회복 */
            if (atomic_load(&s_isLoggingActive)) {
                if (s_cached_disk_free_mb < DISK_LOW_THRESHOLD_MB &&
                    s_cached_disk_free_mb > 0) {
                    atomic_store(&s_logStatus, LOG_STATUS_WARNING_DISK_LOW);
                } else if (atomic_load(&s_logStatus) == LOG_STATUS_WARNING_DISK_LOW &&
                           s_cached_disk_free_mb >= DISK_LOW_THRESHOLD_MB) {
                    /* [Issue #5] DISK_LOW → 용량 확보 시 LOGGING으로 회복 */
                    atomic_store(&s_logStatus, LOG_STATUS_LOGGING);
                }
            }
        }

        _HandleCommandQueue();

        if (atomic_load(&s_isLoggingActive)) {
            /* Time-bounded drain: 100ms 주기 중 최대 50ms 할당해 Hot → USB drain.
             * Pre-allocation 으로 로깅 중 f_sync 0회. */
            uint32_t drain_start = osKernelGetTickCount();
            do {
                _ProcessRingBuffer();
                /* EmergencyStop 발생 시 (파일 닫힘) 즉시 탈출 — 닫힌 파일 재write 방지 */
                if (!atomic_load(&s_isLoggingActive)) break;
                if (_GetBufferUsedSize(atomic_load(&s_logHead), s_logTail) == 0) break;
            } while ((osKernelGetTickCount() - drain_start) < (LOGGER_TASK_PERIOD_MS / 2));

            /* File rolling: drain 완료 후 1회만 체크.
             * 루프 안에서 매번 호출하면 rolling 시 close+open+prealloc이 50ms budget 폭발. */
            _CheckFileRolling();
            /* [2026-04-18 C안] FDCAN bus trace: OffloadTask(20ms) → DataLoggerTask(100ms) 이관.
             * BusOff 전이 감지만 정확하면 충분 (fill peak 는 100ms 해상도로도 유의미). */
            _TraceFdcanStats();
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * @brief [수정] ioif_agrb_fs API 사용
 */
static bool _CheckUsbSpace(void)
{
    uint32_t free_mb = 0;
    if (AGRBFileSystem.get_free_space_mb(&free_mb) == IOIF_FileSystem_OK) {
        return (free_mb > s_rolling_size_mb);
    }
    return false;
}

/**
 * @brief [Rev2.0] RTC 타임스탬프 기반 자동 세션명 생성 (YYMMDD_HHMMSS)
 * @param[out] out_name 결과 세션명 버퍼
 * @param[in]  out_size 버퍼 크기
 * @return true: 성공, false: RTC 읽기 실패
 * @note RTC 초 단위 해상도 → 1초 이내 연속 START 시 동일 이름 가능하나,
 *       실사용 시나리오에서는 발생 불가 (START → 측정 → STOP → 분석 루프).
 */
static bool _GenerateRtcSessionName(char* out_name, uint32_t out_size)
{
    XmDateTime_t now;
    if (!XM_RTC_GetDateTime(&now)) return false;

    snprintf(out_name, out_size, "%02u%02u%02u_%02u%02u%02u",
             (unsigned)(now.year % 100), (unsigned)now.month, (unsigned)now.day,
             (unsigned)now.hour, (unsigned)now.minute, (unsigned)now.second);
    return true;
}


/**
 * @brief 세션 시작 시 빈 '세션 번호'를 찾고 part_000 파일 생성
 */
static bool _InitSensorLogSession(const char* sessionName, const char* metadata)
{
    IOIF_FILEx_t file_id;
    char path[MAX_FULL_PATH_SIZE];

    // 1. 기본 로그 디렉터리 생성
    AGRBFileSystem.mkdir(LOG_DIR_PATH, NULL); // "/LOGS"

    // 2. 세션 디렉터리 생성 (예: "/LOGS/S_001")
    snprintf(s_currentSessionPath, MAX_PATH_SIZE, "%s/%s", LOG_DIR_PATH, sessionName);
    AGRBFileSystem.mkdir(s_currentSessionPath, NULL);

    /* [Option A] 세션 이름 보관 — Emergency Stop 후 재진입 시 같은 폴더 사용 */
    strncpy(s_currentSessionName, sessionName, MAX_SESSION_NAME_SIZE);
    s_currentSessionName[MAX_SESSION_NAME_SIZE] = '\0';

    /* [Phase 3b M5] 세션 시작 RTC 시각 캡처 (1회 SPI 호출, ~5ms) */
    memset(&s_session_rtc_start, 0, sizeof(s_session_rtc_start));
    XM_RTC_GetDateTime(&s_session_rtc_start);

    // 3. 메타데이터 파일 저장 (예: "/LOGS/S_001/metadata.txt")
    snprintf(path, sizeof(path), "%s/metadata.txt", s_currentSessionPath);

    /* [Option A Redesign 2026-04-17] metadata.txt가 이미 있으면 재작성 스킵.
     * Why: Emergency Stop 후 같은 폴더 재진입 시 매번 OVERWRITE → FAT32 wear hotspot →
     *      USB 내부 손상 가속 (실측으로 USB 개체가 테스트 중 망가지는 현상 재현).
     *      같은 세션의 metadata는 불변이므로 skip 안전. */
    IOIF_FILEx_t check_id;
    bool metadata_exists = (AGRBFileSystem.open(&check_id, path,
                              IOIF_FileSystem_AccessMode_READONLY, NULL) == IOIF_FileSystem_OK);
    if (metadata_exists) {
        AGRBFileSystem.close(check_id, NULL);
        /* 재진입 — metadata write 전체 스킵, 아래 블록 skip */
    } else if (AGRBFileSystem.open_write(&file_id, path, IOIF_FileSystem_CreateMode_OVERWRITE, NULL) == IOIF_FileSystem_OK) {
        _log_write(file_id, (uint8_t*)metadata, strlen(metadata), NULL);

        uint32_t ts_bytes = s_auto_timestamp_enabled ? LOG_TIMESTAMP_SIZE : 0;
        uint32_t record_payload = ts_bytes + s_user_packet_size;
        uint32_t record_total = sizeof(LogPacketHeader_t) + record_payload;

        char sys_meta[512];
        int len = snprintf(sys_meta, sizeof(sys_meta),
            "\n\n=== System Info ===\n"
            "auto_timestamp=%d\n"
            "timestamp_bytes=%lu\n"
            "user_payload_bytes=%lu\n"
            "record_header_bytes=%u\n"
            "record_total_bytes=%lu\n"
            "rolling_size_mb=%lu\n"
            "buffer_size_kb=%u\n"
            "logger_period_ms=%u\n"
            "rtc_start=%04u-%02u-%02u %02u:%02u:%02u\n"
            "file_format_version=%u\n",
            (int)s_auto_timestamp_enabled,
            (unsigned long)ts_bytes,
            (unsigned long)s_user_packet_size,
            (unsigned)sizeof(LogPacketHeader_t),
            (unsigned long)record_total,
            (unsigned long)s_rolling_size_mb,
            (unsigned)(LOG_BUFFER_SIZE / 1024),
            (unsigned)LOGGER_TASK_PERIOD_MS,
            s_session_rtc_start.year, s_session_rtc_start.month, s_session_rtc_start.day,
            s_session_rtc_start.hour, s_session_rtc_start.minute, s_session_rtc_start.second,
            (unsigned)LOG_FILE_FORMAT_VERSION);
        _log_write(file_id, (uint8_t*)sys_meta, (uint32_t)len, NULL);

        AGRBFileSystem.close(file_id, NULL);
    } else {
        return false;
    }

    // 4. 이진 탐색으로 '빈 세션 번호(data_XXX)' 찾기 찾기 (Max 10회 시도)
    uint32_t low = 0;
    uint32_t high = 1000; // 최대 파일 개수 제한
    uint32_t mid = 0;
    IOIF_FILEx_t temp_id;
    char temp_path[MAX_FULL_PATH_SIZE];

    // "data_XXX_part_000.bin"이 존재하는지 확인하여 세션 번호 결정
    while (low < high) {
        mid = (low + high) / 2;

        // 확인용 경로: .../data_mid_part_000.bin
        snprintf(temp_path, sizeof(temp_path), "%s/data_%03lu_part_000.bin", s_currentSessionPath, mid);
        // 파일 존재 여부만 빠르게 확인 (open_read 시도)
        // (주의: EXCLUSIVE로 열지 않고, 단순히 읽기 모드로 열어서 존재만 확인)
        if (AGRBFileSystem.open(&temp_id, temp_path, IOIF_FileSystem_AccessMode_READONLY, NULL) == IOIF_FileSystem_OK) {
            AGRBFileSystem.close(temp_id, NULL);
            low = mid + 1; // 존재함 -> 더 높은 번호 탐색
        } else {
            high = mid;    // 존재 안함 -> 이 번호가 후보
        }
    }

    if (low >= 1000) return false; // 세션 1000개 초과
    
    // 최종적으로 'low'가 우리가 사용할 인덱스입니다.
    s_currentSessionIndex = low; // 결정된 세션 번호
    s_currentSplitIndex = 0;     // 분할 번호는 0부터 시작
    
    if (s_currentSessionIndex >= 1000) {
        return false; // 1000개 파일 꽉 참
    }

    // 5. 첫 번째 파일 생성 (data_XXX_part_000.bin)
    char final_path[MAX_FULL_PATH_SIZE];
    snprintf(final_path, sizeof(final_path), "%s/data_%03lu_part_%03lu.bin",
             s_currentSessionPath, s_currentSessionIndex, s_currentSplitIndex);

    if (AGRBFileSystem.open_write(&s_currentFileId, final_path, IOIF_FileSystem_CreateMode_EXCLUSIVE, NULL) != IOIF_FileSystem_OK) {
        return false;
    }

    /* [Phase 3b] 파일 헤더 기록 (첫 파트) */
    _WriteFileHeader();

    /* ★ Pre-allocation: Rolling Size만큼 FAT 클러스터 사전 할당.
     * f_lseek(rolling_size) → FAT 체인 확장, f_sync 1회 → FAT+디렉토리 확정.
     * 이후 로깅 중 f_sync 0회 → data gap 0.
     * 비정상 종료 시에도 파일이 full size로 디스크에 존재 → 데이터 복구 가능. */
    uint32_t prealloc_bytes = s_rolling_size_mb * 1024 * 1024;
    if (AGRBFileSystem.seek(s_currentFileId, prealloc_bytes, NULL) != IOIF_FileSystem_OK) {
        AGRBFileSystem.close(s_currentFileId, NULL);
        return false;
    }
    if (AGRBFileSystem.sync(s_currentFileId, NULL) != IOIF_FileSystem_OK) {
        AGRBFileSystem.close(s_currentFileId, NULL);
        return false;
    }
    /* 헤더 직후로 R/W 포인터 복귀 — 이후 f_write는 여기부터 순차 기록 */
    if (AGRBFileSystem.seek(s_currentFileId, sizeof(DataLogFileHeader_t), NULL) != IOIF_FileSystem_OK) {
        AGRBFileSystem.close(s_currentFileId, NULL);
        return false;
    }

    return true;
}

/**
 * @brief 파일 용량이 rolling_size_mb를 넘으면 다음 part 파일 생성.
 * @note  [Bug fix 2026-04-17] Pre-allocation 도입 후 get_size_mb는 prealloc 크기(10MB)를
 *        그대로 반환 → 첫 32KB 쓰기 직후 rolling 즉시 발동 → 매 write마다 파일 생성하는
 *        폭발 현상 발생 (D:\LOGS 세션 1개당 bin 200개). 실 쓰기 카운터인 s_part_data_bytes
 *        로 판정해야 정확. close 직전 _WriteFileHeader가 이 값을 리셋.
 */
static bool _CheckFileRolling(void)
{
    const uint32_t rolling_bytes = s_rolling_size_mb * 1024UL * 1024UL;

    if (s_part_data_bytes < rolling_bytes) {
        return true;
    }

    {
        /* [Phase 3d] 불완전 블록 CRC flush + [Phase 3b] 풋터 기록 후 close */
        _FlushPartialBlockCRC();
        _WriteFileFooter();
        /* ★ Pre-allocation 정리: 실제 데이터 크기로 파일 축소 */
        AGRBFileSystem.truncate(s_currentFileId, NULL);
        AGRBFileSystem.close(s_currentFileId, NULL);
        s_file_open = false; /* [Q6] close 후 상태 갱신 */

        // 2. 분할 인덱스 증가
        s_currentSplitIndex++;

        // 3. 다음 파일 경로 생성 (data_XXX_part_001.bin ...)
        char next_path[MAX_FULL_PATH_SIZE];
        snprintf(next_path, sizeof(next_path), "%s/data_%03lu_part_%03lu.bin",
                s_currentSessionPath, s_currentSessionIndex, s_currentSplitIndex);

        // 4. 새 파일 열기
        if (AGRBFileSystem.open_write(&s_currentFileId, next_path, IOIF_FileSystem_CreateMode_EXCLUSIVE, NULL) != IOIF_FileSystem_OK) {
            /* [Phase 3a] 롤링 실패 → _EmergencyStop (old 파일은 이미 close됨) */
            _EmergencyStop(STOP_REASON_ROLLING_FAILURE);
            return false;
        }
        s_file_open = true; /* [Q6] open 성공 후 상태 갱신 */

        /* [Phase 3b] 새 파트 파일 헤더 기록 */
        _WriteFileHeader();

        /* ★ Pre-allocation: 새 파트 파일도 Rolling Size만큼 사전 할당 */
        uint32_t prealloc_bytes = s_rolling_size_mb * 1024 * 1024;
        if (AGRBFileSystem.seek(s_currentFileId, prealloc_bytes, NULL) != IOIF_FileSystem_OK ||
            AGRBFileSystem.sync(s_currentFileId, NULL) != IOIF_FileSystem_OK ||
            AGRBFileSystem.seek(s_currentFileId, sizeof(DataLogFileHeader_t), NULL) != IOIF_FileSystem_OK) {
            _EmergencyStop(STOP_REASON_ROLLING_FAILURE);
            return false;
        }
    }
    return true;
}

/**
 * @brief 2단계 큐(명령 큐)를 확인하고 처리하는 헬퍼
 */
static void _HandleCommandQueue(void)
{
    /* [Phase 2 Fix] static: sizeof(LoggerCommand_t) ≈ 2.1KB → 스택에서 제거.
     * DataLoggerTask 단일 호출이므로 재진입 위험 없음. */
    static LoggerCommand_t cmdBuffer;

    // 비차단으로 명령 큐 확인
    if (xQueueReceive(g_logCmdQueue, &cmdBuffer, 0) == pdTRUE) {
        if (cmdBuffer.command == LOG_CMD_START_SESSION) {

            /* [Sync Rev1.1] 이전 로깅이 비정상 종료되었다면 CRC+Footer 정리 후 close
             * Why: close()만 하면 이전 파일의 Footer/CRC가 누락되어 디코더가 파일 끝을 인식 못함. */
            if (s_file_open) {
                _FlushPartialBlockCRC();
                _WriteFileFooter();
                AGRBFileSystem.truncate(s_currentFileId, NULL);
                AGRBFileSystem.close(s_currentFileId, NULL);
                s_file_open = false;
            }

            /* [Rev2.0] RTC 타임스탬프 기반 자동 세션명 생성 (YYMMDD_HHMMSS) */
            char session_name[MAX_SESSION_NAME_SIZE + 1];
            if (cmdBuffer.sessionName[0] == '\0') {
                if (!_GenerateRtcSessionName(session_name, sizeof(session_name))) {
                    atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
                    return;
                }
            } else {
                strncpy(session_name, cmdBuffer.sessionName, MAX_SESSION_NAME_SIZE);
                session_name[MAX_SESSION_NAME_SIZE] = '\0';
            }

            if (DataLogger_IsReady() && _CheckUsbSpace()) {
                if (_InitSensorLogSession(session_name, cmdBuffer.metadata)) {
                    s_file_open = true; /* [Q6] 파일 생성 성공 → open 상태 */
                    atomic_store(&s_logHead, 0);
                    s_logTail = 0;
                    memset(&s_stats, 0, sizeof(s_stats));
                    s_stats.start_tick = xTaskGetTickCount();
                    /* [Diag 2026-04-17] USBH_MSC polling/write 실측 — 세션별 깨끗한 샘플 */
                    USBH_DiskIO_ResetDiag();
                    /* [Diag 2026-04-17] CM PDO loop counter 연속성 — CAN 수신 품질 */
                    CM_ResetPdoDiag();
                    /* [Diag 2026-04-17] FDCAN bus 세션 통계 — CAN 층 무결성 */
                    _ResetFdcanSessionStats();

                    /* [Sync Rev1.1] 이전 overflow 플래그 클리어
                     * Why: 이전 세션의 Emergency Stop 플래그가 남아있으면
                     *      새 세션이 즉시 emergency stop됨. */
                    s_pending_stop_reason = STOP_REASON_NORMAL;

                    /* [Sync Rev1.1] 디스크 캐시 즉시 갱신 (최대 10초 stale 방지) */
                    s_cached_disk_free_mb = DataLogger_GetDiskFreeMB();
                    s_cached_disk_total_mb = DataLogger_GetDiskTotalMB();

                    atomic_store(&s_isLoggingActive, true);
                    atomic_store(&s_logStatus, LOG_STATUS_LOGGING);
                } else {
                    atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
                }
            } else {
                /* [Sync Rev1.1] Silent fail 방지: Ready/Space 실패 시
                 * 명령만 소비되고 상태가 IDLE로 남아 앱이 ACTIVE인데 로깅은 안 되는 버그.
                 * ERROR_STOPPED로 설정하여 앱이 감지 가능하게 함. */
                atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
            }
        } 
        else if (cmdBuffer.command == LOG_CMD_STOP_SESSION) {
            /* [Sync Rev1.1] s_isLoggingActive는 DataLogger_Stop()에서 이미 false로 설정됨.
             * 파일 마무리는 s_file_open 기준으로 수행. */
            atomic_store(&s_isLoggingActive, false);  /* 방어적 중복 설정 */

            /* [2026-04-18 C안] 세션 종료 시 Hot → USB 잔여 데이터 전부 drain.
             * isLoggingActive=false → UserTask 쓰기 중단. 남은 bytes 소진 후 파일 close. */
            while (_GetBufferUsedSize(atomic_load(&s_logHead), s_logTail) > 0) {
                _ProcessRingBuffer();
                /* EmergencyStop 발생 시 탈출 */
                if (!s_file_open) break;
            }

            /* [Phase 3d] 불완전 블록 CRC flush + [Phase 3b] 풋터 기록 후 close */
            if (s_file_open) {
                _FlushPartialBlockCRC();
                _WriteFileFooter();
                /* ★ Pre-allocation 정리: 실제 데이터 크기로 파일 축소 */
                AGRBFileSystem.truncate(s_currentFileId, NULL);
                AGRBFileSystem.close(s_currentFileId, NULL);
                s_file_open = false; /* [Q6] close 후 상태 갱신 */

                s_stats.end_tick = xTaskGetTickCount();
                /* [Step 0 2026-04-18] Summary write 이전에 histogram freeze →
                 * summary 자체의 tight spin 이 측정에 포함되지 않도록 함 */
                DiagPerf_Finalize();
                _WriteSummary_WithStatus(STOP_REASON_NORMAL);
                _WriteDiagProfile();
            }
            atomic_store(&s_logStatus, LOG_STATUS_IDLE);
        }
    }
}

/**
 * @brief 링 버퍼의 현재 사용량을 계산합니다.
 */
static uint32_t _GetBufferUsedSize(uint32_t head, uint32_t tail)
{
    if (head >= tail) {
        return head - tail;
    } else {
        return (LOG_BUFFER_SIZE - tail) + head;
    }
}

/* [2026-04-18 C안] _GetColdBufferUsedSize / _TransferHotToCold 제거.
 * Hot 만 존재하므로 _GetBufferUsedSize 로 충분. */

/* ====================================================================
 *  [Diag 2026-04-17] FDCAN 세션 통계 Trace API
 *  DataLoggerTask(100ms) 에서 주기적으로 호출 (C안 이전 OffloadTask 20ms 대체).
 * ==================================================================== */
static void _ResetFdcanSessionStats(void)
{
    s_fdcan_fill_max       = 0;
    s_fdcan_bus_off_events = 0;
    /* 현재 상태를 baseline으로 설정 — 과거 bus-off가 '전이'로 잘못 카운트되지 않도록 */
    uint8_t bus_status = 0;
    s_fdcan_was_bus_off =
        (IOIF_FDCAN_GetBusStatus(System_GetFDCAN1_Id(), NULL, &bus_status) == AGRBStatus_OK)
            ? ((bus_status & 0x01u) != 0u) : false;
}

static void _TraceFdcanStats(void)
{
    uint32_t fill = IOIF_FDCAN_GetRxFifo0FillLevel(System_GetFDCAN1_Id());
    if (fill > s_fdcan_fill_max) { s_fdcan_fill_max = (uint8_t)fill; }

    uint8_t bus_status = 0;
    if (IOIF_FDCAN_GetBusStatus(System_GetFDCAN1_Id(), NULL, &bus_status) == AGRBStatus_OK) {
        bool bus_off_now = ((bus_status & 0x01u) != 0u);
        if (bus_off_now && !s_fdcan_was_bus_off) {
            s_fdcan_bus_off_events++;
        }
        s_fdcan_was_bus_off = bus_off_now;
    }
}

static void _CollectFdcanFinalStats(void)
{
    uint8_t tec = 0, rec = 0;
    if (IOIF_FDCAN_GetErrorCounters(System_GetFDCAN1_Id(), &tec, &rec) == AGRBStatus_OK) {
        s_stats.fdcan_rec_end = rec;
        s_stats.fdcan_tec_end = tec;
    }
    uint8_t lec = 0;
    if (IOIF_FDCAN_GetBusStatus(System_GetFDCAN1_Id(), &lec, NULL) == AGRBStatus_OK) {
        s_stats.fdcan_lec_last = lec;
    }
    s_stats.fdcan_rx_fifo0_fill_max = s_fdcan_fill_max;
    s_stats.fdcan_bus_off_events    = s_fdcan_bus_off_events;
}

/* [2026-04-18 C안] _OffloadTask + _FlushHotToCold 제거.
 * DataLoggerTask 가 Hot 을 직접 USB 로 drain — 중간 버퍼 없음. */

/**
 * @brief [2026-04-18 C안] Hot Buffer → USB 파일 쓰기 (DataLoggerTask 100ms 주기).
 * @details Rev1.1 패턴: OffloadTask / Cold Buffer 제거 후 DataLoggerTask 단일 consumer.
 *   1. Hot Buffer 수위 감시 (overflow 시 _EmergencyStop)
 *   2. Hot (D2) → s_read_buf (D1, 32KB) wrap-aware memcpy
 *   3. D-Cache Clean (DMA 전 필수) → _WriteDataWithBlockCRC → USB f_write
 *   4. tail 전진 → 다음 drain cycle 에서 이어서
 * SPSC: UserTask 가 head 만, DataLoggerTask 가 tail 만 갱신 → race-free.
 */
static void _ProcessRingBuffer(void)
{
    uint32_t head = atomic_load(&s_logHead);
    uint32_t tail = s_logTail;
    uint32_t hot_used = _GetBufferUsedSize(head, tail);

    /* --- 1. Hot Buffer 수위 감시 --- */
    if (hot_used > 0) {
        uint32_t usage_percent = (uint32_t)((uint64_t)hot_used * 100 / LOG_BUFFER_SIZE);
        if ((uint8_t)usage_percent > s_stats.hot_buffer_peak_percent) {
            s_stats.hot_buffer_peak_percent = (uint8_t)usage_percent;
        }
        if (usage_percent > BUFFER_STOP_THRESHOLD_PERCENT) {
            /* Hot 98% 초과 → overflow 임박. 재귀 방지: isLoggingActive 가드. */
            if (atomic_load(&s_isLoggingActive)) {
                _EmergencyStop(STOP_REASON_HOT_OVERFLOW);
            }
            return;
        } else if (usage_percent > BUFFER_WARNING_THRESHOLD_PERCENT) {
            atomic_store(&s_logStatus, LOG_STATUS_WARNING_QUEUE_FULL);
        } else {
            atomic_store(&s_logStatus, LOG_STATUS_LOGGING);
        }
    }

    if (hot_used == 0) return;

    uint32_t read_size = (hot_used < MAX_WRITE_CHUNK_SIZE) ? hot_used : MAX_WRITE_CHUNK_SIZE;

    /* --- 2. Hot (D2) → s_read_buf (D1) wrap-aware memcpy ---
     * s_logBuffer (D2) 는 CPU-only 접근 (DMA 없음) → cache 관리 불필요.
     * SCB Clean 은 s_read_buf (D1 Write-Back) 에서 USB OTG DMA 전 수행. */
    uint32_t to_end = LOG_BUFFER_SIZE - tail;
    if (read_size <= to_end) {
        memcpy(s_read_buf, &s_logBuffer[tail], read_size);
    } else {
        memcpy(s_read_buf, &s_logBuffer[tail], to_end);
        memcpy(s_read_buf + to_end, &s_logBuffer[0], read_size - to_end);
    }

    /* --- 3. D-Cache Clean → USB DMA 정합성 확보 */
    SCB_CleanDCache_by_Addr((uint32_t*)s_read_buf, read_size);

    /* [Phase 3d] 4KB 블록 CRC + [Phase 2B] data+CRC consolidation */
    bool write_ok = _WriteDataWithBlockCRC(s_read_buf, read_size);

    if (write_ok) {
        s_logTail = (tail + read_size) % LOG_BUFFER_SIZE;
        s_stats.total_bytes_written += read_size;
        s_part_data_bytes += read_size;
    } else {
        s_stats.write_errors++;
        s_stats.usb_write_errors++;
        _EmergencyStop(STOP_REASON_WRITE_FAILURE);
    }
}

/* ===== [Phase 3a] Emergency Stop — 통합 에러 정리 =====
 * Why: 5개 에러 경로에서 각각 다른 수준의 cleanup → _EmergencyStop으로 통합.
 *      USB 분리 시 FATFS 호출 자동 스킵 (DataLogger_IsReady() == false).
 * Thread-Safety:
 *   - DataLoggerTask 컨텍스트: _EmergencyStop() 직접 호출 (FATFS 안전)
 *   - UserTask 컨텍스트: _RequestEmergencyStop() → flag만 설정 → 다음 사이클 cleanup
 */

/**
 * @brief [DataLoggerTask 컨텍스트] 에러 발생 시 파일 닫기 + summary 기록 + 상태 전환
 * @param[in] reason 종료 사유
 * @note USB 분리 시 DataLogger_IsReady()==false → FATFS 호출 자동 스킵
 */
static void _EmergencyStop(StopReason_e reason)
{
    atomic_store(&s_isLoggingActive, false);
    s_stats.end_tick = xTaskGetTickCount();

    if (DataLogger_IsReady() && s_file_open) {
        /* [2026-04-18 C안] 남은 Hot Buffer 데이터 drain (isLoggingActive=false 이므로
         * UserTask 쓰기 중단 → head 고정 → 유한 시간 내 완료 보장). */
        while (_GetBufferUsedSize(atomic_load(&s_logHead), s_logTail) > 0) {
            _ProcessRingBuffer();
            /* 재진입 방지: _ProcessRingBuffer 내부에서 write 실패 시 _EmergencyStop
             * 재귀 호출되지만 isLoggingActive=false 가드로 탈출. 파일 닫히면 즉시 break. */
            if (!s_file_open) break;
        }

        /* [Phase 3d] 불완전 블록 CRC flush + [Phase 3b] 풋터 기록 후 close */
        _FlushPartialBlockCRC();
        _WriteFileFooter();
        /* ★ Pre-allocation 정리: 실제 데이터 크기로 파일 축소 */
        AGRBFileSystem.truncate(s_currentFileId, NULL);
        AGRBFileSystem.close(s_currentFileId, NULL);
        s_file_open = false; /* [Q6] close 후 상태 갱신 */
        /* [Step 0 2026-04-18] Summary write 이전에 histogram freeze */
        DiagPerf_Finalize();
        _WriteSummary_WithStatus(reason);
        _WriteDiagProfile();
    }
    /* USB 분리 시 f_close 호출 금지 (매체 없으므로 hang 위험).
     * ioif_agrb_fs.c HandleHostEvent(false)가 이미 f_mount(NULL) 처리. */

    atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
}

/**
 * @brief [UserTask 컨텍스트] flag만 설정 — FATFS 접근 금지.
 *        DataLoggerTask 다음 사이클에서 _EmergencyStop() 수행.
 * @param[in] reason 종료 사유
 */
static void _RequestEmergencyStop(StopReason_e reason)
{
    s_pending_stop_reason = reason;
    atomic_store(&s_isLoggingActive, false);
    atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
}

/**
 * @brief 링 버퍼에 데이터 쓰기 (Wrap-around 자동 처리)
 */
static void _RingBufferWrite(uint32_t* idx, const uint8_t* data, uint32_t size)
{
    uint32_t pos = *idx;
    uint32_t to_end = LOG_BUFFER_SIZE - pos;

    if (size <= to_end) {
        memcpy(&s_logBuffer[pos], data, size);
    } else {
        memcpy(&s_logBuffer[pos], data, to_end);
        memcpy(&s_logBuffer[0], data + to_end, size - to_end);
    }
    *idx = (pos + size) % LOG_BUFFER_SIZE;
}

/* ===== [Phase 3a] _WriteSummary → _WriteSummary_WithStatus 확장 =====
 * [AS-IS] summary.txt에 통계만 기록
 * [TO-BE] status= (OK/ERROR) + error_reason= 추가 → 비정상 종료 원인 추적 가능
 * [Impact] 기존 summary.txt 포맷에 2줄 추가 (하위 호환 유지) */

/** @brief StopReason_e → 사람이 읽을 수 있는 문자열 */
static const char* _StopReasonToString(StopReason_e reason)
{
    switch (reason) {
        case STOP_REASON_NORMAL:          return "NORMAL";
        case STOP_REASON_HOT_OVERFLOW:    return "HOT_BUFFER_OVERFLOW";
        case STOP_REASON_USB_DISCONNECT:  return "USB_DISCONNECT";
        case STOP_REASON_COLD_OVERFLOW:   return "COLD_BUFFER_OVERFLOW";
        case STOP_REASON_WRITE_FAILURE:   return "WRITE_FAILURE";
        case STOP_REASON_ROLLING_FAILURE: return "ROLLING_FAILURE";
        default:                          return "UNKNOWN";
    }
}

/**
 * @brief 세션 종료 시 summary.txt를 자동 생성합니다.
 * @param[in] reason 종료 사유 (STOP_REASON_NORMAL = 정상)
 */
static void _WriteSummary_WithStatus(StopReason_e reason)
{
    IOIF_FILEx_t file_id;
    char path[MAX_FULL_PATH_SIZE];
    snprintf(path, sizeof(path), "%s/summary.txt", s_currentSessionPath);

    if (AGRBFileSystem.open_write(&file_id, path, IOIF_FileSystem_CreateMode_OVERWRITE, NULL) != IOIF_FileSystem_OK) {
        return;
    }

    uint32_t duration_ms = s_stats.end_tick - s_stats.start_tick;

    /* [Phase 3b M5] 종료 시 RTC 시각 캡처 (1회 SPI, ~5ms) */
    XmDateTime_t rtc_end = {0};
    XM_RTC_GetDateTime(&rtc_end);

    /* [Diag 2026-04-17] USBH_MSC polling/write 실측 스냅샷.
     * Why ms 단위로 스케일: newlib-nano는 %llu 미지원 → uint64_t 직접 출력 불가.
     * 80s 세션 × 200µs/call 평균이어도 ms sum은 수십만 수준 → uint32_t 여유. */
    USBH_DiskIO_Diag_t usbh_diag;
    USBH_DiskIO_GetDiag(&usbh_diag);
    s_stats.usbh_polling_entries   = usbh_diag.polling_entries;
    s_stats.usbh_polling_iters_max = usbh_diag.polling_iters_max;
    s_stats.usbh_write_us_max      = usbh_diag.write_us_max;
    s_stats.usbh_write_us_sum      = usbh_diag.write_us_sum;
    uint32_t usbh_write_us_avg =
        (usbh_diag.write_call_count > 0)
            ? (uint32_t)(usbh_diag.write_us_sum / usbh_diag.write_call_count)
            : 0;
    uint32_t usbh_write_ms_sum = (uint32_t)(usbh_diag.write_us_sum / 1000ULL);

    /* [Diag 2026-04-17] CM PDO 연속성 진단 스냅샷 (CAN 수신 품질).
     * Python 디코더의 h10_loop_count 분석(USB 저장 후)과 교차검증용. */
    CM_PdoDiag_t pdo_diag;
    CM_GetPdoDiag(&pdo_diag);
    uint32_t pdo_total = pdo_diag.normal_count + pdo_diag.dup_count
                       + pdo_diag.gap_count + pdo_diag.wrap_count;

    /* [Diag 2026-04-17] FDCAN 세션 종료 시점 통계 확정 */
    _CollectFdcanFinalStats();

    /* [2026-04-18] session_index / prev_session_stop_reason:
     * Emergency stop → app auto-restart 체인 추적용. session 000 실패 → 001 재시작 시
     * 001 summary 의 prev_session_stop_reason 필드로 000 의 실패 사유 확인 가능. */
    const char* prev_stop_str = (s_prev_session_stop_reason == STOP_REASON_NORMAL)
                              ? "NONE" : _StopReasonToString(s_prev_session_stop_reason);

    char buf[1024];
    int len = snprintf(buf, sizeof(buf),
        "=== Session Summary ===\n"
        "session_index=%lu\n"
        "prev_session_stop_reason=%s\n"
        "status=%s\n"
        "error_reason=%s\n"
        "rtc_start=%04u-%02u-%02u %02u:%02u:%02u\n"
        "rtc_end=%04u-%02u-%02u %02u:%02u:%02u\n"
        "start_tick=%lu\n"
        "end_tick=%lu\n"
        "duration_ms=%lu\n"
        "total_packets=%lu\n"
        "total_bytes=%lu\n"
        "dropped_packets=%lu\n"
        "write_errors=%lu\n"
        "sync_count=%lu\n"
        "file_count=%lu\n"
        "hot_buffer_peak_percent=%u\n"
        "disk_free_mb=%lu\n"
        "disk_total_mb=%lu\n"
        "usbh_polling_entries=%lu\n"
        "usbh_polling_iters_max=%lu\n"
        "usbh_write_us_max=%lu\n"
        "usbh_write_ms_sum=%lu\n"
        "usbh_write_call_count=%lu\n"
        "usbh_write_us_avg=%lu\n"
        "usbh_write_burst_max_us=%lu\n"
        "pdo_total=%lu\n"
        "pdo_normal=%lu\n"
        "pdo_dup=%lu\n"
        "pdo_gap=%lu\n"
        "pdo_wrap=%lu\n"
        "pdo_missed_cycles=%lu\n"
        "pdo_gap_max_burst=%lu\n"
        "pdo_gap_hist_1=%lu\n"
        "pdo_gap_hist_2=%lu\n"
        "pdo_gap_hist_3=%lu\n"
        "pdo_gap_hist_4=%lu\n"
        "pdo_gap_hist_5plus=%lu\n"
        "fdcan_rec_end=%u\n"
        "fdcan_tec_end=%u\n"
        "fdcan_lec_last=%u\n"
        "fdcan_rx_fifo0_fill_max=%u\n"
        "fdcan_bus_off_events=%lu\n"
        "usb_write_errors=%lu\n",
        (unsigned long)s_currentSessionIndex,
        prev_stop_str,
        (reason == STOP_REASON_NORMAL) ? "OK" : "ERROR",
        _StopReasonToString(reason),
        s_session_rtc_start.year, s_session_rtc_start.month, s_session_rtc_start.day,
        s_session_rtc_start.hour, s_session_rtc_start.minute, s_session_rtc_start.second,
        rtc_end.year, rtc_end.month, rtc_end.day,
        rtc_end.hour, rtc_end.minute, rtc_end.second,
        (unsigned long)s_stats.start_tick,
        (unsigned long)s_stats.end_tick,
        (unsigned long)duration_ms,
        (unsigned long)s_stats.total_packets_logged,
        (unsigned long)s_stats.total_bytes_written,
        (unsigned long)s_stats.dropped_packets,
        (unsigned long)s_stats.write_errors,
        (unsigned long)s_stats.sync_count,
        (unsigned long)(s_currentSplitIndex + 1),
        (unsigned)s_stats.hot_buffer_peak_percent,
        (unsigned long)s_cached_disk_free_mb,
        (unsigned long)s_cached_disk_total_mb,
        (unsigned long)s_stats.usbh_polling_entries,
        (unsigned long)s_stats.usbh_polling_iters_max,
        (unsigned long)s_stats.usbh_write_us_max,
        (unsigned long)usbh_write_ms_sum,
        (unsigned long)usbh_diag.write_call_count,
        (unsigned long)usbh_write_us_avg,
        (unsigned long)usbh_diag.write_burst_max_us,
        (unsigned long)pdo_total,
        (unsigned long)pdo_diag.normal_count,
        (unsigned long)pdo_diag.dup_count,
        (unsigned long)pdo_diag.gap_count,
        (unsigned long)pdo_diag.wrap_count,
        (unsigned long)pdo_diag.missed_cycles,
        (unsigned long)pdo_diag.gap_max_burst,
        (unsigned long)pdo_diag.gap_hist[0],
        (unsigned long)pdo_diag.gap_hist[1],
        (unsigned long)pdo_diag.gap_hist[2],
        (unsigned long)pdo_diag.gap_hist[3],
        (unsigned long)pdo_diag.gap_hist[4],
        (unsigned)s_stats.fdcan_rec_end,
        (unsigned)s_stats.fdcan_tec_end,
        (unsigned)s_stats.fdcan_lec_last,
        (unsigned)s_stats.fdcan_rx_fifo0_fill_max,
        (unsigned long)s_stats.fdcan_bus_off_events,
        (unsigned long)s_stats.usb_write_errors);

    _log_write(file_id, (uint8_t*)buf, (uint32_t)len, NULL);
    AGRBFileSystem.close(file_id, NULL);

    /* [2026-04-18] 다음 세션용 체인 갱신. Emergency stop 후 재시작 시 다음 세션의
     * summary prev_session_stop_reason 필드로 이 세션의 reason 노출. */
    s_prev_session_stop_reason = reason;
}

/**
 * @brief [Step 0] 세션 종료 시 diag_profile.txt 작성.
 * @note  DiagPerf_Finalize() 이후 호출. DIAG_PROFILE_ENABLED 미정의 시 no-op stub 가
 *        0 반환하므로 파일 생성 스킵.
 */
static void _WriteDiagProfile(void)
{
    /* Step A-1 확장 시 per-ISR 5개 필드 × 10 ISRs + category 등으로 ~4KB 필요.
     * Stack 부담 회피 위해 static 으로 배치 (세션당 1회 호출, 재진입 없음). */
    static char buf[4096];
    int  len = DiagPerf_FormatReport(buf, (int)sizeof(buf));
    if (len <= 0) return;  /* DIAG_PROFILE_ENABLED=OFF 이면 stub 가 0 반환 */

    IOIF_FILEx_t file_id;
    char path[MAX_FULL_PATH_SIZE];
    snprintf(path, sizeof(path), "%s/diag_profile.txt", s_currentSessionPath);

    if (AGRBFileSystem.open_write(&file_id, path, IOIF_FileSystem_CreateMode_OVERWRITE, NULL)
            != IOIF_FileSystem_OK) {
        return;
    }
    _log_write(file_id, (uint8_t*)buf, (uint32_t)len, NULL);
    AGRBFileSystem.close(file_id, NULL);
}


/* ─────────────────────────────────────────────────────────────
 * [Step B 재설계 #1] Pending HardFault dump → /LOGS/hardfault_<uptime>.txt
 *
 * 동작:
 *   - main() 의 HardFault_LoadBootDump() 가 이미 `.noinit` → 모듈 static 이관
 *   - 본 함수는 USB 마운트 완료 후 첫 호출에서 파일 기록 (1-shot)
 *   - HasBootDump() false 면 즉시 return — idle 사이클 오버헤드 0
 *   - 파일 경로: /LOGS/hardfault_<uptime_ms>.txt (RTC 캐시가 아직 없을 수도 있어 uptime 기반)
 * ───────────────────────────────────────────────────────────── */
static void _TryWritePendingFaultDump(void)
{
    if (!HardFault_HasBootDump()) {
        return;
    }

    HardFaultDump_t dump;
    if (!HardFault_TakeBootDump(&dump)) {
        return;
    }

    /* 포맷된 텍스트 — stack 1.5KB, 포맷 최대 ~900B */
    char buf[1536];
    const uint32_t len = HardFault_FormatReport(&dump, buf, sizeof(buf));
    if (len == 0) {
        return;
    }

    /* /LOGS 디렉터리 확보 — 이미 있으면 OK */
    (void)AGRBFileSystem.mkdir(LOG_DIR_PATH, NULL);

    char path[MAX_FULL_PATH_SIZE];
    snprintf(path, sizeof(path), "%s/hardfault_%lu.txt",
             LOG_DIR_PATH, (unsigned long)dump.uptime_ms);

    IOIF_FILEx_t file_id = IOIF_FILE_INVALID_ID;
    if (AGRBFileSystem.open_write(&file_id, path,
                                  IOIF_FileSystem_CreateMode_OVERWRITE, NULL)
            != IOIF_FileSystem_OK) {
        return;
    }
    _log_write(file_id, (const uint8_t*)buf, len, NULL);
    AGRBFileSystem.close(file_id, NULL);
}
