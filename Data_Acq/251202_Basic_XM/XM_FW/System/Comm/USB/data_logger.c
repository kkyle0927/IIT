/**
 ******************************************************************************
 * @file    data_logger.c
 * @author  HyundoKim
 * @brief   [System Layer] 저순위 USB MSC 로깅 태스크 (Consumer) 구현
 * @details
 * - 2-Stage 큐 방식에서 Lock-Free SPSC 링 버퍼 방식으로 변경.
 * - 2ms UserTask(Producer)는 링 버퍼에 memcpy 후 atomic_store(head)만 수행.
 * - DataLoggerTask(Consumer)는 링 버퍼에서 데이터를 읽어(tail) f_write 수행.
 * - f_write가 멈추더라도 UserTask는 링 버퍼가 꽉 찰 때까지 영향을 받지 않음.
 * @version 0.1
 * @date    Nov 10, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "data_logger.h"
#include "usb_mode_handler.h" // USB 상태 플래그(g_isUsbHostReady) 사용
#include "ioif_agrb_fs.h"     // IOIF 파일 시스템 API
#include <stdio.h>
#include <string.h>
#include <stdatomic.h> // [신규] 원자적 연산 사용

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

// --- 1. RAM 링 버퍼 (댐) 설정 ---
// D2 RAM (288KB) 중 280KB를 로깅 버퍼로 사용
// 필요 대역폭 300Bytes x 500Hz = 150kB/s
// 버퍼 용량 280kB의 USB write시의 가비지 컬렉션에 의한 멈춤 시간을 버티는 시간 280/150 = 대략 1.86초 
// USB 스틱의 일반적인 최대 지연이 0.5초 미만이므로 1.86초는 안정적인 마진
#define LOG_BUFFER_SIZE         (280 * 1024)

// --- 2. 로깅 태스크 주기 ---
#define LOGGER_TASK_PERIOD_MS   (100) // 100ms마다 댐 수위 확인 및 방류(f_write)

// --- 3. 링 버퍼 위험 수위 (에러 감지용) ---
#define BUFFER_WARNING_THRESHOLD_PERCENT (90) // 90%
#define BUFFER_STOP_THRESHOLD_PERCENT    (98) // 98% (오버플로우 직전)

// --- 4. 배치 쓰기(f_write) 단위 ---
#define MAX_WRITE_CHUNK_SIZE    (32 * 1024) // 한 번에 최대 32KB씩 쓰기

// 2단계 큐 (API -> DataLoggerTask)
#define CMD_QUEUE_SIZE          (5)     // Start/Stop 명령 5개 버퍼링
#define MAX_METADATA_SIZE       (2048)

// [수정 1] 최종 경로를 위한 버퍼 크기를 512으로 늘림
#define MAX_PATH_SIZE           (512)
// [신규] End-User가 입력할 세션 이름의 길이를 100자로 제한
#define MAX_SESSION_NAME_SIZE   (100)

#define LOG_FILE_ROLLING_SIZE_MB (10) // 10MB 마다 파일 롤링
#define LOG_DIR_PATH            "/LOGS"

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

// --- [신규] Lock-Free SPSC 링 버퍼 및 포인터 ---
// D2 RAM (288KB) 영역에 128KB 링 버퍼를 할당하고,
// 이 영역이 Non-Cacheable DMA 섹션임을 링커에게 알림
__attribute__((section(IOIF_FS_SECTION)))
static uint8_t s_logBuffer[LOG_BUFFER_SIZE];

// Head: UserTask(Producer)가 씀. Atomic 필수.
static volatile atomic_uint s_logHead = 0; 
// Tail: DataLoggerTask(Consumer)가 씀. 
// (더 높은 우선순위 태스크가 접근하지 않으므로 atomic 불필요)
static volatile uint32_t s_logTail = 0;

// --- 상태 변수 ---
static volatile atomic_bool s_isLoggingActive = false;
static volatile atomic_uint s_logStatus = LOG_STATUS_IDLE;

static IOIF_FILEx_t s_currentFileId;
static char s_currentSessionPath[MAX_PATH_SIZE];
// 세션 인덱스 (data_XXX)
static uint32_t s_currentSessionIndex = 0; 
// 파일 분할 인덱스 (part_YYY)
static uint32_t s_currentSplitIndex = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void StartDataLoggerTask(void* argument);
static bool _CheckUsbSpace(void);
static bool _InitSensorLogSession(const char* sessionName, const char* metadata);
static bool _CheckFileRolling(void);
static void _HandleCommandQueue(void);
static void _ProcessRingBuffer(void);
static uint32_t _GetBufferUsedSize(uint32_t head, uint32_t tail);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void DataLogger_Init(void)
{
    // 2. 2단계 큐 (명령 큐) 생성
    g_logCmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(LoggerCommand_t));

    // 링 버퍼 포인터 및 상태 초기화
    atomic_store(&s_logHead, 0);
    s_logTail = 0;
    atomic_store(&s_isLoggingActive, false);
    atomic_store(&s_logStatus, LOG_STATUS_IDLE);

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
    if (!g_isUsbHostReady || sessionName == NULL || metadata == NULL) return false;

    // 런타임 길이 검증
    // sessionName이 너무 길면 버퍼 오버플로우가 발생하므로 즉시 거부
    if (strlen(sessionName) > MAX_SESSION_NAME_SIZE) {
        return false; // Error: Session name is too long
    }
    if (strlen(metadata) >= MAX_METADATA_SIZE) {
        return false; // Error: Metadata is too long
    }

    LoggerCommand_t cmd = {0};
    cmd.command = LOG_CMD_START_SESSION;
    strncpy(cmd.sessionName, sessionName, MAX_SESSION_NAME_SIZE);
    strncpy(cmd.metadata, metadata, MAX_METADATA_SIZE - 1);
    
    // 2단계 큐에 "로깅 시작" 명령 전송 (100ms 타임아웃)
    return (xQueueSend(g_logCmdQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE);
}

void DataLogger_Stop(void)
{
    LoggerCommand_t cmd = {0};
    cmd.command = LOG_CMD_STOP_SESSION;
    xQueueSend(g_logCmdQueue, &cmd, 0); // 비차단
}

/**
 * @brief [실시간] 2ms UserTask가 링 버퍼에 직접 쓰기 (Lock-Free)
 */
bool DataLogger_Log(const void* logPacket, uint32_t packetSize)
{
    if (!atomic_load(&s_isLoggingActive)) return false;
    
    // [수정] packetSize가 0이거나 MAX_LOG_PACKET_SIZE를 초과하면 무시
    if (packetSize == 0 || packetSize > MAX_LOG_PACKET_SIZE) return false; 

    // 1. 현재 링 버퍼 포인터 읽기 (원자적)
    uint32_t head = atomic_load(&s_logHead);
    uint32_t tail = s_logTail; // Prio 16 태스크 값은 volatile 읽기
    
    // 2. 링 버퍼 남은 공간 확인
    uint32_t used_size = _GetBufferUsedSize(head, tail);
    uint32_t free_space = LOG_BUFFER_SIZE - used_size - 1; // 1바이트 여유
    
    // 3. 쓸 데이터 크기 = 헤더(크기 정보) + 실제 데이터
    uint32_t total_write_size = sizeof(LogPacketHeader_t) + packetSize;
    
    if (total_write_size > free_space) {
        // [비상 조치] 링 버퍼 오버플로우 발생!
        atomic_store(&s_isLoggingActive, false);
        atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
        return false;
    }

    // 4. 데이터 쓰기 (Wrap-around 고려)
    uint32_t idx = head;
    LogPacketHeader_t header = { .packetSize = packetSize };

    // 4a. 헤더(packetSize) 쓰기
    uint8_t* header_ptr = (uint8_t*)&header;
    for(int i=0; i<sizeof(LogPacketHeader_t); i++) {
        s_logBuffer[idx] = header_ptr[i];
        idx = (idx + 1) % LOG_BUFFER_SIZE;
    }

    // 4b. 실제 패킷 데이터(logPacket) 쓰기
    uint8_t* data_ptr = (uint8_t*)logPacket;
    for(int i=0; i<packetSize; i++) {
        s_logBuffer[idx] = data_ptr[i];
        idx = (idx + 1) % LOG_BUFFER_SIZE;
    }
    
    // 5. Head 포인터를 원자적으로 갱신하여 "쓰기 완료(Commit)"
    atomic_store(&s_logHead, idx);
    return true;
}

DataLogger_Status_e DataLogger_GetStatus(void)
{
    return (DataLogger_Status_e)atomic_load(&s_logStatus);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief [핵심] 저순위(Prio 16) 데이터 저장 태스크 (100ms 주기)
 */
static void StartDataLoggerTask(void* argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(LOGGER_TASK_PERIOD_MS);

    for (;;) {
        // 2. [신규] IOIF 계층에 마운트 시도 위임
        // (이 함수는 내부적으로 s_is_mount_pending 플래그를 확인)
        AGRBFileSystem.process_mount();
        
        // 3. 마운트가 완료되었는지 확인 (DataLogger_IsReady()가 s_is_fs_ready를 체크)
        if (!DataLogger_IsReady()) {
            if (atomic_load(&s_isLoggingActive)) {
                // (마운트가 해제되면 로깅 중지)
                atomic_store(&s_isLoggingActive, false);
                atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
            }
            continue;
        }

        // 4. (마운트 완료) 비실시간 명령(Start/Stop) 처리
        _HandleCommandQueue();

        // 5. (마운트 완료) 링 버퍼의 데이터를 파일로 쓰기
        if (atomic_load(&s_isLoggingActive)) {
            _ProcessRingBuffer();
        }

        // 1. 100ms 주기로 깨어남
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
        return (free_mb > LOG_FILE_ROLLING_SIZE_MB); // 최소 10MB 필요
    }
    return false;
}

/**
 * @brief 세션 시작 시 빈 '세션 번호'를 찾고 part_000 파일 생성
 */
static bool _InitSensorLogSession(const char* sessionName, const char* metadata)
{
    IOIF_FILEx_t file_id;
    char path[MAX_PATH_SIZE];

    // 1. 기본 로그 디렉터리 생성
    AGRBFileSystem.mkdir(LOG_DIR_PATH, NULL); // "/LOGS"

    // 2. 세션 디렉터리 생성 (예: "/LOGS/S_001")
    snprintf(s_currentSessionPath, MAX_PATH_SIZE, "%s/%s", LOG_DIR_PATH, sessionName);
    AGRBFileSystem.mkdir(s_currentSessionPath, NULL);

    // 3. 메타데이터 파일 저장 (예: "/LOGS/S_001/metadata.txt")
    snprintf(path, MAX_PATH_SIZE, "%s/metadata.txt", s_currentSessionPath);
    if (AGRBFileSystem.open_write(&file_id, path, IOIF_FileSystem_CreateMode_OVERWRITE, NULL) == IOIF_FileSystem_OK) {
        AGRBFileSystem.write(file_id, (uint8_t*)metadata, strlen(metadata), NULL);
        AGRBFileSystem.close(file_id, NULL);
    } else {
        return false; // 메타데이터 쓰기 실패
    }

    // 4. 이진 탐색으로 '빈 세션 번호(data_XXX)' 찾기 찾기 (Max 10회 시도)
    uint32_t low = 0;
    uint32_t high = 1000; // 최대 파일 개수 제한
    uint32_t mid = 0;
    IOIF_FILEx_t temp_id;
    char temp_path[MAX_PATH_SIZE];

    // "data_XXX_part_000.bin"이 존재하는지 확인하여 세션 번호 결정
    while (low < high) {
        mid = (low + high) / 2;

        // 확인용 경로: .../data_mid_part_000.bin
        snprintf(temp_path, MAX_PATH_SIZE, "%s/data_%03lu_part_000.bin", s_currentSessionPath, mid);
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
    char final_path[MAX_PATH_SIZE];
    snprintf(final_path, MAX_PATH_SIZE, "%s/data_%03lu_part_%03lu.bin", 
             s_currentSessionPath, s_currentSessionIndex, s_currentSplitIndex);

    if (AGRBFileSystem.open_write(&s_currentFileId, final_path, IOIF_FileSystem_CreateMode_EXCLUSIVE, NULL) != IOIF_FileSystem_OK) {
        return false;
    }

    return true;
}

/**
 * @brief 파일 용량이 10MB를 넘으면 다음 part 파일 생성
 */
static bool _CheckFileRolling(void)
{
    uint32_t current_size_mb = 0;
    
    // 현재 파일 크기 확인
    if (AGRBFileSystem.get_size_mb(s_currentFileId, &current_size_mb) != IOIF_FileSystem_OK) {
        return false;
    }

    // 10MB 미만이면 통과
    if (current_size_mb < LOG_FILE_ROLLING_SIZE_MB) {
        return true;
    }

    // --- 파일 롤링(교체) 시작 ---
    // 10MB가 넘으면 새 파일 열기
    if (current_size_mb >= LOG_FILE_ROLLING_SIZE_MB) {
        AGRBFileSystem.close(s_currentFileId, NULL);
        
        // 2. 분할 인덱스 증가
        s_currentSplitIndex++;
        
        // 3. 다음 파일 경로 생성 (data_XXX_part_001.bin ...)
        char next_path[MAX_PATH_SIZE];
        snprintf(next_path, MAX_PATH_SIZE, "%s/data_%03lu_part_%03lu.bin", 
                s_currentSessionPath, s_currentSessionIndex, s_currentSplitIndex);
        
        // 4. 새 파일 열기
        if (AGRBFileSystem.open_write(&s_currentFileId, next_path, IOIF_FileSystem_CreateMode_EXCLUSIVE, NULL) != IOIF_FileSystem_OK) {
            atomic_store(&s_isLoggingActive, false); // 실패 시 로깅 중단
            atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
            return false;
        }
    }
    return true;
}

/**
 * @brief [신규] 링 버퍼의 데이터를 f_write로 비우는 함수 (핵심 로직)
 */
static void _ProcessRingBuffer(void)
{
    uint32_t head = atomic_load(&s_logHead);
    uint32_t tail = s_logTail;
    uint32_t used_size = _GetBufferUsedSize(head, tail);

    // --- 1. 버퍼 수위 감시 (에러 처리) ---
    uint32_t usage_percent = (used_size * 100) / LOG_BUFFER_SIZE;
    
    if (usage_percent > BUFFER_STOP_THRESHOLD_PERCENT) {
        atomic_store(&s_isLoggingActive, false);
        atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
        AGRBFileSystem.close(s_currentFileId, NULL);
        return;
    } else if (usage_percent > BUFFER_WARNING_THRESHOLD_PERCENT) {
        atomic_store(&s_logStatus, LOG_STATUS_WARNING_QUEUE_FULL);
    } else {
        atomic_store(&s_logStatus, LOG_STATUS_LOGGING);
    }

    // --- 2. 배치 쓰기 (Batch Write) ---
    if (used_size == 0) return; // 쓸 데이터 없음

    // 한 번에 16KB(MAX_WRITE_CHUNK_SIZE)까지만 쓰기
    // [수정] 단, 패킷 헤더를 읽어 패킷이 중간에 잘리지 않도록 함
    uint32_t write_size = 0;
    uint32_t temp_tail = tail;

    while (write_size < MAX_WRITE_CHUNK_SIZE) {
        uint32_t remaining_in_buffer = _GetBufferUsedSize(head, temp_tail);
        if (remaining_in_buffer < sizeof(LogPacketHeader_t)) {
            break; // 헤더조차 읽을 수 없음
        }

        // 1. 헤더 읽기 (Wrap-around 고려)
        LogPacketHeader_t header;
        uint8_t* header_ptr = (uint8_t*)&header;
        for (int i = 0; i < sizeof(LogPacketHeader_t); i++) {
            header_ptr[i] = s_logBuffer[temp_tail];
            temp_tail = (temp_tail + 1) % LOG_BUFFER_SIZE;
        }
        
        // 2. 패킷 전체가 버퍼에 있는지 확인
        if (header.packetSize > (remaining_in_buffer - sizeof(LogPacketHeader_t))) {
            break; // 패킷 데이터가 아직 다 안 쓰임
        }

        // 3. 이번에 쓸 총 크기에 추가
        uint32_t total_packet_size = sizeof(LogPacketHeader_t) + header.packetSize;
        if (write_size + total_packet_size > MAX_WRITE_CHUNK_SIZE) {
            break; // 청크 크기 초과
        }

        write_size += total_packet_size;
        temp_tail = (temp_tail + header.packetSize) % LOG_BUFFER_SIZE;
    }
    
    if (write_size == 0) return; // 쓸 데이터 없음

    // --- 3. 실제 파일 쓰기 (f_write) ---
    FRESULT res;
    if ( (s_logTail + write_size) > LOG_BUFFER_SIZE ) {
        // 데이터가 버퍼 끝에서 나뉘는 경우 (Wrap-around)
        uint32_t first_chunk = LOG_BUFFER_SIZE - s_logTail;
        uint32_t second_chunk = write_size - first_chunk;
        
        AGRBFileSystem.write(s_currentFileId, &s_logBuffer[s_logTail], first_chunk, &res);
        if(res == FR_OK) {
            AGRBFileSystem.write(s_currentFileId, &s_logBuffer[0], second_chunk, &res);
        }
        s_logTail = second_chunk; // tail 갱신
    } else {
        // 데이터가 연속된 경우
        AGRBFileSystem.write(s_currentFileId, &s_logBuffer[s_logTail], write_size, &res);
        s_logTail = (s_logTail + write_size) % LOG_BUFFER_SIZE; // tail 갱신
    }
    
    if (res != FR_OK) {
        // 쓰기 실패 (USB 뽑힘 등)
        atomic_store(&s_isLoggingActive, false);
        atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
        AGRBFileSystem.close(s_currentFileId, NULL);
    } else {
        // [수정] f_sync()를 매번 호출하면 느리므로 _CheckFileRolling() 내부에서 처리 (주기적)
        _CheckFileRolling(); 
    }
}

/**
 * @brief 2단계 큐(명령 큐)를 확인하고 처리하는 헬퍼
 */
static void _HandleCommandQueue(void)
{
    LoggerCommand_t cmdBuffer;
    
    // 비차단으로 명령 큐 확인
    if (xQueueReceive(g_logCmdQueue, &cmdBuffer, 0) == pdTRUE) {
        if (cmdBuffer.command == LOG_CMD_START_SESSION) {
            
            // 만약 이전 로깅이 비정상 종료되었다면 파일 닫기
            if(atomic_load(&s_isLoggingActive)) {
                 AGRBFileSystem.close(s_currentFileId, NULL);
            }

            if (DataLogger_IsReady() && _CheckUsbSpace()) {
                if (_InitSensorLogSession(cmdBuffer.sessionName, cmdBuffer.metadata)) {
                    // 링 버퍼 포인터 리셋
                    atomic_store(&s_logHead, 0);
                    s_logTail = 0;
                    atomic_store(&s_isLoggingActive, true);
                    atomic_store(&s_logStatus, LOG_STATUS_LOGGING);
                } else {
                    atomic_store(&s_logStatus, LOG_STATUS_ERROR_STOPPED);
                }
            }
        } 
        else if (cmdBuffer.command == LOG_CMD_STOP_SESSION) {
            if (atomic_load(&s_isLoggingActive)) {
                // 남은 데이터 마저 쓰기
                _ProcessRingBuffer(); 
                AGRBFileSystem.close(s_currentFileId, NULL);
            }
            atomic_store(&s_isLoggingActive, false);
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
        return head - tail; // H: 100, T: 80 -> Used: 20
    } else {
        return (LOG_BUFFER_SIZE - tail) + head; // H: 20, T: 80 -> Used: (1024-80)+20 = 264
    }
}
