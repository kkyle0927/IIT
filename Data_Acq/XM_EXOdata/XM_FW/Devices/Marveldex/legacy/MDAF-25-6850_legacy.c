/**
 ******************************************************************************
 * @file    mdaf-25-6850.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Nov 5, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "MDAF-25-6850.h"
#include <string.h>

// FreeRTOS 큐 및 IOIF Tick을 위해 헤더 포함
#include "cmsis_os2.h"
#include "ioif_agrb_tim.h" // IOIF_GetTick() 사용

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

// #define MARVELDEX_PACKET_BUFFER_MAX (64)
// #define MAX_SENSOR_NUM (2)
// #define MAX_MESSAGE_QUEUE_SIZE (16)

// #if (MAX_SENSOR_NUM > IOIF_UART_MAX_INSTANCES)
//     #error "Too many sensor!"
// #endif

#define QUEUE_SIZE          (16)    // 16개 패킷 버퍼링
#define FSR_HEADER_0        (0xFF)
#define FSR_HEADER_1        (0xFF)
#define FSR_FOOTER          (0xFE)

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief FSR 패킷 프레이밍(Framing) 상태
 */
typedef enum {
    STATE_WAIT_HEADER_0,    // 0xFF 대기
    STATE_WAIT_HEADER_1,    // 0xFF 대기
    STATE_COLLECT_PAYLOAD   // 나머지 26바이트 수집
} ParseState_t;

// typedef enum {
//     PACKET_IDLE = 0,
//     PACKET_START,
//     PACKET_END,
//     PACKET_ERROR,
// } ParseStage_e;

// typedef enum {
//     PACKET_RAW_IDLE = 0,    //OxFF
//     PACKET_RAW_START_0,     //0xFF
//     PACKET_RAW_START_1,     //0x00
//     PACKET_RAW_START_2,     //0X01
//     PACKET_RAW_DATA_PUSH,   //COLLECT DATA
// } ParseStage_raw_e;

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

static IOIF_UARTx_t _instance[MAX_SENSOR_NUM]; // [0]: 왼발, [1]: 오른발
static osMessageQueueId_t _queue = NULL;

// 각 센서(UART 인스턴스)별 파싱 상태를 독립적으로 관리
static ParseState_t _parseState[MAX_SENSOR_NUM] = {STATE_WAIT_HEADER_0, STATE_WAIT_HEADER_0};
static MarvelDex_raw_packet_t _rx_buffer[MAX_SENSOR_NUM];
static uint8_t _rx_index[MAX_SENSOR_NUM] = {0, 0};

// static uint32_t sensor_map[IOIF_UART_MAX_INSTANCES];

// static osMessageQueueId_t _queue;
static const osMessageQueueAttr_t _attr = {
    .name = "MarvelDex",
};

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static bool initialize(IOIF_UARTx_t id0, IOIF_UARTx_t id1);
static bool get_value(MarvelDex_packet_t* output);
static int32_t get_sensor_index(IOIF_UARTx_t id);
static void callback(uint8_t* rx_buf, uint32_t size, uint32_t id);
static bool parsing(MarvelDex_raw_packet_t* raw_packet, MarvelDex_packet_t* output);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

MarvelDexFSR_t marvelDexFSR = {
    .init = initialize,
    .get = get_value,
};

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

//초기화 함수. 태스크가 시작하기 전 또는 크리티컬 섹션 등 안전한 상태에서 호출 권장
/**
 * @brief 드라이버 초기화 (System Layer에서 호출)
 */
/**
 * @brief 드라이버 초기화 (System Layer에서 호출)
 */
static bool initialize(IOIF_UARTx_t id0, IOIF_UARTx_t id1)
{
    _instance[0] = id0;
    _instance[1] = id1;

    // 파싱이 완료된 패킷을 담을 RTOS 큐 생성
    _queue = osMessageQueueNew(QUEUE_SIZE, sizeof(MarvelDex_packet_t), &_attr);
    if (_queue == NULL) {
        return false; // 큐 생성 실패
    }

    // IOIF UART 계층에 'callback' 함수를 등록
    ioif_uart_update_rx_callback(id0, callback);
    ioif_uart_update_rx_callback(id1, callback);

    return true;
}

// static bool initialize(IOIF_UARTx_t id0, IOIF_UARTx_t id1)
// {
//     _queue = osMessageQueueNew(QUEUE_SIZE, sizeof(MarvelDex_packet_t), &_attr);

//     if (_queue == NULL) return false;
    
//     memset(sensor_map, -1, sizeof(sensor_map));

//     if ( id0 < IOIF_UART_MAX_INSTANCES && id0 != IOIF_UART_ID_NOT_ALLOCATED ) sensor_map[id0] = 0;
//     if ( id1 < IOIF_UART_MAX_INSTANCES && id1 != IOIF_UART_ID_NOT_ALLOCATED ) sensor_map[id1] = 1;

//     ioif_uart_update_rx_callback(id0, callback);
//     ioif_uart_update_rx_callback(id1, callback);

//     return true;
// }

//데이터를 가져가는 함수. freeRTOS 구조를 몰라도 사용할수 있게 할때 씀. ThreadSafe
/**
 * @brief 파싱 완료된 패킷 가져오기 (Consumer Task에서 호출)
 */
static bool get_value(MarvelDex_packet_t* output)
{
    if (output == NULL) return false;
    // 큐에서 파싱 완료된 패킷을 가져옴 (Non-blocking)
    if (osMessageQueueGet(_queue, output, NULL, 0) != osOK) return false;
    return true;
}

/**
 * @brief IOIF UARTx_t ID를 내부 인덱스(0 또는 1)로 변환
 */
static int32_t get_sensor_index(IOIF_UARTx_t id)
{
    if (id == _instance[0]) return 0;
    if (id == _instance[1]) return 1;
    return -1; // 등록되지 않은 ID
}

/**
 * @brief IOIF UART 태스크에 의해 호출되는 콜백 (패킷 프레이밍)
 */
static void callback(uint8_t* rx_buf, uint32_t size, uint32_t id)
{
    int32_t idx = get_sensor_index(id);
    if (idx == -1) return; // 알 수 없는 UART 인스턴스

    // 이 인스턴스(센서)의 상태 변수들을 가져옴
    ParseState_t* state = &_parseState[idx];
    uint8_t* buffer = _rx_buffer[idx].raw;
    uint8_t* count = &_rx_index[idx];

    for (uint32_t i = 0; i < size; i++) {
        uint8_t byte = rx_buf[i];
        
        switch (*state) {
            case STATE_WAIT_HEADER_0:
                // 1. 헤더 0 (0xFF) 대기
                if (byte == FSR_HEADER_0) {
                    buffer[0] = byte;
                    *count = 1;
                    *state = STATE_WAIT_HEADER_1;
                }
                break;

            case STATE_WAIT_HEADER_1:
                // 2. 헤더 1 (0xFF) 대기
                if (byte == FSR_HEADER_1) {
                    buffer[1] = byte;
                    *count = 2;
                    *state = STATE_COLLECT_PAYLOAD;
                } else {
                    // 0xFF 다음 0xFF가 아니면 리셋
                    *state = STATE_WAIT_HEADER_0;
                    *count = 0;
                }
                break;

            case STATE_COLLECT_PAYLOAD:
                // 3. 나머지 26바이트 수집
                buffer[*count] = byte;
                (*count)++;

                if (*count >= MARVELDEX_RAW_PACKET_SIZE) {
                    // 4. 28바이트가 모두 모이면 파싱 시도
                    MarvelDex_packet_t parsed_packet;
                    if (parsing(&_rx_buffer[idx], &parsed_packet)) {
                        
                        // 5. 파싱 성공 시 큐에 삽입
                        osMessageQueuePut(_queue, &parsed_packet, 0, 0);
                    }
                    // 6. 상태 리셋 (파싱 성공/실패 무관)
                    // 파싱 성공/실패 여부와 관계없이
                    // 상태, 카운트, 그리고 버퍼를 모두 리셋해야 합니다.
                    *state = STATE_WAIT_HEADER_0;
                    *count = 0;

                    // (선택 사항) 버퍼 전체를 0으로 클리어
                    memset(buffer, 0, MARVELDEX_RAW_PACKET_SIZE);
                    
                    // (필수) 'Stale Footer' 버그를 막기 위한 가장 효율적인 방법
                    // 버퍼 전체를 0으로 닦는 대신,
                    // 파싱에 사용되는 푸터 바이트(27번 인덱스)만
                    // 유효하지 않은 값(예: 0x00)으로 덮어씁니다.
                    // _rx_buffer[idx].raw[MARVELDEX_RAW_PACKET_SIZE - 1] = 0x00;
                }
                break;
        }
    }
}

/**
 * @brief 28바이트 원시 패킷을 구조체로 파싱 (FSR 예제 코드 반영)
 */
static bool parsing(MarvelDex_raw_packet_t* raw_packet, MarvelDex_packet_t* output)
{
    // 1. 푸터 검증
    if (raw_packet->raw[27] != FSR_FOOTER) {
        return false; // 잘못된 푸터 싱크워드
    }
    
    // (헤더는 callback 함수에서 이미 검증되었음)

    // 2. 타임스탬프
    // FSR 예제 코드의 주석 권장 사항대로, 수신 시점의 XM10 마스터의 시스템 틱으로 덮어씁니다.
    output->timestamp = IOIF_GetTick();
    
    // (참고) FSR 패킷 내부의 타임스탬프 (Big-endian) __REV()와 동일한 동작
    // XM10 Master의 시간이 아닌 센서의 시간이 필요하다면 아래와 같이 파싱하면 됨.
    // uint32_t packet_timestamp = ((uint32_t)raw_packet->raw[4] << 24) |
    //                             ((uint32_t)raw_packet->raw[5] << 16) |
    //                             ((uint32_t)raw_packet->raw[6] << 8)  |
    //                             ((uint32_t)raw_packet->raw[7]);

    // 3. 센서 공간 및 롤링 인덱스 파싱
    output->sensorSpace = raw_packet->raw[8];
    output->rollingIndex = raw_packet->raw[9];

    // 4. 14개 센서 채널 파싱 (memcpy로 최적화)
    memcpy(output->sensorData, &raw_packet->raw[10], MARVELDEX_CHANNEL_SIZE);

    // 5. 배터리 레벨 및 상태 플래그 파싱
    output->batteryLevel = raw_packet->raw[24];
    output->statusFlags = raw_packet->raw[25];

    return true; // 파싱 성공
}

// static inline void _move_buffer(uint8_t* dest, uint8_t* src, size_t len)
// {
//     for(uint32_t i = 0; i < len ; i ++ ) dest[i] = src[i];
// }

// static uint32_t _debug_fsr_stage = 0;
// static uint8_t _debug_packet_buffer[MARVELDEX_PACKET_BUFFER_MAX];
// static uint32_t _debug_packet_counter;
// static uint32_t _debug_send_counter = 0;
// static void callback(uint8_t* buffer, uint32_t len, IOIF_UARTx_t id)
// {    
//     static uint8_t _packet[MAX_SENSOR_NUM][MARVELDEX_PACKET_BUFFER_MAX];
//     static uint32_t _packet_count[MAX_SENSOR_NUM];
//     static ParseStage_e _stage[MAX_SENSOR_NUM] = {PACKET_IDLE};
//     static ParseStage_raw_e _stage_raw[MAX_SENSOR_NUM] = {PACKET_RAW_IDLE, };

//     if ((buffer == NULL) || (len == 0) || (id > IOIF_UART_MAX_INSTANCES)) return;

//     uint32_t index = sensor_map[id];
//     if ( index == -1 ) return;

//     uint8_t* packet = (uint8_t*)_packet[index];
//     uint32_t* packet_count = (uint32_t*)&_packet_count[index];
//     ParseStage_e* stage = &_stage[index];
//     ParseStage_raw_e* raw_stage = &_stage_raw[index];

//     //패킷의 헤드를 찾아야 함
//     MarvelDex_packet_t output;
//     MarvelDex_raw_packet_t* output_buff = (MarvelDex_raw_packet_t*)packet;
     
//     for ( uint32_t i = 0 ; i < len ; i++ )
//     {
//         uint8_t byte = buffer[i];

//         switch( raw_stage[0] )
//         {
//             case PACKET_RAW_IDLE:
//             {
//                 if ( byte == MARVELDEX_PACKET_HEAD_0 ) raw_stage[0]++;
//             } break;
//             case PACKET_RAW_START_0:
//             {
//                 if( byte == MARVELDEX_PACKET_HEAD_1 ) raw_stage[0]++;
//                 else raw_stage[0] = PACKET_RAW_IDLE;
//             } break;
//             case PACKET_RAW_START_1:
//             {
//                 if( byte == MARVELDEX_PACKET_HEAD_2 ) raw_stage[0]++;
//                 else raw_stage[0] = PACKET_RAW_IDLE;
//             } break;
//             case PACKET_RAW_START_2:
//             {
//                 if( byte == MARVELDEX_PACKET_HEAD_3 ) {
//                     raw_stage[0]++;
//                     output_buff->header = MARVELDEX_PACKET_HEADER;
//                     packet_count[0] = MARVELDEX_PACKET_HEADER_SIZE;
//                 }
//                 else raw_stage[0] = PACKET_RAW_IDLE;
//             } break;
//             case PACKET_RAW_DATA_PUSH:
//             {
//                 packet[packet_count[0]++] = byte;

//                 if( packet_count[0] < sizeof(MarvelDex_raw_packet_t)) break;

//                 if(parsing(output_buff, &output)){ 
//                     if(osMessageQueuePut(_queue, &output, 0, 0) != osOK)
//                     {
//                         //EXCEPTION
//                     } else {
//                         _debug_send_counter++;
//                     }
//                 }

//                 //memset(packet, 0, MARVELDEX_PACKET_BUFFER_MAX );
//                 packet_count[0] = 0;
//                 raw_stage[0] = PACKET_RAW_IDLE;
//             } break;
//             default:
//             break;
//         }
//     }
//     #if 0
//     for ( uint32_t i = 0 ; i < len ; i ++ )
//     {
//         uint8_t byte = buffer[i];
//         packet[packet_count[0]++] = byte;

//         switch( *stage )
//         {
//             case PACKET_IDLE:
//             {
//             	_debug_fsr_stage=1;
//                 packet[packet_count[0]++] = byte;
//                 if ( packet_count[0] < (MARVELDEX_PACKET_HEADER_SIZE) ) continue;

//                 if ( output_buff->header == MARVELDEX_PACKET_HEADER )
//                 {
//                 	_debug_fsr_stage=10;
//                     *stage = PACKET_START;
//                 } else {
//                     packet_count[0]--;
//                     _move_buffer(packet, &packet[1], packet_count[0]);
//                     //memcpy(packet, &packet[1], --*packet_count);
//                 }
//             } break;
//             case PACKET_START:
//             {
//             	_debug_fsr_stage=2;
//                 packet[packet_count[0]++] = byte;

//                 if ( packet_count[0] >= sizeof(MarvelDex_raw_packet_t))
//                 {
//                     if(parsing(output_buff, &output)){
//                         *stage = PACKET_END;
//                         if(osMessageQueuePut(_queue, &output, 0, 0) != osOK)
//                         {
//                             //EXCEPTION
//                         } else {
//                             _debug_send_counter++;
//                         }
//                         memset(packet, 0, MARVELDEX_PACKET_BUFFER_MAX );
//                         packet_count[0] = 0;
//                         *stage = PACKET_IDLE;
//                     }
//                     else{
//                         *stage = PACKET_ERROR;
//                         packet_count[0]=0;
//                         memset(packet, 0, MARVELDEX_PACKET_BUFFER_MAX );
//                         //_move_buffer(packet, &packet[1], packet_count[0]);
//                         //memcpy(packet, &packet[1], --*packet_count);
//                         *stage = PACKET_IDLE;
//                     }
//                 } else {
//                     *stage = PACKET_IDLE;  
//                     packet_count[0]=0;
//                     memset(packet, 0, MARVELDEX_PACKET_BUFFER_MAX );
//                 }
//             } break;
//             case PACKET_END: _debug_fsr_stage=3;
//             case PACKET_ERROR: _debug_fsr_stage=4;
//             default:
//             	_debug_fsr_stage = 5;
//             	memset(packet, 0, MARVELDEX_PACKET_BUFFER_MAX );
//                 packet_count[0] = 0;
//                 break;
//         }
//     }
//     #endif
//     memcpy(_debug_packet_buffer, packet, MARVELDEX_PACKET_BUFFER_MAX);
//     _debug_packet_counter = packet_count[0];
// }

// static bool parsing(MarvelDex_raw_packet_t* raw_packet, MarvelDex_packet_t* packet_out)
// {
//     if(raw_packet == NULL) return false;
//     if(packet_out == NULL) return false;
//     //     헤더(4)     : 0xff 0xff 0x00 0x01
//     if (raw_packet->header != MARVELDEX_PACKET_HEADER) return false;
//     //     푸터(2)     : 0x00 0xfe
//     // Footer 체크 비활성화 - 센서가 Footer를 보내지 않음
//     if (raw_packet->footer != MARVELDEX_PACKET_FOOTER) return false;

//     memset(packet_out, 0, sizeof(MarvelDex_packet_t));
//     uint8_t lr = raw_packet->sensor_LR;
// //     센서공간(1)  : 1=왼발, 2=오른발
//     packet_out->sensor_LR = (lr == 1)?MARVELDEX_SENSOR_SPACE_LEFT:(lr == 2)?MARVELDEX_SENSOR_SPACE_RIGHT:MARVELDEX_SENSOR_SPACE_UNKNOWN;
// //     타임스탬프(4): millis() 값 (빅엔디안)
//     packet_out->timestamp = __REV(raw_packet->timestamp);
// //     롤링인덱스(1): 0~199 순환
//     packet_out->rolling_index = raw_packet->rolling_index;

// //     센서데이터(14): 14채널 매핑된 값 (0~0xEF)
//     for (uint8_t i=0; i < MARVELDEX_PACKET_DATA_NUM; i++){
//         packet_out->sensor_data[i] = raw_packet->sensor_data[i];
//     }

//     return true;    
// }
