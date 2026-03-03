/**
 ******************************************************************************
 * @file    agr_dop.c
 * @author  HyundoKim
 * @brief   AGR-DOP V2 - CANopen SDO & PDO Protocol Implementation
 * @version 2.0 (CANopen CiA 301 기반)
 * @date    Dec 8, 2025
 * 
 * @details
 * CANopen 표준 (CiA 301)을 준수하는 SDO, PDO 프로토콜 구현입니다.
 * 
 * [지원 프로토콜]
 * - SDO (Service Data Object): Object Dictionary 읽기/쓰기
 *   - CAN ID 0x600 + Node ID: SDO Download (Write Request)
 *   - CAN ID 0x580 + Node ID: SDO Upload (Read Response)
 * - PDO (Process Data Object): 실시간 데이터 전송
 *   - CAN ID 0x180/0x280 + Node ID: TPDO1/TPDO2
 * 
 * [Object Dictionary]
 * - 0x0000 ~ 0x0FFF: Data Type Definitions
 * - 0x1000 ~ 0x1FFF: Communication Profile
 * - 0x2000 ~ 0x5FFF: Manufacturer Specific (제조사 영역) ✅
 * - 0x6000 ~ 0x9FFF: Device Profile (CiA 401, 402 등)
 * 
 * [WriteCb (Write Callback)]
 * - SDO Download 완료 후 자동 호출 (CANopen 표준)
 * - Object Dictionary Entry에 정의
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_dop.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/** @brief SDO 헤더 크기 (cmd + index + subindex) */
#define SDO_HEADER_SIZE     4

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

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

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static int _ProcessSDOReadRequest(AGR_DOP_Ctx_t* ctx,
                                   const AGR_SDO_Msg_t* req,
                                   AGR_SDO_Msg_t* out_rsp);

static int _ProcessSDOWriteRequest(AGR_DOP_Ctx_t* ctx,
                                    const AGR_SDO_Msg_t* req,
                                    AGR_SDO_Msg_t* out_rsp);

static void _CreateAbortResponse(AGR_SDO_Msg_t* out_rsp,
                                  uint16_t index,
                                  uint8_t subindex,
                                  AGR_SDO_AbortCode_t abort_code);

static void _ProcessSyncMessage(AGR_DOP_Ctx_t* ctx, uint32_t current_ms);

static void _ProcessEmergencyMessage(AGR_DOP_Ctx_t* ctx,
                                      const uint8_t* data,
                                      uint8_t len);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/*============================================================
 * INITIALIZATION
 *============================================================*/

int AGR_DOP_Init(AGR_DOP_Ctx_t* ctx,
                 const AGR_OD_Table_t* od,
                 uint8_t node_id,
                 AGR_TxFunc_t tx_func)
{
    if (ctx == NULL || od == NULL || tx_func == NULL) {
        return -1;
    }
    
    memset(ctx, 0, sizeof(AGR_DOP_Ctx_t));
    
    ctx->od.entries = od->entries;
    ctx->od.entry_count = od->entry_count;
    ctx->node_id = node_id;
    ctx->tx_func = tx_func;
    
    /* PDO Mapping 초기화 (모든 TPDO/RPDO) */
    for (uint8_t i = 0; i < 4; i++) {
        ctx->tx_pdo_map[i].count = 0;
        ctx->rx_pdo_map[i].count = 0;
    }
    
    return 0;
}

void AGR_DOP_Reset(AGR_DOP_Ctx_t* ctx)
{
    if (ctx == NULL) {
        return;
    }
    
    /* PDO Mapping 초기화 (모든 TPDO/RPDO) */
    for (uint8_t i = 0; i < 4; i++) {
        ctx->tx_pdo_map[i].count = 0;
        ctx->rx_pdo_map[i].count = 0;
    }
}

/*============================================================
 * OBJECT DICTIONARY
 *============================================================*/

const AGR_OD_Entry_t* AGR_DOP_FindODEntry(const AGR_DOP_Ctx_t* ctx, uint16_t index)
{
    if (ctx == NULL || ctx->od.entries == NULL) {
        return NULL;
    }
    
    for (uint16_t i = 0; i < ctx->od.entry_count; i++) {
        if (ctx->od.entries[i].index == index) {
            return &ctx->od.entries[i];
        }
    }
    
    return NULL;
}

const AGR_OD_Entry_t* AGR_DOP_FindODEntryEx(const AGR_DOP_Ctx_t* ctx, 
                                             uint16_t index, 
                                             uint8_t subindex)
{
    if (ctx == NULL || ctx->od.entries == NULL) {
        return NULL;
    }
    
    for (uint16_t i = 0; i < ctx->od.entry_count; i++) {
        if (ctx->od.entries[i].index == index && 
            ctx->od.entries[i].subindex == subindex) {
            return &ctx->od.entries[i];
        }
    }
    
    return NULL;
}

int AGR_DOP_ReadODValue(const AGR_OD_Entry_t* entry, void* out_buf, uint8_t buf_len)
{
    if (entry == NULL || out_buf == NULL) {
        return -1;
    }
    
    if (entry->data_ptr == NULL) {
        return -2;
    }
    
    /* 접근 권한 확인 */
    if (entry->access == AGR_ACCESS_WO) {
        return -3;  /* Write Only */
    }
    
    uint8_t copy_len = (buf_len < entry->size) ? buf_len : entry->size;
    memcpy(out_buf, entry->data_ptr, copy_len);
    
    return copy_len;
}

int AGR_DOP_WriteODValue(const AGR_OD_Entry_t* entry, const void* in_buf, uint8_t in_len)
{
    if (entry == NULL || in_buf == NULL) {
        return -1;
    }
    
    if (entry->data_ptr == NULL) {
        return -2;
    }
    
    /* 접근 권한 확인 */
    if (entry->access == AGR_ACCESS_RO) {
        return -3;  /* Read Only */
    }
    
    /* 크기 확인 */
    if (in_len > entry->size) {
        return -4;
    }
    
    memcpy(entry->data_ptr, in_buf, in_len);
    
    /* 쓰기 완료 콜백 */
    if (entry->on_write != NULL) {
        entry->on_write();
    }
    
    return 0;
}

/*============================================================
 * SDO
 *============================================================*/

void AGR_DOP_CreateSDOReadReq(AGR_SDO_Msg_t* out_msg, uint16_t index, uint8_t subindex)
{
    if (out_msg == NULL) {
        return;
    }
    
    memset(out_msg, 0, sizeof(AGR_SDO_Msg_t));
    out_msg->cs = AGR_SDO_CS_UPLOAD_INIT_REQ;  /* CANopen: Upload = Read */
    out_msg->index = index;
    out_msg->subindex = subindex;
    out_msg->data_len = 0;
}

void AGR_DOP_CreateSDOWriteReq(AGR_SDO_Msg_t* out_msg, 
                                uint16_t index, 
                                uint8_t subindex,
                                const void* data, 
                                uint8_t data_len)
{
    if (out_msg == NULL) {
        return;
    }
    
    memset(out_msg, 0, sizeof(AGR_SDO_Msg_t));
    out_msg->cs = AGR_SDO_CS_DOWNLOAD_INIT_REQ;  /* CANopen: Download = Write */
    out_msg->index = index;
    out_msg->subindex = subindex;
    
    if (data != NULL && data_len > 0) {
        uint8_t copy_len = (data_len > AGR_SDO_MAX_DATA_SIZE) ? AGR_SDO_MAX_DATA_SIZE : data_len;
        memcpy(out_msg->data, data, copy_len);
        out_msg->data_len = copy_len;
    }
}

int AGR_DOP_ProcessSDORequest(AGR_DOP_Ctx_t* ctx,
                               const AGR_SDO_Msg_t* req,
                               AGR_SDO_Msg_t* out_rsp)
{
    if (ctx == NULL || req == NULL || out_rsp == NULL) {
        return -1;
    }
    
    /* 커스텀 콜백이 있으면 먼저 호출 */
    if (ctx->on_sdo_request != NULL) {
        ctx->on_sdo_request(req, out_rsp);
        /* 콜백에서 처리 완료했으면 리턴 */
        if (out_rsp->cs != 0) {
            return 0;
        }
    }
    
    /* CANopen Command Specifier 처리 */
    switch (req->cs & 0xE0) {  /* 상위 3비트로 명령 구분 */
        case 0x40:  /* Upload Initiate (Read) */
            return _ProcessSDOReadRequest(ctx, req, out_rsp);
            
        case 0x20:  /* Download Initiate (Write) */
            return _ProcessSDOWriteRequest(ctx, req, out_rsp);
            
        case 0x80:  /* Abort */
            /* Abort는 별도 처리 불필요 (요청자가 중단) */
            return 0;
            
        default:
            _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                                AGR_SDO_ABORT_INVALID_CS);
            return -2;
    }
}

int AGR_DOP_EncodeSDO(const AGR_SDO_Msg_t* msg, uint8_t* out_buf)
{
    if (msg == NULL || out_buf == NULL) {
        return -1;
    }
    
    /* 헤더 인코딩 (CANopen 표준) */
    out_buf[0] = msg->cs;  /* Command Specifier */
    out_buf[1] = (uint8_t)(msg->index & 0xFF);         /* Index Low */
    out_buf[2] = (uint8_t)((msg->index >> 8) & 0xFF);  /* Index High */
    out_buf[3] = msg->subindex;
    
    /* 데이터 복사 */
    if (msg->data_len > 0) {
        memcpy(&out_buf[4], msg->data, msg->data_len);
    }
    
    /* 
     * [CANopen 표준] Expedited Transfer는 DLC = 8 (고정)
     * - Upload Response (cs & 0xE0 == 0x40): 항상 8바이트
     * - Download Response (cs & 0xE0 == 0x60): 항상 8바이트
     * - Abort (cs == 0x80): 항상 8바이트
     */
    if ((msg->cs & 0xE0) == 0x40 || (msg->cs & 0xE0) == 0x60 || msg->cs == 0x80) {
        /* Expedited Transfer 또는 Abort: 8바이트 고정 (패딩 포함) */
        return 8;
    }
    
    return SDO_HEADER_SIZE + msg->data_len;
}

int AGR_DOP_DecodeSDO(const uint8_t* in_buf, uint8_t in_len, AGR_SDO_Msg_t* out_msg)
{
    if (in_buf == NULL || out_msg == NULL) {
        return -1;
    }
    
    if (in_len < SDO_HEADER_SIZE) {
        return -2;  /* 헤더 부족 */
    }
    
    memset(out_msg, 0, sizeof(AGR_SDO_Msg_t));
    
    /* 헤더 디코딩 (CANopen 표준) */
    out_msg->cs = in_buf[0];  /* Command Specifier */
    out_msg->index = (uint16_t)in_buf[1] | ((uint16_t)in_buf[2] << 8);
    out_msg->subindex = in_buf[3];
    
    /* 
     * [CANopen Expedited Transfer 처리]
     * - Upload Response (cs & 0xE0 == 0x40): cs의 n 비트에서 data_len 추출
     *   - n = (cs >> 2) & 0x03  (bits 2-3)
     *   - data_len = 4 - n
     * - 다른 메시지: in_len - SDO_HEADER_SIZE
     */
    if ((out_msg->cs & 0xE0) == 0x40) {
        /* Upload Response (Expedited): n 비트에서 데이터 길이 추출 */
        uint8_t n = (out_msg->cs >> 2) & 0x03;  /* bits 2-3 */
        out_msg->data_len = 4 - n;  /* 실제 데이터 길이 */
        
        if (out_msg->data_len > 4) {
            out_msg->data_len = 4;  /* 안전 범위 */
        }
    } else {
        /* 다른 SDO 메시지: 총 길이에서 헤더 제외 */
        out_msg->data_len = in_len - SDO_HEADER_SIZE;
        if (out_msg->data_len > AGR_SDO_MAX_DATA_SIZE) {
            out_msg->data_len = AGR_SDO_MAX_DATA_SIZE;
        }
    }
    
    if (out_msg->data_len > 0) {
        memcpy(out_msg->data, &in_buf[4], out_msg->data_len);
    }
    
    return 0;
}

int AGR_DOP_SendSDO(AGR_DOP_Ctx_t* ctx, uint8_t target_id, const AGR_SDO_Msg_t* msg)
{
    if (ctx == NULL || ctx->tx_func == NULL || msg == NULL) {
        return -1;
    }
    
    uint8_t buf[AGR_CANFD_MAX_PAYLOAD];
    int len = AGR_DOP_EncodeSDO(msg, buf);
    
    if (len < 0) {
        return len;
    }
    
    /* CAN-ID 결정: 요청이면 SDO_RX, 응답이면 SDO_TX */
    uint32_t can_id;
    if (msg->cs == AGR_SDO_CS_UPLOAD_INIT_REQ || msg->cs == AGR_SDO_CS_DOWNLOAD_INIT_REQ) {
        can_id = AGR_DOP_GetSDORequestCANID(target_id);
    } else {
        can_id = AGR_DOP_GetSDOResponseCANID(ctx->node_id);
    }
    
    return ctx->tx_func(can_id, buf, (uint8_t)len);
}

/*============================================================
 * PDO MAPPING
 *============================================================*/

void AGR_DOP_ClearTxPDOMap(AGR_DOP_Ctx_t* ctx)
{
    if (ctx == NULL) {
        return;
    }
    /* TPDO1 클리어 (하위 호환성) */
    ctx->tx_pdo_map[0].count = 0;
}

void AGR_DOP_ClearTxPDOMapN(AGR_DOP_Ctx_t* ctx, uint8_t pdo_num)
{
    if (ctx == NULL || pdo_num < 1 || pdo_num > 4) {
        return;
    }
    ctx->tx_pdo_map[pdo_num - 1].count = 0;
}

int AGR_DOP_AddTxPDOMap(AGR_DOP_Ctx_t* ctx, uint16_t index, uint8_t subindex)
{
    /* TPDO1 추가 (하위 호환성) */
    return AGR_DOP_AddTxPDOMapN(ctx, 1, index, subindex);
}

int AGR_DOP_AddTxPDOMapN(AGR_DOP_Ctx_t* ctx, uint8_t pdo_num, uint16_t index, uint8_t subindex)
{
    if (ctx == NULL || pdo_num < 1 || pdo_num > 4) {
        return -1;
    }
    
    AGR_PDO_MapTable_t* map = &ctx->tx_pdo_map[pdo_num - 1];
    
    /* 테이블 공간 확인 */
    if (map->count >= AGR_PDO_MAP_MAX_ENTRIES) {
        return -1;  /* 테이블 가득 참 */
    }
    
    /* OD Entry 존재 확인 */
    const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, index, subindex);
    if (entry == NULL) {
        return -2;  /* OD Entry 없음 */
    }
    
    /* 중복 확인 */
    for (uint8_t i = 0; i < map->count; i++) {
        if (map->items[i].od_index == index &&
            map->items[i].od_subindex == subindex) {
            return 0;  /* 이미 존재 */
        }
    }
    
    /* 추가 */
    map->items[map->count].od_index = index;
    map->items[map->count].od_subindex = subindex;
    map->count++;
    
    return 0;
}

int AGR_DOP_ApplyTxPDOMapFromSDO(AGR_DOP_Ctx_t* ctx, 
                                  const uint8_t* data, 
                                  uint8_t data_len)
{
    /* TPDO1 적용 (하위 호환성) */
    return AGR_DOP_ApplyTxPDOMapFromSDON(ctx, 1, data, data_len);
}

int AGR_DOP_ApplyTxPDOMapFromSDON(AGR_DOP_Ctx_t* ctx, 
                                   uint8_t pdo_num,
                                   const uint8_t* data, 
                                   uint8_t data_len)
{
    if (ctx == NULL || data == NULL || pdo_num < 1 || pdo_num > 4) {
        return -1;
    }
    
    /* 데이터 길이 확인 */
    if (data_len < 1) {
        return -2;  /* 최소 1 byte (Number of objects) 필요 */
    }
    
    /* 기존 매핑 초기화 */
    AGR_DOP_ClearTxPDOMapN(ctx, pdo_num);
    
    /* 
     * [CANopen 표준 PDO Mapping Format]
     * Byte 0: Number of mapped objects (1B)
     * Byte 1-4: 1st object (4B)
     *   - Byte 0: Object Length (bits) - 예: 32 (4B × 8), 160 (20B × 8)
     *   - Byte 1: Object SubIndex
     *   - Byte 2-3: Object Index (Little Endian)
     * Byte 5-8: 2nd object (4B)
     * ...
     */
    
    uint8_t num_objects = data[0];  /* 첫 바이트: 항목 개수 */
    
    int added = 0;
    for (uint8_t i = 1; i + 3 < data_len; i += 4) {  /* ✅ data[1]부터 시작 */
        uint8_t  obj_len_bits = data[i];      /* Byte 0: Length (bits, unused - OD에서 자동 추출) */
        uint8_t  subindex = data[i + 1];      /* Byte 1: SubIndex */
        uint16_t index = (uint16_t)data[i + 2] | ((uint16_t)data[i + 3] << 8);  /* Byte 2-3: Index (Little Endian) */
        
        (void)obj_len_bits;  /* OD Entry에서 size를 가져오므로 무시 */
        
        if (AGR_DOP_AddTxPDOMapN(ctx, pdo_num, index, subindex) == 0) {
            added++;
        }
        
        /* 선언된 개수만큼만 추가 */
        if (added >= num_objects) {
            break;
        }
    }
    
    return added;
}

void AGR_DOP_ClearRxPDOMap(AGR_DOP_Ctx_t* ctx)
{
    if (ctx == NULL) {
        return;
    }
    /* RPDO1 클리어 (하위 호환성) */
    ctx->rx_pdo_map[0].count = 0;
}

int AGR_DOP_AddRxPDOMap(AGR_DOP_Ctx_t* ctx, uint16_t index, uint8_t subindex)
{
    if (ctx == NULL) {
        return -1;
    }
    
    AGR_PDO_MapTable_t* map = &ctx->rx_pdo_map[0];  /* RPDO1 (하위 호환성) */
    
    /* 테이블 공간 확인 */
    if (map->count >= AGR_PDO_MAP_MAX_ENTRIES) {
        return -1;
    }
    
    /* OD Entry 존재 확인 */
    const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, index, subindex);
    if (entry == NULL) {
        return -2;
    }
    
    /* 중복 확인 */
    for (uint8_t i = 0; i < map->count; i++) {
        if (map->items[i].od_index == index &&
            map->items[i].od_subindex == subindex) {
            return 0;
        }
    }
    
    /* 추가 */
    map->items[map->count].od_index = index;
    map->items[map->count].od_subindex = subindex;
    map->count++;
    
    return 0;
}

/*============================================================
 * PDO ENCODE/DECODE
 *============================================================*/

int AGR_DOP_EncodeTxPDO(const AGR_DOP_Ctx_t* ctx, uint8_t* out_buf, uint8_t buf_size)
{
    /* TPDO1 인코딩 (하위 호환성) */
    return AGR_DOP_EncodeTxPDON(ctx, 1, out_buf, buf_size);
}

int AGR_DOP_EncodeTxPDON(const AGR_DOP_Ctx_t* ctx, uint8_t pdo_num, uint8_t* out_buf, uint8_t buf_size)
{
    if (ctx == NULL || out_buf == NULL || pdo_num < 1 || pdo_num > 4) {
        return -1;
    }
    
    const AGR_PDO_MapTable_t* map = &ctx->tx_pdo_map[pdo_num - 1];
    uint8_t offset = 0;
    
    for (uint8_t i = 0; i < map->count; i++) {
        const AGR_PDO_MapItem_t* item = &map->items[i];
        const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, 
                                                            item->od_index, 
                                                            item->od_subindex);
        
        if (entry == NULL || entry->data_ptr == NULL) {
            continue;
        }
        
        /* 버퍼 오버플로우 방지 */
        if (offset + entry->size > buf_size) {
            break;
        }
        
        /* 데이터 복사 */
        memcpy(&out_buf[offset], entry->data_ptr, entry->size);
        offset += entry->size;
    }
    
    return offset;
}

int AGR_DOP_DecodeRxPDO(AGR_DOP_Ctx_t* ctx, const uint8_t* in_buf, uint8_t in_len)
{
    if (ctx == NULL || in_buf == NULL) {
        return -1;
    }
    
    const AGR_PDO_MapTable_t* map = &ctx->rx_pdo_map[0];  /* RPDO1 (하위 호환성) */
    uint8_t offset = 0;
    
    for (uint8_t i = 0; i < map->count; i++) {
        const AGR_PDO_MapItem_t* item = &map->items[i];
        const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, 
                                                            item->od_index, 
                                                            item->od_subindex);
        
        if (entry == NULL || entry->data_ptr == NULL) {
            continue;
        }
        
        /* 버퍼 언더플로우 방지 */
        if (offset + entry->size > in_len) {
            break;
        }
        
        /* 데이터 복사 (쓰기 가능한 경우만) */
        if (entry->access != AGR_ACCESS_RO) {
            memcpy(entry->data_ptr, &in_buf[offset], entry->size);
            
            /* 쓰기 완료 콜백 */
            if (entry->on_write != NULL) {
                entry->on_write();
            }
        }
        
        offset += entry->size;
    }
    
    return offset;
}

/*============================================================
 * MESSAGE PROCESSING
 *============================================================*/

/**
 * @brief CAN 메시지 수신 처리 (CANopen 표준 Function Code 기반)
 * @param ctx    DOP Context
 * @param can_id CAN-ID (11-bit 또는 29-bit)
 * @param data   수신 데이터
 * @param len    데이터 길이
 * @return 0=처리 완료, 1=처리 안 함, <0=에러
 * 
 * @details
 * CANopen 표준에 따라 CAN-ID를 Function Code와 Node ID로 분리하여 처리합니다.
 * 
 * [CAN-ID 구조]
 * - 11-bit ID: [Func Code 4-bit][Node ID 7-bit]
 * - func_code = (can_id & 0x780) >> 7
 * - source_node = can_id & 0x7F
 * 
 * [지원하는 Function Code]
 * - NMT (0x00): Network Management (현재 미지원, 상위 계층에서 처리)
 * - SYNC (0x01): Synchronization Object
 * - EMCY (0x01): Emergency Messages
 * - TPDO1~4 (0x03,0x05,0x07,0x09): Master → Node (현재 미지원)
 * - RPDO1~4 (0x04,0x06,0x08,0x0A): Node → Master (수신 처리)
 * - SDO RX (0x0C): SDO Request (Master → Node)
 * - SDO TX (0x0B): SDO Response (현재 미지원, Slave는 요청만 받음)
 * - Heartbeat (0x0E): 생존 신호 (상위 계층에서 처리)
 */
int AGR_DOP_ProcessRxMessage(AGR_DOP_Ctx_t* ctx, 
                              uint32_t can_id, 
                              const uint8_t* data, 
                              uint8_t len)
{
    if (ctx == NULL || data == NULL) {
        return -1;
    }
    
    /* CAN-ID에서 Function Code와 Node ID 추출 (CANopen 표준) */
    uint8_t func_code = (can_id & 0x780) >> 7;     /* Upper 4 bits */
    uint8_t source_node = can_id & 0x7F;           /* Lower 7 bits */
    
    /* Function Code 기반 메시지 처리 (CANopen 표준) */
    switch (func_code) {
        /* ===== NMT (Network Management) ===== */
        case AGR_CAN_FUNC_NMT:
            /* NMT는 보통 상위 계층(Link Layer)에서 처리 */
            return 1;  /* 처리 안 함 */
        
        /* ===== SYNC / EMCY (동일한 Function Code) ===== */
        case AGR_CAN_FUNC_SYNC_EMCY:
            /* SYNC: CAN-ID = 0x080 (Node ID 없음) */
            if (can_id == AGR_CAN_ID_SYNC) {
                uint32_t current_ms = 0;  /* 필요 시 타이머 함수 호출 */
                _ProcessSyncMessage(ctx, current_ms);
                return 0;
            }
            
            /* EMCY: CAN-ID = 0x080 + Node ID */
            if (source_node != 0) {
                _ProcessEmergencyMessage(ctx, data, len);
                return 0;
            }
            
            return 1;  /* Unknown */
        
        /* ===== TPDO (Transmit PDO from Master) ===== */
        case AGR_CAN_FUNC_TPDO1:
        case AGR_CAN_FUNC_TPDO2:
        case AGR_CAN_FUNC_TPDO3:
        case AGR_CAN_FUNC_TPDO4:
            /* IMU Hub는 Slave이므로 Master의 TPDO를 받지 않음 */
            /* 만약 받는다면 target_node_id 확인 후 처리 */
            if (source_node == ctx->target_node_id) {
                /* 필요 시 TPDO 처리 로직 추가 */
                return 0;
            }
            return 1;  /* 처리 안 함 */
        
        /* ===== RPDO (Receive PDO from Node) ===== */
        case AGR_CAN_FUNC_RPDO1:
        case AGR_CAN_FUNC_RPDO2:
        case AGR_CAN_FUNC_RPDO3:
        case AGR_CAN_FUNC_RPDO4:
            /* 내가 받아야 할 RPDO인지 확인 (CAN-ID == RPDOx + my_node_id) */
            {
                uint8_t pdo_num = 0;
                uint32_t expected_can_id = 0;
                
                /* Function Code에 따라 PDO 번호 결정 */
                if (func_code == AGR_CAN_FUNC_RPDO1) {
                    pdo_num = 1;
                    expected_can_id = AGR_DOP_GetRPDOCANID(ctx->node_id, pdo_num);
                } else if (func_code == AGR_CAN_FUNC_RPDO2) {
                    pdo_num = 2;
                    expected_can_id = AGR_DOP_GetRPDOCANID(ctx->node_id, pdo_num);
                } else if (func_code == AGR_CAN_FUNC_RPDO3) {
                    pdo_num = 3;
                    expected_can_id = AGR_DOP_GetRPDOCANID(ctx->node_id, pdo_num);
                } else if (func_code == AGR_CAN_FUNC_RPDO4) {
                    pdo_num = 4;
                    expected_can_id = AGR_DOP_GetRPDOCANID(ctx->node_id, pdo_num);
                }
                
                /* CAN-ID가 내 Node ID와 일치하는지 확인 */
                if (can_id == expected_can_id) {
                    /* RPDO 디코딩 */
                    AGR_DOP_DecodeRxPDO(ctx, data, len);
                    
                    /* PDO 수신 콜백 */
                    if (ctx->on_pdo_received != NULL) {
                        ctx->on_pdo_received(can_id, data, len);
                    }
                    
                    return 0;
                }
            }
            return 1;  /* 처리 안 함 */
        
        /* ===== SDO TX (Response from Node) ===== */
        case AGR_CAN_FUNC_SDO_TX:
            /* IMU Hub는 Slave이므로 다른 Node의 SDO Response를 받지 않음 */
            /* Master 모드에서는 여기서 SDO Response 처리 */
            return 1;  /* 처리 안 함 */
        
        /* ===== SDO RX (Request to Node) ===== */
        case AGR_CAN_FUNC_SDO_RX:
            /* 나에게 온 SDO Request인지 확인 (CAN-ID == 0x600 + my_node_id) */
            if (can_id == AGR_DOP_GetSDORequestCANID(ctx->node_id)) {
                /* ===== 표준 CANopen SDO 처리 ===== */
                AGR_SDO_Msg_t req, rsp;
                
                if (AGR_DOP_DecodeSDO(data, len, &req) != 0) {
                    return -2;  /* 디코딩 실패 */
                }
                
                if (AGR_DOP_ProcessSDORequest(ctx, &req, &rsp) != 0) {
                    return -3;  /* SDO 처리 실패 */
                }
                
                /* 응답 전송 */
                uint8_t rsp_buf[AGR_CANFD_MAX_PAYLOAD];
                int rsp_len = AGR_DOP_EncodeSDO(&rsp, rsp_buf);
                if (rsp_len > 0) {
                    ctx->tx_func(AGR_DOP_GetSDOResponseCANID(ctx->node_id), 
                                rsp_buf, (uint8_t)rsp_len);
                }
                
                return 0;
            }
            return 1;  /* 처리 안 함 */
        
        /* ===== Heartbeat ===== */
        case AGR_CAN_FUNC_HEARTBEAT:
            /* Heartbeat는 보통 Link Layer에서 처리 (xm_drv.c 등) */
            /* DOP에서는 처리하지 않고 상위 계층으로 전달 */
            return 1;  /* 처리 안 함 */
        
        /* ===== Unknown Function Code ===== */
        default:
            return 1;  /* 처리 안 함 */
    }
}

int AGR_DOP_SendTxPDO(AGR_DOP_Ctx_t* ctx, uint8_t pdo_num)
{
    if (ctx == NULL || ctx->tx_func == NULL) {
        return -1;
    }
    
    if (pdo_num < 1 || pdo_num > 4) {
        return -2;
    }
    
    uint8_t buf[AGR_CANFD_MAX_PAYLOAD];
    int len = AGR_DOP_EncodeTxPDO(ctx, buf, sizeof(buf));
    
    if (len <= 0) {
        return -3;
    }
    
    uint32_t can_id = AGR_DOP_GetTPDOCANID(ctx->node_id, pdo_num);
    
    return ctx->tx_func(can_id, buf, (uint8_t)len);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief SDO Read 요청 처리 (CANopen Upload Initiate)
 */
static int _ProcessSDOReadRequest(AGR_DOP_Ctx_t* ctx,
                                   const AGR_SDO_Msg_t* req,
                                   AGR_SDO_Msg_t* out_rsp)
{
    /* OD Entry 검색 */
    const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, req->index, req->subindex);
    
    if (entry == NULL) {
        _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                            AGR_SDO_ABORT_NOT_EXIST);
        return -1;
    }
    
    /* 읽기 권한 확인 */
    if (entry->access == AGR_ACCESS_WO) {
        _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                            AGR_SDO_ABORT_WRITE_ONLY);
        return -2;
    }
    
    /* 응답 생성 (Upload Initiate Response) */
    out_rsp->index = req->index;
    out_rsp->subindex = req->subindex;
    
    int read_len = AGR_DOP_ReadODValue(entry, out_rsp->data, AGR_SDO_MAX_DATA_SIZE);
    if (read_len < 0) {
        _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                            AGR_SDO_ABORT_GENERAL);
        return -3;
    }
    
    out_rsp->data_len = (uint8_t)read_len;
    
    /* CANopen Command Specifier: Expedited Upload with size */
    uint8_t n = 4 - read_len;  /* n = bytes NOT containing data */
    if (n > 3) n = 0;  /* CANFD 확장: 큰 데이터는 n=0 */
    out_rsp->cs = AGR_SDO_CS_UPLOAD_EXP(n);
    
    return 0;
}

/**
 * @brief SDO Write 요청 처리 (CANopen Download Initiate)
 */
static int _ProcessSDOWriteRequest(AGR_DOP_Ctx_t* ctx,
                                    const AGR_SDO_Msg_t* req,
                                    AGR_SDO_Msg_t* out_rsp)
{
    /* OD Entry 검색 */
    const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, req->index, req->subindex);
    
    if (entry == NULL) {
        _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                            AGR_SDO_ABORT_NOT_EXIST);
        return -1;
    }
    
    /* 쓰기 권한 확인 */
    if (entry->access == AGR_ACCESS_RO) {
        _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                            AGR_SDO_ABORT_READ_ONLY);
        return -2;
    }
    
    /* 데이터 쓰기 */
    int result = AGR_DOP_WriteODValue(entry, req->data, req->data_len);
    if (result < 0) {
        _CreateAbortResponse(out_rsp, req->index, req->subindex, 
                            AGR_SDO_ABORT_GENERAL);
        return -3;
    }
    
    /* 성공 응답 생성 (Download Initiate Response) */
    out_rsp->cs = AGR_SDO_CS_DOWNLOAD_INIT_RSP;
    out_rsp->index = req->index;
    out_rsp->subindex = req->subindex;
    out_rsp->data_len = 0;
    
    return 0;
}

/**
 * @brief SDO Abort 응답 생성 (CANopen 표준)
 */
static void _CreateAbortResponse(AGR_SDO_Msg_t* out_rsp,
                                  uint16_t index,
                                  uint8_t subindex,
                                  AGR_SDO_AbortCode_t abort_code)
{
    out_rsp->cs = AGR_SDO_CS_ABORT;
    out_rsp->index = index;
    out_rsp->subindex = subindex;
    
    /* Abort Code를 데이터에 저장 (4 bytes, Little Endian) */
    out_rsp->data[0] = (uint8_t)(abort_code & 0xFF);
    out_rsp->data[1] = (uint8_t)((abort_code >> 8) & 0xFF);
    out_rsp->data[2] = (uint8_t)((abort_code >> 16) & 0xFF);
    out_rsp->data[3] = (uint8_t)((abort_code >> 24) & 0xFF);
    out_rsp->data_len = 4;
}

/* ===== SDO Transmission (Tx) Functions ===== */

/**
 * @brief 통신 대상 Node ID 설정
 */
void AGR_DOP_SetTargetNodeId(AGR_DOP_Ctx_t* ctx, uint8_t target_node_id)
{
    if (ctx != NULL) {
        ctx->target_node_id = target_node_id;
    }
}

/**
 * @brief 단일 SDO Write Request 전송 (CANopen Download Initiate)
 */
int AGR_DOP_SendSDOWrite(AGR_DOP_Ctx_t* ctx, 
                         uint16_t index, 
                         uint8_t subindex,
                         const void* data, 
                         uint8_t data_len)
{
    if (ctx == NULL || ctx->tx_func == NULL || data == NULL) {
        return -1;
    }
    
    if (data_len > AGR_SDO_MAX_DATA_SIZE) {
        return -2;  /* 데이터 크기 초과 */
    }
    
    /* SDO Write Request 구성 (CANopen Download Initiate) */
    AGR_SDO_Msg_t sdo_req;
    sdo_req.cs = AGR_SDO_CS_DOWNLOAD_INIT_REQ;  /* Download Initiate Request */
    sdo_req.index = index;
    sdo_req.subindex = subindex;
    memcpy(sdo_req.data, data, data_len);
    sdo_req.data_len = data_len;
    
    /* Encode & Transmit */
    uint8_t buf[AGR_CANFD_MAX_PAYLOAD];
    int len = AGR_DOP_EncodeSDO(&sdo_req, buf);
    if (len <= 0) {
        return -3;
    }
    
    /* 상대방 Node의 SDO Request COB-ID로 전송 */
    uint32_t can_id = AGR_DOP_GetSDORequestCANID(ctx->target_node_id);
    return ctx->tx_func(can_id, (uint8_t*)buf, (uint8_t)len);
}

/*============================================================
 * PDO INHIBIT TIME (전송 간격 제한)
 *============================================================*/

/**
 * @brief PDO Inhibit Time 설정
 */
void AGR_DOP_SetPDOInhibitTime(AGR_DOP_Ctx_t* ctx, 
                                uint8_t pdo_num, 
                                uint32_t inhibit_time_us)
{
    if (ctx == NULL || pdo_num < 1 || pdo_num > 4) {
        return;
    }
    
    uint8_t idx = pdo_num - 1;  /* 0-based index */
    ctx->pdo_inhibit[idx].inhibit_enabled = true;
    ctx->pdo_inhibit[idx].inhibit_time_us = inhibit_time_us;
    ctx->pdo_inhibit[idx].last_tx_tick_us = 0;
}

/**
 * @brief PDO Inhibit Time 비활성화
 */
void AGR_DOP_DisablePDOInhibit(AGR_DOP_Ctx_t* ctx, uint8_t pdo_num)
{
    if (ctx == NULL || pdo_num < 1 || pdo_num > 4) {
        return;
    }
    
    uint8_t idx = pdo_num - 1;
    ctx->pdo_inhibit[idx].inhibit_enabled = false;
}

/**
 * @brief PDO 전송 가능 여부 확인
 */
bool AGR_DOP_CanSendPDO(AGR_DOP_Ctx_t* ctx, 
                         uint8_t pdo_num, 
                         uint32_t current_time_us)
{
    if (ctx == NULL || pdo_num < 1 || pdo_num > 4) {
        return false;
    }
    
    uint8_t idx = pdo_num - 1;
    
    /* Inhibit Time이 비활성화되어 있으면 항상 전송 가능 */
    if (!ctx->pdo_inhibit[idx].inhibit_enabled) {
        return true;
    }
    
    /* 첫 전송인 경우 항상 허용 */
    if (ctx->pdo_inhibit[idx].last_tx_tick_us == 0) {
        return true;
    }
    
    /* 경과 시간 계산 */
    uint32_t elapsed_us = current_time_us - ctx->pdo_inhibit[idx].last_tx_tick_us;
    
    /* Inhibit Time 경과 여부 확인 */
    return (elapsed_us >= ctx->pdo_inhibit[idx].inhibit_time_us);
}

/**
 * @brief PDO 전송 완료 기록
 */
void AGR_DOP_MarkPDOSent(AGR_DOP_Ctx_t* ctx, 
                          uint8_t pdo_num, 
                          uint32_t current_time_us)
{
    if (ctx == NULL || pdo_num < 1 || pdo_num > 4) {
        return;
    }
    
    uint8_t idx = pdo_num - 1;
    ctx->pdo_inhibit[idx].last_tx_tick_us = current_time_us;
}

/*============================================================
 * SYNC (동기화 객체)
 *============================================================*/

/**
 * @brief SYNC 기능 활성화
 */
void AGR_DOP_EnableSync(AGR_DOP_Ctx_t* ctx, 
                        uint32_t period_us,
                        void (*on_sync)(void* ctx))
{
    if (ctx == NULL) {
        return;
    }
    
    ctx->sync.sync_enabled = true;
    ctx->sync.sync_period_us = period_us;
    ctx->sync.sync_counter = 0;
    ctx->sync.last_sync_tick = 0;
    ctx->sync.on_sync = on_sync;
}

/**
 * @brief SYNC 기능 비활성화
 */
void AGR_DOP_DisableSync(AGR_DOP_Ctx_t* ctx)
{
    if (ctx == NULL) {
        return;
    }
    
    ctx->sync.sync_enabled = false;
    ctx->sync.on_sync = NULL;
}

/**
 * @brief SYNC 메시지 전송 (Master 모드)
 */
int AGR_DOP_SendSYNC(AGR_DOP_Ctx_t* ctx)
{
    if (ctx == NULL || ctx->tx_func == NULL) {
        return -1;
    }
    
    if (!ctx->sync.sync_enabled) {
        return -2;  /* SYNC 비활성화 상태 */
    }
    
    /* SYNC 카운터 증가 */
    ctx->sync.sync_counter++;
    
    /* SYNC 메시지 전송 (CAN-ID: 0x080, Data: [counter] or empty) */
    uint8_t sync_data[1] = { ctx->sync.sync_counter };
    uint32_t can_id = AGR_CAN_ID_SYNC;
    
    /* CANopen 표준: SYNC는 데이터 없이 전송 가능 (0 bytes) */
    /* 또는 counter 포함 (1 byte) */
    return ctx->tx_func(can_id, sync_data, 1);
}

/**
 * @brief SYNC 메시지 처리 (수신 시 호출, ISR Context)
 */
static void _ProcessSyncMessage(AGR_DOP_Ctx_t* ctx, uint32_t current_ms)
{
    if (!ctx->sync.sync_enabled) {
        return;
    }
    
    /* SYNC 카운터 업데이트 (수신 시) */
    ctx->sync.last_sync_tick = current_ms;
    
    /* 콜백 호출 (ISR 내에서 즉시 실행) */
    if (ctx->sync.on_sync != NULL) {
        ctx->sync.on_sync(ctx->user_ctx);
    }
}

/*============================================================
 * EMERGENCY (긴급 메시지)
 *============================================================*/

/**
 * @brief Emergency 기능 활성화
 */
void AGR_DOP_EnableEmergency(AGR_DOP_Ctx_t* ctx,
                              void (*on_emergency)(uint16_t error_code, void* ctx))
{
    if (ctx == NULL) {
        return;
    }
    
    ctx->emcy.emergency_enabled = true;
    ctx->emcy.last_error_code = 0;
    ctx->emcy.last_emcy_sent_ms = 0;
    ctx->emcy.on_emergency = on_emergency;
}

/**
 * @brief Emergency 메시지 전송
 */
int AGR_DOP_SendEmergency(AGR_DOP_Ctx_t* ctx, uint16_t error_code)
{
    if (ctx == NULL || ctx->tx_func == NULL) {
        return -1;
    }
    
    if (!ctx->emcy.emergency_enabled) {
        return -2;  /* Emergency 비활성화 상태 */
    }
    
    /* Emergency 프레임 구성 (CANopen 표준) */
    uint8_t emcy_data[8] = {
        (uint8_t)(error_code & 0xFF),       /* Error Code Low */
        (uint8_t)(error_code >> 8),         /* Error Code High */
        0x00,                               /* Error Register (0=No error, 1~255=Error) */
        0x00, 0x00, 0x00, 0x00, 0x00       /* Manufacturer Specific Error Field */
    };
    
    /* CAN-ID: 0x080 + Node ID */
    uint32_t can_id = AGR_CAN_ID_EMCY + ctx->node_id;
    
    /* 에러 코드 저장 */
    ctx->emcy.last_error_code = error_code;
    
    /* 전송 */
    return ctx->tx_func(can_id, emcy_data, 8);
}

/**
 * @brief Emergency 메시지 처리 (수신 시 호출, ISR Context)
 */
static void _ProcessEmergencyMessage(AGR_DOP_Ctx_t* ctx, 
                                      const uint8_t* data, 
                                      uint8_t len)
{
    if (!ctx->emcy.emergency_enabled || len < 2) {
        return;
    }
    
    /* Error Code 추출 */
    uint16_t error_code = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    
    /* 콜백 호출 (ISR 내에서 즉시 실행) */
    if (ctx->emcy.on_emergency != NULL) {
        ctx->emcy.on_emergency(error_code, ctx->user_ctx);
    }
}
