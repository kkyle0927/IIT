/**
 ******************************************************************************
 * @file    agr_sdo_protocol.c
 * @author  HyundoKim
 * @brief   AGR SDO Protocol - Core Implementation (Transport-Agnostic)
 * @version 3.0
 * @date    Feb 25, 2026
 *
 * @details
 * CAN-ID / Transport에 의존하지 않는 순수 SDO 프로토콜 구현입니다.
 * 모든 함수는 AGR_OD_Table_t 또는 AGR_SDO_Msg_t를 직접 받습니다.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_sdo_protocol.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

/** @brief SDO 헤더 크기 (cs + index + subindex) */
#define SDO_HEADER_SIZE     4
/** @brief 명시-길이(0x21/0x41) size 필드 크기 (uint16 LE) */
#define SDO_SIZE_FIELD      2
/** @brief 명시-길이 메시지의 data 시작 오프셋 (header 4 + size 2) */
#define SDO_EXPLICIT_OFFSET 6

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static int _ProcessSDOReadRequest(const AGR_OD_Table_t* od,
                                  const AGR_SDO_Msg_t* req,
                                  AGR_SDO_Msg_t* out_rsp);

static int _ProcessSDOWriteRequest(const AGR_OD_Table_t* od,
                                   const AGR_SDO_Msg_t* req,
                                   AGR_SDO_Msg_t* out_rsp);

/*============================================================
 * SDO MESSAGE CREATE
 *============================================================*/

void AGR_SDO_CreateReadReq(AGR_SDO_Msg_t* out_msg,
                           uint16_t index,
                           uint8_t subindex)
{
    if (out_msg == NULL) {
        return;
    }
    
    memset(out_msg, 0, sizeof(AGR_SDO_Msg_t));
    out_msg->cs = AGR_SDO_CS_UPLOAD_INIT_REQ;
    out_msg->index = index;
    out_msg->subindex = subindex;
    out_msg->data_len = 0;
}

void AGR_SDO_CreateWriteReq(AGR_SDO_Msg_t* out_msg,
                            uint16_t index,
                            uint8_t subindex,
                            const void* data,
                            uint16_t data_len)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(AGR_SDO_Msg_t));
    out_msg->cs = AGR_SDO_CS_DOWNLOAD_INIT_REQ_SIZED;  /* 0x21 — 명시 길이 (전 크기 통일) */
    out_msg->index = index;
    out_msg->subindex = subindex;

    if (data != NULL && data_len > 0) {
        uint16_t copy_len = (data_len > AGR_SDO_MAX_DATA_SIZE) ? AGR_SDO_MAX_DATA_SIZE : data_len;
        memcpy(out_msg->data, data, copy_len);
        out_msg->data_len = copy_len;
    }
}

void AGR_SDO_CreateAbortResponse(AGR_SDO_Msg_t* out_rsp,
                                 uint16_t index,
                                 uint8_t subindex,
                                 AGR_SDO_AbortCode_t abort_code)
{
    if (out_rsp == NULL) {
        return;
    }
    
    out_rsp->cs = AGR_SDO_CS_ABORT;
    out_rsp->index = index;
    out_rsp->subindex = subindex;
    
    out_rsp->data[0] = (uint8_t)(abort_code & 0xFF);
    out_rsp->data[1] = (uint8_t)((abort_code >> 8) & 0xFF);
    out_rsp->data[2] = (uint8_t)((abort_code >> 16) & 0xFF);
    out_rsp->data[3] = (uint8_t)((abort_code >> 24) & 0xFF);
    out_rsp->data_len = 4;
}

/*============================================================
 * SDO PROCESS (Slave-side)
 *============================================================*/

int32_t AGR_SDO_ProcessRequest(const AGR_OD_Table_t* od,
                           const AGR_SDO_Msg_t* req,
                           AGR_SDO_Msg_t* out_rsp)
{
    if (od == NULL || req == NULL || out_rsp == NULL) {
        return -1;
    }
    
    switch (req->cs & 0xE0) {
        case 0x40:  /* Upload Initiate (Read) */
            return _ProcessSDOReadRequest(od, req, out_rsp);
            
        case 0x20:  /* Download Initiate (Write) */
            return _ProcessSDOWriteRequest(od, req, out_rsp);
            
        case 0x80:  /* Abort */
            return 0;
            
        default:
            AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                        AGR_SDO_ABORT_INVALID_CS);
            return -2;
    }
}

/*============================================================
 * SDO ENCODE / DECODE
 *============================================================*/

int32_t AGR_SDO_Encode(const AGR_SDO_Msg_t* msg, uint8_t* out_buf, uint16_t out_buf_len)
{
    if (msg == NULL || out_buf == NULL) {
        return -1;
    }

    /* 명시-길이 메시지: Write Req(0x21) / Read Resp(0x41) — byte4-5 에 2B 길이 */
    bool explicit_len = (msg->cs == AGR_SDO_CS_DOWNLOAD_INIT_REQ_SIZED) ||
                        (msg->cs == AGR_SDO_CS_UPLOAD_INIT_RSP_SIZED);

    uint8_t  data_off = explicit_len ? SDO_EXPLICIT_OFFSET : SDO_HEADER_SIZE;
    uint32_t need     = (uint32_t)data_off + msg->data_len;

    /* Download Resp(0x60) / Abort(0x80): 8B 고정 */
    if ((msg->cs & 0xE0) == 0x60 || msg->cs == AGR_SDO_CS_ABORT) {
        need = 8u;
    }

    if (need > out_buf_len) {
        return -2;  /* 버퍼 초과 (CAN-FD: 명시-길이 data > 58B) */
    }

    out_buf[0] = msg->cs;
    out_buf[1] = (uint8_t)(msg->index & 0xFF);
    out_buf[2] = (uint8_t)((msg->index >> 8) & 0xFF);
    out_buf[3] = msg->subindex;

    if (explicit_len) {
        out_buf[4] = (uint8_t)(msg->data_len & 0xFF);
        out_buf[5] = (uint8_t)((msg->data_len >> 8) & 0xFF);
    }
    if (msg->data_len > 0) {
        memcpy(&out_buf[data_off], msg->data, msg->data_len);
    }

    return (int32_t)need;
}

int32_t AGR_SDO_Decode(const uint8_t* in_buf, uint16_t in_len, AGR_SDO_Msg_t* out_msg)
{
    if (in_buf == NULL || out_msg == NULL) {
        return -1;
    }

    if (in_len < SDO_HEADER_SIZE) {
        return -2;
    }

    memset(out_msg, 0, sizeof(AGR_SDO_Msg_t));

    out_msg->cs = in_buf[0];
    out_msg->index = (uint16_t)in_buf[1] | ((uint16_t)in_buf[2] << 8);
    out_msg->subindex = in_buf[3];

    /*
     * [AGR 단일 포맷] 길이는 DLC 가 아니라 byte4-5 의 2B 명시 값에서 확정.
     * 구형(expedited 0x2x / frame-len 0x20 / 0x43|n)은 미지원 → INVALID_CS.
     */

    /* Length-bearing: Write Req(0x21) / Read Resp(0x41) — e=0, s=1, 2B 명시 길이 */
    if (out_msg->cs == AGR_SDO_CS_DOWNLOAD_INIT_REQ_SIZED ||
        out_msg->cs == AGR_SDO_CS_UPLOAD_INIT_RSP_SIZED) {
        if (in_len < SDO_EXPLICIT_OFFSET) {
            return -2;  /* size 필드 미수신 */
        }
        uint16_t sz = (uint16_t)in_buf[4] | ((uint16_t)in_buf[5] << 8);
        if (sz > AGR_SDO_MAX_DATA_SIZE) {
            return -3;  /* oversize → caller 가 abort(DATA_LONG) + diag */
        }
        out_msg->data_len = sz;
        if (sz > 0) {
            memcpy(out_msg->data, &in_buf[SDO_EXPLICIT_OFFSET], sz);
        }
        return 0;
    }

    /* Control: Read Req(0x40) — 데이터 없음 */
    if (out_msg->cs == AGR_SDO_CS_UPLOAD_INIT_REQ) {
        out_msg->data_len = 0;
        return 0;
    }

    /* Control: Download Resp(0x60) / Abort(0x80) — header(+code) 고정 */
    if ((out_msg->cs & 0xE0) == 0x60 || out_msg->cs == AGR_SDO_CS_ABORT) {
        uint16_t n = (in_len > SDO_HEADER_SIZE) ? (uint16_t)(in_len - SDO_HEADER_SIZE) : 0u;
        if (n > AGR_SDO_MAX_DATA_SIZE) {
            n = AGR_SDO_MAX_DATA_SIZE;
        }
        out_msg->data_len = n;
        if (n > 0) {
            memcpy(out_msg->data, &in_buf[SDO_HEADER_SIZE], n);
        }
        return 0;
    }

    /* 구형(expedited / frame-len) — 미지원 */
    return -4;  /* INVALID_CS — caller 가 abort */
}

/*============================================================
 * STATIC FUNCTIONS
 *============================================================*/

/**
 * @brief SDO Read 요청 처리 (CANopen Upload Initiate)
 */
static int _ProcessSDOReadRequest(const AGR_OD_Table_t* od,
                                  const AGR_SDO_Msg_t* req,
                                  AGR_SDO_Msg_t* out_rsp)
{
    const AGR_OD_Entry_t* entry = AGR_OD_FindEntryEx(od, req->index, req->subindex);
    
    if (entry == NULL) {
        AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                    AGR_SDO_ABORT_NOT_EXIST);
        return -1;
    }
    
    if (entry->access == AGR_ACCESS_WO) {
        AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                    AGR_SDO_ABORT_WRITE_ONLY);
        return -2;
    }
    
    out_rsp->index = req->index;
    out_rsp->subindex = req->subindex;
    
    int read_len = AGR_OD_ReadValue(entry, out_rsp->data, AGR_SDO_MAX_DATA_SIZE);
    if (read_len < 0) {
        AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                    AGR_SDO_ABORT_GENERAL);
        return -3;
    }
    
    out_rsp->data_len = (uint16_t)read_len;
    /* 단일 포맷: 전 크기 명시-길이 Read Resp (0x41, e=0 s=1) */
    out_rsp->cs = AGR_SDO_CS_UPLOAD_INIT_RSP_SIZED;

    return 0;
}

/**
 * @brief SDO Write 요청 처리 (CANopen Download Initiate)
 */
static int _ProcessSDOWriteRequest(const AGR_OD_Table_t* od,
                                   const AGR_SDO_Msg_t* req,
                                   AGR_SDO_Msg_t* out_rsp)
{
    const AGR_OD_Entry_t* entry = AGR_OD_FindEntryEx(od, req->index, req->subindex);
    
    if (entry == NULL) {
        AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                    AGR_SDO_ABORT_NOT_EXIST);
        return -1;
    }
    
    if (entry->access == AGR_ACCESS_RO) {
        AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                    AGR_SDO_ABORT_READ_ONLY);
        return -2;
    }
    
    int result = AGR_OD_WriteValue(entry, req->data, req->data_len);
    if (result < 0) {
        AGR_SDO_CreateAbortResponse(out_rsp, req->index, req->subindex,
                                    AGR_SDO_ABORT_GENERAL);
        return -3;
    }
    
    out_rsp->cs = AGR_SDO_CS_DOWNLOAD_INIT_RSP;
    out_rsp->index = req->index;
    out_rsp->subindex = req->subindex;
    out_rsp->data_len = 0;
    
    return 0;
}
