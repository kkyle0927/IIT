/**
 ******************************************************************************
 * @file    agr_sdo_master.c
 * @author  HyundoKim
 * @brief   AGR SDO Master (Client) - Core implementation
 * @version 1.0
 * @date    Jun 19, 2026
 *
 * @details
 * Transaction 풀 기반 Master SDO 응답 추적. 순수 C, transport-agnostic.
 * Design SSOT: docs/plan_sdo_master.md. Types: agr_dop_types.h.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_sdo_master.h"
#include <string.h>

/* data_len 은 Transaction/callback 에서 uint8_t — MAX_DATA_SIZE 가 255 초과면
 * 무음 절단. (CAN-FD 는 64B 이라 무관, UDP/Serial override 대비 guard.) */
_Static_assert(AGR_SDO_MAX_DATA_SIZE <= 255,
    "AGR_SDO_MAX_DATA_SIZE > 255 requires widening Transaction/callback data_len to uint16_t");

/** @brief 응답 페이로드(4B LE)에서 CiA 301 abort code 추출 */
static AGR_SDO_AbortCode_t _ExtractAbort(const AGR_SDO_Msg_t* rsp)
{
    if (rsp->data_len >= 4) {
        return (AGR_SDO_AbortCode_t)((uint32_t)rsp->data[0]
                                   | ((uint32_t)rsp->data[1] << 8)
                                   | ((uint32_t)rsp->data[2] << 16)
                                   | ((uint32_t)rsp->data[3] << 24));
    }
    return AGR_SDO_ABORT_GENERAL;
}

void AGR_SDO_Master_Init(AGR_SDO_Master_t* m, uint32_t (*get_tick_ms)(void))
{
    if (m == NULL) {
        return;
    }
    memset(m, 0, sizeof(*m));   /* 모든 슬롯 AGR_SDO_TX_IDLE(0) */
    m->get_tick_ms = get_tick_ms;
}

AGR_SDO_Transaction_t* AGR_SDO_Master_AllocTx(AGR_SDO_Master_t* m, uint8_t slave_id)
{
    if (m == NULL) {
        return NULL;
    }

    /* single-in-flight per slave: 같은 slave 에 RESERVED/PENDING 있으면 거부 */
    for (int i = 0; i < AGR_SDO_MASTER_MAX_PENDING; ++i) {
        if ((m->pool[i].state == AGR_SDO_TX_PENDING ||
             m->pool[i].state == AGR_SDO_TX_RESERVED) &&
            m->pool[i].slave_id == slave_id) {
            return NULL;
        }
    }

    /* 빈 슬롯 클레임: RESERVED 로 표시(중복 alloc 방지). 호출자(Request)가 모든
     * 필드를 채운 뒤 PENDING 으로 arm — half-filled 슬롯이 ProcessResponse/Tick 에
     * 매칭되지 않도록(claim window 차단). */
    for (int i = 0; i < AGR_SDO_MASTER_MAX_PENDING; ++i) {
        if (m->pool[i].state == AGR_SDO_TX_IDLE) {
            AGR_SDO_Transaction_t* tx = &m->pool[i];
            memset(tx, 0, sizeof(*tx));
            tx->slave_id = slave_id;
            tx->state    = AGR_SDO_TX_RESERVED;
            return tx;
        }
    }

    return NULL;   /* 풀 가득 참 */
}

int32_t AGR_SDO_Master_ProcessResponse(AGR_SDO_Master_t*    m,
                                       uint8_t              from_slave,
                                       const AGR_SDO_Msg_t* rsp)
{
    if (m == NULL || rsp == NULL) {
        return -1;
    }

    for (int i = 0; i < AGR_SDO_MASTER_MAX_PENDING; ++i) {
        AGR_SDO_Transaction_t* tx = &m->pool[i];

        if (tx->state != AGR_SDO_TX_PENDING)            continue;
        if (tx->slave_id != from_slave)                 continue;
        if (tx->index != rsp->index ||
            tx->subindex != rsp->subindex)              continue;

        /* ── 매칭 ── CS + 방향(is_write) 로 결과 분류 ───────────────
         * 응답 CS 가 요청 방향과 맞아야 완료 처리. 불일치(예: write 요청에 read
         * 응답, read 요청에 write-ack)나 미인식 SCS 는 무음 완료하지 않고 abort 로
         * 보고 → caller 가 즉시 인지(timeout 대기 회피). */
        const uint8_t* cb_data = NULL;
        uint8_t        cb_len  = 0;

        if (rsp->cs == AGR_SDO_CS_ABORT) {
            tx->state      = AGR_SDO_TX_ABORTED;
            tx->abort_code = _ExtractAbort(rsp);
        } else if (((rsp->cs & 0xE0) == 0x40) && !tx->is_write) {
            /* Upload(Read) response (0x40 base / 0x41 sized) for a read req — 데이터 전달 */
            uint8_t n = (rsp->data_len <= AGR_SDO_MAX_DATA_SIZE)
                            ? (uint8_t)rsp->data_len : AGR_SDO_MAX_DATA_SIZE;
            if (n > 0) {
                memcpy(tx->data, rsp->data, n);
            }
            tx->data_len = n;
            tx->state    = AGR_SDO_TX_DONE;
            cb_data      = tx->data;
            cb_len       = n;
        } else if (((rsp->cs & 0xE0) == 0x60) && tx->is_write) {
            /* Download(Write) ack (0x60) for a write req */
            tx->data_len = 0;
            tx->state    = AGR_SDO_TX_DONE;
        } else {
            /* 방향/CS 불일치 또는 미인식 SCS — 무음 완료 금지 */
            tx->state      = AGR_SDO_TX_ABORTED;
            tx->abort_code = AGR_SDO_ABORT_INVALID_CS;
        }

        if (tx->on_done != NULL) {
            tx->on_done(tx, cb_data, cb_len, tx->abort_code, tx->user_ctx);
        }
        tx->state = AGR_SDO_TX_IDLE;   /* 콜백 후 슬롯 반환 (tx 이후 무효) */
        return 0;
    }

    return -2;   /* orphan / stale response — 폐기 */
}

void AGR_SDO_Master_Tick(AGR_SDO_Master_t* m, uint32_t current_ms)
{
    /* get_tick_ms==NULL → timeout 미동작 계약 (sent_tick_ms 가 0 으로 고정되어
     * spurious timeout 을 유발하므로 여기서 no-op). */
    if (m == NULL || m->get_tick_ms == NULL) {
        return;
    }

    for (int i = 0; i < AGR_SDO_MASTER_MAX_PENDING; ++i) {
        AGR_SDO_Transaction_t* tx = &m->pool[i];

        if (tx->state != AGR_SDO_TX_PENDING) {
            continue;
        }
        if (tx->timeout_ms == 0u) {
            continue;   /* timeout_ms==0 = no timeout (sentinel) */
        }
        if ((uint32_t)(current_ms - tx->sent_tick_ms) >= tx->timeout_ms) {
            tx->state = AGR_SDO_TX_TIMEOUT;
            if (tx->on_done != NULL) {
                tx->on_done(tx, NULL, 0, AGR_SDO_ABORT_TIMEOUT, tx->user_ctx);
            }
            tx->state = AGR_SDO_TX_IDLE;   /* 슬롯 반환 */
        }
    }
}
