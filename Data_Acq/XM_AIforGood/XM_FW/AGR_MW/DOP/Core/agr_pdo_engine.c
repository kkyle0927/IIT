/**
 ******************************************************************************
 * @file    agr_pdo_engine.c
 * @author  HyundoKim
 * @brief   AGR PDO Engine - Core Implementation (Transport-Agnostic)
 * @version 3.0
 * @date    Feb 25, 2026
 *
 * @details
 * DOP Context에 의존하지 않는 순수 PDO 엔진 구현입니다.
 * Mapping / Encode / Decode / Inhibit 로직만 포함하며,
 * CAN-ID, tx_func 등 transport 코드는 일절 포함하지 않습니다.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_pdo_engine.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

/** @brief Legacy SDO PDO Mapping Entry 크기 (4B: BitLength + SubIndex + Index LE) */
#define PDO_MAP_SDO_ENTRY_SIZE      4

/** @brief Extended SDO PDO Mapping Entry 크기 (8B: Index + SubIndex + Flags + Length) */
#define PDO_MAP_SDO_ENTRY_EXT_SIZE  8

/** @brief Wire format header byte bit 7 = Extended flag */
#define PDO_MAP_EXT_FLAG            0x80u

/** @brief Wire format header byte bit 0..6 = Number of mapped objects */
#define PDO_MAP_COUNT_MASK          0x7Fu

/*============================================================
 * PDO MAPPING
 *============================================================*/

void AGR_PDO_ClearMap(AGR_PDO_MapTable_t* map)
{
    if (map == NULL) {
        return;
    }
    map->count = 0;
}

int32_t AGR_PDO_AddMap(AGR_PDO_MapTable_t* map,
                   const AGR_OD_Table_t* od,
                   uint16_t index,
                   uint8_t subindex)
{
    return AGR_PDO_AddMapExt(map, od, index, subindex, 0u, 0u);
}

int32_t AGR_PDO_AddMapExt(AGR_PDO_MapTable_t* map,
                      const AGR_OD_Table_t* od,
                      uint16_t index,
                      uint8_t subindex,
                      uint32_t length_bytes,
                      uint8_t flags)
{
    if (map == NULL || od == NULL) {
        return -1;
    }

    if (map->count >= AGR_PDO_MAP_MAX_ENTRIES) {
        return -1;
    }

    const AGR_OD_Entry_t* entry = AGR_OD_FindEntryEx(od, index, subindex);
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

    map->items[map->count].od_index     = index;
    map->items[map->count].od_subindex  = subindex;
    map->items[map->count].flags        = flags;
    map->items[map->count].length_bytes = (length_bytes != 0u)
                                          ? length_bytes
                                          : (uint32_t)entry->size;
    map->count++;

    return 0;
}

int32_t AGR_PDO_ApplyMapFromSDO(AGR_PDO_MapTable_t* map,
                            const AGR_OD_Table_t* od,
                            const uint8_t* data,
                            uint8_t data_len)
{
    if (map == NULL || od == NULL || data == NULL) {
        return -1;
    }

    if (data_len < 1) {
        return -2;
    }

    AGR_PDO_ClearMap(map);

    /*
     * [Wire format — header byte]
     *   bit 7     = Extended flag (0 = Legacy 4B, 1 = Extended 8B)
     *   bit 0..6  = Number of mapped objects (max 127)
     */
    uint8_t  hdr         = data[0];
    bool     is_extended = (hdr & PDO_MAP_EXT_FLAG) != 0u;
    uint8_t  num_objects = hdr & PDO_MAP_COUNT_MASK;
    uint8_t  entry_size  = is_extended ? (uint8_t)PDO_MAP_SDO_ENTRY_EXT_SIZE
                                       : (uint8_t)PDO_MAP_SDO_ENTRY_SIZE;

    /* Count=0 은 매핑 클리어 요청 (CiA 301 해석). 루프 진입 시
     * `added >= num_objects` 조건이 첫 valid entry 를 추가한 뒤에야
     * 평가되어 의도치 않게 1 개가 들어가므로 사전 차단. */
    if (num_objects == 0u) {
        return 0;
    }

    int added = 0;

    for (uint8_t i = 1;
         (i + (entry_size - 1u)) < data_len;
         i += entry_size)
    {
        uint16_t index;
        uint8_t  subindex;
        uint8_t  flags;
        uint32_t length_bytes;

        if (is_extended) {
            /* Extended 8B entry: Index(2) + SubIndex(1) + Flags(1) + Length(4 LE) */
            index        = (uint16_t)data[i] |
                           ((uint16_t)data[i + 1] << 8);
            subindex     = data[i + 2];
            flags        = data[i + 3];
            length_bytes = (uint32_t)data[i + 4]        |
                           ((uint32_t)data[i + 5] << 8) |
                           ((uint32_t)data[i + 6] << 16)|
                           ((uint32_t)data[i + 7] << 24);
        } else {
            /* Legacy 4B entry: BitLength(1) + SubIndex(1) + Index(2 LE) */
            uint8_t bit_len = data[i];
            subindex     = data[i + 1];
            index        = (uint16_t)data[i + 2] |
                           ((uint16_t)data[i + 3] << 8);
            flags        = 0u;
            /* bit_len 을 bytes 로 환산. 0 이면 OD entry->size 를 사용하는
             * 관용 동작을 유지 (deprecation 예정). */
            length_bytes = (bit_len != 0u) ? (uint32_t)(bit_len / 8u) : 0u;
        }

        const AGR_OD_Entry_t* entry = AGR_OD_FindEntryEx(od, index, subindex);
        if (entry == NULL) {
            continue;
        }

        if (map->count >= AGR_PDO_MAP_MAX_ENTRIES) {
            break;
        }

        map->items[map->count].od_index     = index;
        map->items[map->count].od_subindex  = subindex;
        map->items[map->count].flags        = flags;
        map->items[map->count].length_bytes = (length_bytes != 0u)
                                              ? length_bytes
                                              : (uint32_t)entry->size;
        map->count++;
        added++;

        if (added >= num_objects) {
            break;
        }
    }

    return added;
}

/*============================================================
 * PDO ENCODE / DECODE
 *============================================================*/

int32_t AGR_PDO_Encode(const AGR_PDO_MapTable_t* map,
                   const AGR_OD_Table_t* od,
                   uint8_t* out_buf,
                   uint16_t buf_size)
{
    if (map == NULL || od == NULL || out_buf == NULL) {
        return -1;
    }

    uint16_t offset = 0;

    for (uint8_t i = 0; i < map->count; i++) {
        const AGR_PDO_MapItem_t* item = &map->items[i];
        const AGR_OD_Entry_t* entry = AGR_OD_FindEntryEx(od,
                                                         item->od_index,
                                                         item->od_subindex);

        if (entry == NULL || entry->data_ptr == NULL) {
            continue;
        }

        if (offset + entry->size > buf_size) {
            break;
        }

        memcpy(&out_buf[offset], entry->data_ptr, entry->size);
        offset += entry->size;
    }

    return offset;
}

int32_t AGR_PDO_Decode(const AGR_PDO_MapTable_t* map,
                   const AGR_OD_Table_t* od,
                   const uint8_t* in_buf,
                   uint8_t in_len)
{
    if (map == NULL || od == NULL || in_buf == NULL) {
        return -1;
    }

    uint8_t offset = 0;

    for (uint8_t i = 0; i < map->count; i++) {
        const AGR_PDO_MapItem_t* item = &map->items[i];
        const AGR_OD_Entry_t* entry = AGR_OD_FindEntryEx(od,
                                                         item->od_index,
                                                         item->od_subindex);

        if (entry == NULL || entry->data_ptr == NULL) {
            /* OD에 해당 항목이 없어도 PDO 프레임 내 데이터는 존재하므로
             * offset을 mapping의 length_bytes만큼 전진시켜야 후속 필드 정렬 유지.
             * in_len(uint8_t) 를 초과하는 length_bytes 는 이 프레임에 실릴 수
             * 없으므로 UINT8_MAX 로 clamp. */
            uint32_t field_size = item->length_bytes;
            uint8_t  gap        = (field_size > UINT8_MAX) ? UINT8_MAX
                                                           : (uint8_t)field_size;
            offset += gap;
            continue;
        }

        if (offset + entry->size > in_len) {
            break;
        }

        if (entry->access != AGR_ACCESS_RO) {
            memcpy(entry->data_ptr, &in_buf[offset], entry->size);

            if (entry->on_write != NULL) {
                entry->on_write();
            }
        }

        offset += entry->size;
    }

    return offset;
}

/*============================================================
 * PDO INHIBIT TIME
 *============================================================*/

void AGR_PDO_SetInhibitTime(AGR_PDO_Inhibit_t* inhibit,
                            uint32_t inhibit_time_us)
{
    if (inhibit == NULL) {
        return;
    }

    inhibit->inhibit_enabled  = true;
    inhibit->inhibit_time_us  = inhibit_time_us;
    inhibit->last_tx_tick_us  = 0;
}

void AGR_PDO_DisableInhibit(AGR_PDO_Inhibit_t* inhibit)
{
    if (inhibit == NULL) {
        return;
    }

    inhibit->inhibit_enabled = false;
}

bool AGR_PDO_CanSend(const AGR_PDO_Inhibit_t* inhibit,
                     uint32_t current_time_us)
{
    if (inhibit == NULL) {
        return false;
    }

    if (!inhibit->inhibit_enabled) {
        return true;
    }

    if (inhibit->last_tx_tick_us == 0) {
        return true;
    }

    uint32_t elapsed_us = current_time_us - inhibit->last_tx_tick_us;
    return (elapsed_us >= inhibit->inhibit_time_us);
}

void AGR_PDO_MarkSent(AGR_PDO_Inhibit_t* inhibit,
                      uint32_t current_time_us)
{
    if (inhibit == NULL) {
        return;
    }

    inhibit->last_tx_tick_us = current_time_us;
}
