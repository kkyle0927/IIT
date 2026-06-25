/**
 ******************************************************************************
 * @file    agr_od.c
 * @author  HyundoKim
 * @brief   AGR Object Dictionary - Core Implementation
 * @version 3.0
 * @date    Feb 25, 2026
 *
 * @details
 * DOP Context 독립적인 순수 OD 조작 함수 구현입니다.
 * 모든 함수는 AGR_OD_Table_t 또는 AGR_OD_Entry_t를 직접 받습니다.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_od.h"
#include <string.h>

/*============================================================
 * OD LOOKUP
 *============================================================*/

const AGR_OD_Entry_t* AGR_OD_FindEntry(const AGR_OD_Table_t* od,
                                       uint16_t index)
{
    if (od == NULL || od->entries == NULL) {
        return NULL;
    }

    for (uint16_t i = 0; i < od->entry_count; i++) {
        if (od->entries[i].index == index) {
            return &od->entries[i];
        }
    }

    return NULL;
}

/** @brief 정렬/이진탐색 key — (index << 8) | subindex */
static inline uint32_t _OD_Key(const AGR_OD_Entry_t* e)
{
    return ((uint32_t)e->index << 8) | (uint32_t)e->subindex;
}

const AGR_OD_Entry_t* AGR_OD_FindEntryEx(const AGR_OD_Table_t* od,
                                         uint16_t index,
                                         uint8_t subindex)
{
    if (od == NULL || od->entries == NULL) {
        return NULL;
    }

    /* 정렬 인덱스 보유 시 이진탐색 — O(log n) 상한 (DETERMINISTIC) */
    if (od->sorted_idx != NULL) {
        uint32_t key = ((uint32_t)index << 8) | (uint32_t)subindex;
        uint16_t lo = 0;
        uint16_t hi = od->entry_count;          /* [lo, hi) */
        while (lo < hi) {
            uint16_t mid = (uint16_t)(lo + ((hi - lo) >> 1));
            uint32_t mid_key = _OD_Key(&od->entries[od->sorted_idx[mid]]);
            if (mid_key == key) {
                return &od->entries[od->sorted_idx[mid]];
            }
            if (mid_key < key) {
                lo = (uint16_t)(mid + 1);
            } else {
                hi = mid;
            }
        }
        return NULL;
    }

    /* 인덱스 미구축 모듈 — 기존 선형 탐색 (zero-init 호환 fallback) */
    for (uint16_t i = 0; i < od->entry_count; i++) {
        if (od->entries[i].index == index &&
            od->entries[i].subindex == subindex) {
            return &od->entries[i];
        }
    }

    return NULL;
}

int32_t AGR_OD_BuildSortedIndex(AGR_OD_Table_t* od,
                                uint16_t* index_buf,
                                uint16_t buf_capacity)
{
    if (od == NULL || od->entries == NULL || index_buf == NULL) {
        return -1;
    }
    if (buf_capacity < od->entry_count) {
        return -2;
    }

    od->sorted_idx = NULL;   /* 빌드 실패 시 선형 fallback 유지 */

    /* 삽입정렬 — n<=수백 entry, Init 1회 비용. 외부 라이브러리 의존 없음 */
    for (uint16_t i = 0; i < od->entry_count; i++) {
        uint32_t key = _OD_Key(&od->entries[i]);
        uint16_t j = i;
        while (j > 0 && _OD_Key(&od->entries[index_buf[j - 1]]) > key) {
            index_buf[j] = index_buf[j - 1];
            j--;
        }
        index_buf[j] = i;
    }

    /* 중복 key 검출 — 선형 탐색에서 앞 entry가 뒤를 가리던 잠재 결함을
     * 부팅 시점에 fail loud (인접 비교, 정렬 후 O(n)) */
    for (uint16_t i = 1; i < od->entry_count; i++) {
        if (_OD_Key(&od->entries[index_buf[i - 1]]) ==
            _OD_Key(&od->entries[index_buf[i]])) {
            return -3;
        }
    }

    od->sorted_idx = index_buf;
    return 0;
}

/*============================================================
 * OD VALUE ACCESS
 *============================================================*/

int32_t AGR_OD_ReadValue(const AGR_OD_Entry_t* entry,
                     void* out_buf,
                     uint16_t buf_len)
{
    if (entry == NULL || out_buf == NULL) {
        return -1;
    }

    if (entry->data_ptr == NULL) {
        return -2;
    }

    if (entry->access == AGR_ACCESS_WO) {
        return -3;
    }

    uint16_t copy_len = (buf_len < entry->size) ? buf_len : entry->size;
    memcpy(out_buf, entry->data_ptr, copy_len);

    return copy_len;
}

int32_t AGR_OD_WriteValue(const AGR_OD_Entry_t* entry,
                      const void* in_buf,
                      uint16_t in_len)
{
    if (entry == NULL || in_buf == NULL) {
        return -1;
    }

    if (entry->data_ptr == NULL) {
        return -2;
    }

    if (entry->access == AGR_ACCESS_RO) {
        return -3;
    }

    if (in_len > entry->size) {
        return -4;
    }

    memcpy(entry->data_ptr, in_buf, in_len);

    if (entry->on_write != NULL) {
        entry->on_write();
    }

    return 0;
}
