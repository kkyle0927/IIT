/**
 ******************************************************************************
 * @file    agr_od.h
 * @author  HyundoKim
 * @brief   AGR Object Dictionary - Core API
 * @version 3.0
 * @date    Feb 25, 2026
 *
 * @details
 * DOP Context에 의존하지 않는 순수 OD 조작 API입니다.
 * AGR_OD_Table_t를 직접 받아 OD Entry를 검색/읽기/쓰기합니다.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef AGR_OD_H
#define AGR_OD_H

#include "agr_dop_types.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * OD LOOKUP API
 *-----------------------------------------------------------
 */

/**
 * @brief OD Entry 검색 (Index)
 * @param od    Object Dictionary 테이블
 * @param index Object Index
 * @return OD Entry 포인터, 없으면 NULL
 */
const AGR_OD_Entry_t* AGR_OD_FindEntry(const AGR_OD_Table_t* od,
                                       uint16_t index);

/**
 * @brief OD Entry 검색 (Index + SubIndex)
 * @param od       Object Dictionary 테이블
 * @param index    Object Index
 * @param subindex Sub-Index
 * @return OD Entry 포인터, 없으면 NULL
 */
const AGR_OD_Entry_t* AGR_OD_FindEntryEx(const AGR_OD_Table_t* od,
                                         uint16_t index,
                                         uint8_t subindex);

/**
 * @brief OD 정렬 인덱스 빌드 — FindEntryEx를 이진탐색(O(log n))으로 전환
 * @details
 * 드라이버 Init에서 1회 호출 (런타임 중 호출 금지 — lookup과 비동기 갱신 불가).
 * key = (index << 8) | subindex 오름차순으로 entry 위치 인덱스를 정렬해
 * index_buf에 채우고 od->sorted_idx에 연결한다. 삽입정렬 — boot 1회 비용만.
 * 중복 key 검출 시 연결하지 않고 -3 반환 (fail loud — 중복 entry는 선형
 * 탐색에서 앞쪽이 뒤쪽을 가리는 잠재 결함이므로 부팅 시점에 드러낸다).
 *
 * > DETERMINISTIC > OBVIOUS — lookup 상한 O(log n) 보장, 정렬은 Init 1회.
 * > 테이블 소스는 섹션 단위 가독성 순서를 유지한다 (정렬 강제는 인덱스가 흡수).
 *
 * @param od           OD 테이블 (sorted_idx 필드가 채워짐)
 * @param index_buf    호출자 제공 정적 버퍼 (수명 = od 수명, stack 금지)
 * @param buf_capacity index_buf 원소 수 (>= od->entry_count)
 * @return 0=성공, -1=NULL 인자, -2=capacity 부족, -3=중복 (index,subindex)
 */
int32_t AGR_OD_BuildSortedIndex(AGR_OD_Table_t* od,
                                uint16_t* index_buf,
                                uint16_t buf_capacity);

/**
 *-----------------------------------------------------------
 * OD VALUE ACCESS API
 *-----------------------------------------------------------
 */

/**
 * @brief OD Entry 값 읽기
 * @param entry   OD Entry 포인터
 * @param out_buf 출력 버퍼
 * @param buf_len 버퍼 크기
 * @return 읽은 바이트 수, <0=에러 (-1=NULL, -2=no data, -3=write-only)
 */
int32_t AGR_OD_ReadValue(const AGR_OD_Entry_t* entry,
                     void* out_buf,
                     uint16_t buf_len);

/**
 * @brief OD Entry 값 쓰기
 * @param entry  OD Entry 포인터
 * @param in_buf 입력 데이터
 * @param in_len 데이터 길이
 * @return 0=성공, <0=에러 (-1=NULL, -2=no data, -3=read-only, -4=size overflow)
 */
int32_t AGR_OD_WriteValue(const AGR_OD_Entry_t* entry,
                      const void* in_buf,
                      uint16_t in_len);

#endif /* AGR_OD_H */
