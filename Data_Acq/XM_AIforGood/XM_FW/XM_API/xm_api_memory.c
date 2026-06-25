/**
 ******************************************************************************
 * @file    xm_api_memory.c
 * @author  HyundoKim
 * @brief   XM10 메모리 접근 API 구현
 * @version 1.0
 * @date    2026-03-02
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api_memory.h"
#include <string.h>
#include "stm32h7xx_hal.h"

/*
 *==========================================================================
 * PRIVATE DEFINITIONS
 *==========================================================================
 */

/* RAM_D1 User Workspace — 범용, Cacheable */
#define USER_WORKSPACE_SIZE     (200U * 1024U)  /* 200KB */

/* PSRAM User Area — 07 문서 PSRAM 맵: 0x90700000 (7MB offset), 1MB */
#define USER_PSRAM_BASE_ADDR    0x90700000U
#define USER_PSRAM_SIZE         (1U * 1024U * 1024U)  /* 1MB */

/* DTCM User Area — Zero-Wait-State 제어 변수 */
#define USER_DTCM_SIZE          (16U * 1024U)  /* 16KB */

/* Flash User NV — Bank 2, Sector 7 (마지막 섹터) */
#define USER_NV_FLASH_ADDR      0x081E0000U  /* Bank 2 Sector 7 시작 주소 */
#define USER_NV_FLASH_SIZE      (128U * 1024U)  /* 128KB (1 sector) */
#define USER_NV_FLASH_BANK      FLASH_BANK_2
#define USER_NV_FLASH_SECTOR    FLASH_SECTOR_7

/* STM32H7 Flash 최소 기록 단위 (256-bit = 32 bytes) */
#define FLASH_WORD_SIZE         32U

/*
 *==========================================================================
 * STATIC MEMORY BUFFERS
 *==========================================================================
 */

/* RAM_D1 사용자 워크스페이스 (.bss → RAM_D1) */
static uint8_t s_user_workspace[USER_WORKSPACE_SIZE] __attribute__((aligned(8)));

/* DTCM 사용자 영역 (.dtcm_data → DTCMRAM, Zero-Wait-State) */
static uint8_t s_user_dtcm[USER_DTCM_SIZE]
    __attribute__((section(".dtcm_data"), aligned(8)));

/*
 *==========================================================================
 * RAM_D1 USER WORKSPACE
 *==========================================================================
 */

void* XM_GetUserWorkspace(void)
{
    return (void*)s_user_workspace;
}

uint32_t XM_GetUserWorkspaceSize(void)
{
    return USER_WORKSPACE_SIZE;
}

/*
 *==========================================================================
 * PSRAM USER AREA
 *==========================================================================
 */

void* XM_GetUserPSRAM(void)
{
    return (void*)USER_PSRAM_BASE_ADDR;
}

uint32_t XM_GetUserPSRAMSize(void)
{
    return USER_PSRAM_SIZE;
}

/*
 *==========================================================================
 * DTCM USER AREA
 *==========================================================================
 */

void* XM_GetUserDTCM(void)
{
    return (void*)s_user_dtcm;
}

uint32_t XM_GetUserDTCMSize(void)
{
    return USER_DTCM_SIZE;
}

/*
 *==========================================================================
 * FLASH USER NV (Non-Volatile Storage)
 *==========================================================================
 */

uint32_t XM_UserNV_GetSize(void)
{
    return USER_NV_FLASH_SIZE;
}

int32_t XM_UserNV_Read(uint32_t offset, void *data, uint32_t size)
{
    if (data == NULL || (offset + size) > USER_NV_FLASH_SIZE) {
        return -1;
    }

    /* Flash는 Memory-Mapped — 직접 읽기 가능 */
    memcpy(data, (const void*)(USER_NV_FLASH_ADDR + offset), size);
    return 0;
}

int32_t XM_UserNV_Write(uint32_t offset, const void *data, uint32_t size)
{
    if (data == NULL || size == 0 || (offset + size) > USER_NV_FLASH_SIZE) {
        return -1;
    }

    /* 32-byte 정렬 시작 주소 계산 */
    uint32_t aligned_offset = offset & ~(FLASH_WORD_SIZE - 1U);
    uint32_t flash_addr = USER_NV_FLASH_ADDR + aligned_offset;

    /* 정렬 패딩 처리를 위한 임시 버퍼 */
    uint8_t word_buf[FLASH_WORD_SIZE];
    const uint8_t *src = (const uint8_t*)data;

    uint32_t bytes_written = 0;
    uint32_t src_offset = 0;

    HAL_FLASH_Unlock();

    /* offset이 32-byte 정렬되지 않은 경우 첫 워드 처리 */
    if (offset != aligned_offset) {
        uint32_t skip = offset - aligned_offset;
        uint32_t chunk = FLASH_WORD_SIZE - skip;
        if (chunk > size) {
            chunk = size;
        }

        /* 기존 데이터 읽기 + 새 데이터 병합 */
        memcpy(word_buf, (const void*)flash_addr, FLASH_WORD_SIZE);
        memcpy(word_buf + skip, src, chunk);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              flash_addr, (uint32_t)(uintptr_t)word_buf) != HAL_OK) {
            HAL_FLASH_Lock();
            return -2;
        }

        flash_addr += FLASH_WORD_SIZE;
        src_offset += chunk;
        bytes_written += chunk;
    }

    /* 32-byte 정렬된 전체 워드 기록 */
    while ((size - bytes_written) >= FLASH_WORD_SIZE) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              flash_addr, (uint32_t)(uintptr_t)(src + src_offset)) != HAL_OK) {
            HAL_FLASH_Lock();
            return -2;
        }

        flash_addr += FLASH_WORD_SIZE;
        src_offset += FLASH_WORD_SIZE;
        bytes_written += FLASH_WORD_SIZE;
    }

    /* 나머지 바이트 처리 (32-byte 미만) */
    if (bytes_written < size) {
        uint32_t remaining = size - bytes_written;

        memcpy(word_buf, (const void*)flash_addr, FLASH_WORD_SIZE);
        memcpy(word_buf, src + src_offset, remaining);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              flash_addr, (uint32_t)(uintptr_t)word_buf) != HAL_OK) {
            HAL_FLASH_Lock();
            return -2;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

int32_t XM_UserNV_Erase(void)
{
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t sector_error = 0;

    erase_init.TypeErase   = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks       = USER_NV_FLASH_BANK;
    erase_init.Sector      = USER_NV_FLASH_SECTOR;
    erase_init.NbSectors   = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V~3.6V */

    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK) ? 0 : -2;
}

bool XM_UserNV_IsErased(void)
{
    const uint32_t *p = (const uint32_t*)USER_NV_FLASH_ADDR;
    uint32_t words = USER_NV_FLASH_SIZE / sizeof(uint32_t);

    for (uint32_t i = 0; i < words; i++) {
        if (p[i] != 0xFFFFFFFFU) {
            return false;
        }
    }
    return true;
}
