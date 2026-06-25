/**
 ******************************************************************************
 * @file    mcp79510.c
 * @author  HyundoKim
 * @brief   [Device Layer] MCP79510 Real-Time Clock SPI Driver 구현부
 * @version 0.1
 * @date    Mar 02, 2026
 *
 * @details Singleton + Dependency Injection 패턴.
 *          Init 시 IOIF SPI ID를 주입받아 내부 static 변수에 저장.
 *          이후 API 호출 시 인자 없이 내부 ID 사용.
 *
 * @note    MCP79510 SPI Protocol:
 *          - SPI Mode 0 (CPOL=0, CPHA=0), MSB First, ≤10MHz
 *          - Instruction byte first, then address, then data
 *          - Timekeeping registers: BCD encoded
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "mcp79510.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define MCP79510_TIME_REG_COUNT   (7U)  /* SECONDS ~ YEAR */

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

/** @brief Singleton SPI 인스턴스 ID (DI로 주입) */
static IOIF_SPIx_t s_spi_id = IOIF_SPI_ID_NOT_ALLOCATED;

/** @brief 초기화 완료 플래그 */
static bool s_initialized = false;

/**
 *-----------------------------------------------------------
 * PRIVATE HELPER FUNCTIONS
 *-----------------------------------------------------------
 */

/** @brief Decimal → BCD 변환 */
static inline uint8_t _DecToBcd(uint8_t dec)
{
    return (uint8_t)(((dec / 10U) << 4U) | (dec % 10U));
}

/** @brief BCD → Decimal 변환 */
static inline uint8_t _BcdToDec(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4U) & 0x0FU) * 10U + (bcd & 0x0FU));
}

/**
 * @brief MCP79510 레지스터 1바이트 읽기
 * @param[in]  addr 레지스터 주소 (0x00~0x1F)
 * @param[out] value 읽은 값
 */
static AGRBStatusDef _ReadRegister(uint8_t addr, uint8_t *value)
{
    uint8_t tx_buf[3] = { MCP79510_CMD_READ, addr, 0x00 };
    uint8_t rx_buf[3] = { 0 };

    AGRBStatusDef status = ioif_spi.duplex(s_spi_id, tx_buf, rx_buf, 3);
    if (status == AGRBStatus_OK) {
        *value = rx_buf[2];  /* 3번째 바이트가 데이터 */
    }
    return status;
}

/**
 * @brief MCP79510 레지스터 1바이트 쓰기
 * @param[in] addr 레지스터 주소 (0x00~0x1F)
 * @param[in] value 쓸 값
 */
static AGRBStatusDef _WriteRegister(uint8_t addr, uint8_t value)
{
    uint8_t tx_buf[3] = { MCP79510_CMD_WRITE, addr, value };
    return ioif_spi.write(s_spi_id, tx_buf, 3);
}

/**
 * @brief MCP79510 연속 레지스터 읽기
 * @param[in]  start_addr 시작 레지스터 주소
 * @param[out] data 읽은 데이터 버퍼
 * @param[in]  len 읽을 바이트 수
 */
static AGRBStatusDef _ReadBurst(uint8_t start_addr, uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[2] = { MCP79510_CMD_READ, start_addr };

    return ioif_spi.read(s_spi_id, tx_buf, 2, data, len);
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION IMPLEMENTATIONS
 *-----------------------------------------------------------
 */

AGRBStatusDef MCP79510_Init(IOIF_SPIx_t spi_id)
{
    if (spi_id == IOIF_SPI_ID_NOT_ALLOCATED) {
        return AGRBStatus_PARAM_ERROR;
    }

    s_spi_id = spi_id;

    /* 오실레이터 확인 (RTCWKDAY의 OSCRUN bit) */
    uint8_t wkday = 0;
    AGRBStatusDef status = _ReadRegister(MCP79510_REG_WEEKDAY, &wkday);
    if (status != AGRBStatus_OK) {
        return status;
    }

    /* 오실레이터가 정지되어 있으면 시작 */
    if (!(wkday & MCP79510_OSCRUN_BIT)) {
        uint8_t sec = 0;
        status = _ReadRegister(MCP79510_REG_SECONDS, &sec);
        if (status != AGRBStatus_OK) return status;

        /* ST bit (RTCSEC[7]) 설정하여 오실레이터 시작 */
        sec |= MCP79510_ST_BIT;
        status = _WriteRegister(MCP79510_REG_SECONDS, sec);
        if (status != AGRBStatus_OK) return status;
    }

    s_initialized = true;
    return AGRBStatus_OK;
}

AGRBStatusDef MCP79510_SetDateTime(const MCP79510_DateTime_t *dt)
{
    if (!s_initialized || dt == NULL) return AGRBStatus_PARAM_ERROR;

    AGRBStatusDef status;

    /* 1. 오실레이터 중지 (ST=0) — 시간 설정 중 RTC 카운트 방지 */
    uint8_t sec_reg = 0;
    status = _ReadRegister(MCP79510_REG_SECONDS, &sec_reg);
    if (status != AGRBStatus_OK) return status;

    sec_reg &= ~MCP79510_ST_BIT;
    status = _WriteRegister(MCP79510_REG_SECONDS, sec_reg);
    if (status != AGRBStatus_OK) return status;

    /* 2. 시간 레지스터 설정 (BCD 인코딩) */
    status = _WriteRegister(MCP79510_REG_MINUTES, _DecToBcd(dt->minute));
    if (status != AGRBStatus_OK) return status;

    status = _WriteRegister(MCP79510_REG_HOURS, _DecToBcd(dt->hour) & 0x3F);
    if (status != AGRBStatus_OK) return status;

    /* RTCWKDAY: 요일 설정 + VBATEN 보존 */
    uint8_t wkday = 0;
    status = _ReadRegister(MCP79510_REG_WEEKDAY, &wkday);
    if (status != AGRBStatus_OK) return status;
    wkday = (wkday & MCP79510_VBATEN_BIT) | (dt->weekday & 0x07);
    status = _WriteRegister(MCP79510_REG_WEEKDAY, wkday);
    if (status != AGRBStatus_OK) return status;

    status = _WriteRegister(MCP79510_REG_DATE, _DecToBcd(dt->day));
    if (status != AGRBStatus_OK) return status;

    status = _WriteRegister(MCP79510_REG_MONTH, _DecToBcd(dt->month) & 0x1F);
    if (status != AGRBStatus_OK) return status;

    status = _WriteRegister(MCP79510_REG_YEAR, _DecToBcd(dt->year));
    if (status != AGRBStatus_OK) return status;

    /* 3. 초 설정 + 오실레이터 재시작 (ST=1) */
    status = _WriteRegister(MCP79510_REG_SECONDS,
                            _DecToBcd(dt->second) | MCP79510_ST_BIT);

    return status;
}

/**
 * @brief 현재 날짜/시간을 읽어옵니다.
 *
 * [변경] _ReadBurst() → 개별 _ReadRegister() 호출로 전환
 * [근거] ioif_spi.read() (burst) 가 IOIF SPI 레이어 버그로 인해
 *        garbage 데이터를 반환함 (PCA9957과 동일 증상).
 *        ioif_spi.duplex() (개별 3-byte 트랜잭션)는 정상 동작 확인됨.
 * [영향] SPI 트랜잭션 1회 → 7회 증가. 5초 주기 호출이므로 성능 영향 무시 가능.
 */
AGRBStatusDef MCP79510_GetDateTime(MCP79510_DateTime_t *dt)
{
    if (!s_initialized || dt == NULL) return AGRBStatus_PARAM_ERROR;

    uint8_t regs[MCP79510_TIME_REG_COUNT];
    AGRBStatusDef status;

    /* 개별 레지스터 읽기 (ioif_spi.read burst 대신 duplex workaround) */
    for (uint8_t i = 0; i < MCP79510_TIME_REG_COUNT; i++) {
        status = _ReadRegister(MCP79510_REG_SECONDS + i, &regs[i]);
        if (status != AGRBStatus_OK) return status;
    }

    dt->second  = _BcdToDec(regs[0] & 0x7F);  /* Mask ST bit */
    dt->minute  = _BcdToDec(regs[1] & 0x7F);
    dt->hour    = _BcdToDec(regs[2] & 0x3F);  /* 24-hour mode mask */
    dt->weekday = regs[3] & 0x07;
    dt->day     = _BcdToDec(regs[4] & 0x3F);
    dt->month   = _BcdToDec(regs[5] & 0x1F);  /* Mask LPYR bit */
    dt->year    = _BcdToDec(regs[6]);

    return AGRBStatus_OK;
}

bool MCP79510_IsRunning(void)
{
    if (!s_initialized) return false;

    uint8_t wkday = 0;
    AGRBStatusDef status = _ReadRegister(MCP79510_REG_WEEKDAY, &wkday);
    if (status != AGRBStatus_OK) return false;

    return (wkday & MCP79510_OSCRUN_BIT) != 0;
}

AGRBStatusDef MCP79510_DiagReadRaw(uint8_t addr, uint8_t *raw_rx, uint8_t rx_len)
{
    if (!s_initialized) return AGRBStatus_NOT_INITIALIZED;
    uint8_t tx_buf[3] = { MCP79510_CMD_READ, addr, 0x00 };
    uint8_t rx_buf[3] = { 0xAA, 0xAA, 0xAA };  /* 센티널 값 (0xFF 감지용) */
    AGRBStatusDef status = ioif_spi.duplex(s_spi_id, tx_buf, rx_buf, 3);
    if (raw_rx != NULL && rx_len >= 3) {
        raw_rx[0] = rx_buf[0];
        raw_rx[1] = rx_buf[1];
        raw_rx[2] = rx_buf[2];
    }
    return status;
}

AGRBStatusDef MCP79510_EnableBatteryBackup(void)
{
    if (!s_initialized) return AGRBStatus_NOT_INITIALIZED;

    uint8_t wkday = 0;
    AGRBStatusDef status = _ReadRegister(MCP79510_REG_WEEKDAY, &wkday);
    if (status != AGRBStatus_OK) return status;

    if (wkday & MCP79510_VBATEN_BIT) {
        return AGRBStatus_OK;  /* 이미 활성화됨 */
    }

    wkday |= MCP79510_VBATEN_BIT;
    return _WriteRegister(MCP79510_REG_WEEKDAY, wkday);
}
