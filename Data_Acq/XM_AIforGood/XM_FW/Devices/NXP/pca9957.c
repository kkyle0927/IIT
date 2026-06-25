/**
 ******************************************************************************
 * @file    pca9957.c
 * @author  HyundoKim
 * @brief   [Device Layer] PCA9957 24-Channel LED Driver SPI Driver 구현부
 * @version 0.1
 * @date    Mar 02, 2026
 *
 * @details Singleton + Dependency Injection 패턴.
 *          Init 시 IOIF SPI ID + GPIO ID(nRESET, nOE)를 주입받아 내부 static 변수에 저장.
 *
 * @note    PCA9957 SPI Protocol:
 *          - SPI Mode 0 (CPOL=0, CPHA=0), MSB First, ≤10MHz
 *          - Command byte: [A6:A0 << 1 | R/W] (7-bit address)
 *          - Auto-Increment mode for bulk PWM update
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "pca9957.h"
#include <string.h>

#if defined(USE_FREERTOS)
#include "cmsis_os2.h"
#endif

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

/** @brief Singleton SPI 인스턴스 ID (DI로 주입) */
static IOIF_SPIx_t s_spi_id = IOIF_SPI_ID_NOT_ALLOCATED;

/** @brief nRESET GPIO ID */
static IOIF_GPIOx_t s_nreset_id = IOIF_GPIO_NOT_INITIALIZED;

/** @brief nOE GPIO ID */
static IOIF_GPIOx_t s_noe_id = IOIF_GPIO_NOT_INITIALIZED;

/** @brief 초기화 완료 플래그 */
static bool s_initialized = false;

/** @brief 내부 PWM 버퍼 (UpdateAll용 캐시) */
static uint8_t s_pwm_cache[PCA9957_NUM_CHANNELS];

/**
 *-----------------------------------------------------------
 * PRIVATE HELPER FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief SPI 쓰기 (RX FIFO 안전)
 * @details [Workaround] STM32H7 SPI Full-Duplex 모드에서 ioif_spi.write()
 *          (HAL_SPI_Transmit) 사용 시 RX FIFO에 stale 데이터가 남아
 *          이후 duplex(read) 호출 시 데이터 오염 발생.
 *          모든 쓰기를 duplex로 수행하여 RX FIFO를 항상 비움.
 * @note    IOIF SPI 레이어 근본 수정 후 ioif_spi.write()로 복원 가능
 */
static AGRBStatusDef _SpiWrite(uint8_t *tx_buf, uint16_t len)
{
    uint8_t rx_dummy[1 + PCA9957_NUM_CHANNELS]; /* 최대: cmd(1) + 24ch */
    return ioif_spi.duplex(s_spi_id, tx_buf, rx_dummy, len);
}

/**
 * @brief PCA9957 레지스터 1바이트 쓰기
 */
static AGRBStatusDef _WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = { PCA9957_CMD_WRITE(reg), value };
    return _SpiWrite(tx_buf, 2);
}

/**
 * @brief PCA9957 레지스터 1바이트 읽기
 */
static AGRBStatusDef _ReadRegister(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[2] = { PCA9957_CMD_READ(reg), 0x00 };
    uint8_t rx_buf[2] = { 0 };

    AGRBStatusDef status = ioif_spi.duplex(s_spi_id, tx_buf, rx_buf, 2);
    if (status == AGRBStatus_OK) {
        *value = rx_buf[1];  /* 2번째 바이트가 데이터 */
    }
    return status;
}

/**
 * @brief LEDOUT 레지스터 설정 — 전채널 PWM 모드
 * @details LEDOUT0~LEDOUT5 (6개 × 4채널 = 24채널)
 *          각 채널 2bit: 0b10 = Individual PWM mode
 */
static AGRBStatusDef _SetAllChannelsPwmMode(void)
{
    /* 0xAA = 0b10_10_10_10 → 4채널 모두 PWM mode
     * [Workaround] IOIF SPI 벌크 전송(AI) 실패 의심 → 개별 2-byte 전송으로 분리
     * TODO: IOIF 벌크 전송 검증 후 AI write로 복원 */
    AGRBStatusDef status;
    for (uint8_t i = 0; i < 6; i++) {
        status = _WriteRegister(PCA9957_REG_LEDOUT0 + i, 0xAA);
        if (status != AGRBStatus_OK) return status;
    }
    return AGRBStatus_OK;
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION IMPLEMENTATIONS
 *-----------------------------------------------------------
 */

AGRBStatusDef PCA9957_Init(IOIF_SPIx_t spi_id,
                           IOIF_GPIOx_t nreset_id,
                           IOIF_GPIOx_t noe_id)
{
    if (spi_id == IOIF_SPI_ID_NOT_ALLOCATED) {
        return AGRBStatus_PARAM_ERROR;
    }

    s_spi_id = spi_id;
    s_nreset_id = nreset_id;
    s_noe_id = noe_id;

    /* 1. 하드웨어 리셋 */
    PCA9957_Reset();

    /* 2. nOE = HIGH (출력 비활성화 상태에서 설정) */
    PCA9957_Disable();

    /* 3. MODE1: Sleep 해제 + Auto-Increment 활성화 — Write-then-Verify 루프
     *
     * [근거] PCA9957 하드웨어 리셋 후 내부 POR 완료 시간이 datasheet 미기재.
     *        고정 delay(5ms)로는 불충분한 케이스 확인됨 (Live Expression 진단).
     *        → Write → Read-back → 불일치 시 재시도하여 칩 준비 완료를 확정적으로 검증.
     */
    AGRBStatusDef status;
    {
        const uint8_t expected = PCA9957_MODE1_AI2;
        uint8_t readback = 0xFF;

        for (uint8_t retry = 0; retry < 10; retry++) {
            status = _WriteRegister(PCA9957_REG_MODE1, expected);
            if (status != AGRBStatus_OK) return status;

#if defined(USE_FREERTOS)
            osDelay(2);
#else
            HAL_Delay(2);
#endif
            status = _ReadRegister(PCA9957_REG_MODE1, &readback);
            if (status != AGRBStatus_OK) return status;

            if (readback == expected) break;
        }

        if (readback != expected) {
            return AGRBStatus_ERROR;
        }
    }

    /* 4. 전채널 PWM 모드 설정 (LEDOUT0~5) */
    status = _SetAllChannelsPwmMode();
    if (status != AGRBStatus_OK) return status;

    /* 5. 전채널 PWM = 0 (OFF) — 개별 2-byte 전송
     * [Workaround] IOIF SPI 벌크 전송 실패 의심 → 개별 전송 */
    memset(s_pwm_cache, 0, sizeof(s_pwm_cache));
    for (uint8_t ch = 0; ch < PCA9957_NUM_CHANNELS; ch++) {
        status = _WriteRegister(PCA9957_REG_PWM0 + ch, 0);
        if (status != AGRBStatus_OK) return status;
    }

    /* 6. 전채널 전류 설정 (기본값: 최대) */
    // status = _WriteRegister(PCA9957_REG_IREFALL, 0xFF);
    status = _WriteRegister(PCA9957_REG_IREFALL, 0x32);
    if (status != AGRBStatus_OK) return status;

    /* 7. nOE = LOW (출력 활성화) */
    PCA9957_Enable();

    s_initialized = true;
    return AGRBStatus_OK;
}

AGRBStatusDef PCA9957_SetChannelPWM(uint8_t channel, uint8_t pwm)
{
    if (!s_initialized || channel >= PCA9957_NUM_CHANNELS) {
        return AGRBStatus_PARAM_ERROR;
    }

    s_pwm_cache[channel] = pwm;
    return _WriteRegister(PCA9957_REG_PWM0 + channel, pwm);
}

AGRBStatusDef PCA9957_SetRGB(PCA9957_Group_t group, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_initialized || group >= PCA9957_GRP_COUNT) {
        return AGRBStatus_PARAM_ERROR;
    }

    uint8_t base_ch = (uint8_t)(group * 3U);
    AGRBStatusDef status;

    /* 개별 2-byte 전송 (IOIF 벌크 전송 workaround) */
    s_pwm_cache[base_ch]     = r;
    s_pwm_cache[base_ch + 1] = g;
    s_pwm_cache[base_ch + 2] = b;

    status = _WriteRegister(PCA9957_REG_PWM0 + base_ch, r);
    if (status != AGRBStatus_OK) return status;
    status = _WriteRegister(PCA9957_REG_PWM0 + base_ch + 1, g);
    if (status != AGRBStatus_OK) return status;
    return _WriteRegister(PCA9957_REG_PWM0 + base_ch + 2, b);
}

AGRBStatusDef PCA9957_UpdateAll(const uint8_t pwm[PCA9957_NUM_CHANNELS])
{
    if (!s_initialized || pwm == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }

    /* 개별 2-byte 전송 (IOIF 벌크 전송 workaround)
     * TODO: IOIF SPI 벌크(AI) 전송 검증 후 25-byte 일괄 전송으로 복원 */
    memcpy(s_pwm_cache, pwm, PCA9957_NUM_CHANNELS);

    for (uint8_t ch = 0; ch < PCA9957_NUM_CHANNELS; ch++) {
        AGRBStatusDef status = _WriteRegister(PCA9957_REG_PWM0 + ch, pwm[ch]);
        if (status != AGRBStatus_OK) return status;
    }
    return AGRBStatus_OK;
}

AGRBStatusDef PCA9957_SetChannelCurrent(uint8_t channel, uint8_t iref)
{
    if (!s_initialized || channel >= PCA9957_NUM_CHANNELS) {
        return AGRBStatus_PARAM_ERROR;
    }

    return _WriteRegister(PCA9957_REG_IREF0 + channel, iref);
}

void PCA9957_Enable(void)
{
    if (s_noe_id != IOIF_GPIO_NOT_INITIALIZED) {
        IOIF_GPIO_RESET(s_noe_id);  /* nOE = LOW → Output Enable */
    }
}

void PCA9957_Disable(void)
{
    if (s_noe_id != IOIF_GPIO_NOT_INITIALIZED) {
        IOIF_GPIO_SET(s_noe_id);  /* nOE = HIGH → Output Disable */
    }
}

void PCA9957_Reset(void)
{
    if (s_nreset_id != IOIF_GPIO_NOT_INITIALIZED) {
        IOIF_GPIO_RESET(s_nreset_id);  /* nRESET = LOW */
#if defined(USE_FREERTOS)
        osDelay(10);  /* NXP 공식 데모 앱 기준 10ms (PCA9957_RESET_DELAY_MS = 0x0A) */
#else
        HAL_Delay(10);
#endif
        IOIF_GPIO_SET(s_nreset_id);  /* nRESET = HIGH */
#if defined(USE_FREERTOS)
        osDelay(1);
#else
        HAL_Delay(1);
#endif
    }
}

AGRBStatusDef PCA9957_ReadReg(uint8_t reg, uint8_t *value)
{
    return _ReadRegister(reg, value);
}
