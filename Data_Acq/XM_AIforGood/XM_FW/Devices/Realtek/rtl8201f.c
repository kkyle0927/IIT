/**
 ******************************************************************************
 * @file    rtl8201f.c
 * @author  Angel Robotics Firmware Team (HyundoKim)
 * @brief   [Device Layer] RTL8201F 10/100 Ethernet PHY 드라이버 구현
 * @version 1.0
 * @date    Mar 04, 2026
 *
 * @details
 * WS5(KAIST_WALKON5) 프로젝트에서 검증된 RTL8201F 드라이버를
 * Arch V2 Device Driver 패턴(Singleton + IOIF ETH MDIO)으로 리팩토링.
 *
 * [Init 시퀀스] (WS5 검증 완료)
 * 1. MDIO Clock Range 설정
 * 2. PHY ID 확인 (PHYID1 = 0x001C)
 * 3. SW Reset (BCR bit 15, 500ms timeout)
 * 4. Page 7: LED 모드 설정
 * 5. Page 7: RMII 타이밍 설정 (TX=0x0500, RX=0x0040) ← 핵심
 * 6. Page 0 복귀
 * 7. 안정화 대기 (2000ms)
 *
 * [WS5 대비 버그 수정]
 * - SoftReset: timeout 발생 시 즉시 return (WS5는 break 없이 계속 loop)
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "rtl8201f.h"
#include "rtl8201f_regmap.h"
#include "cmsis_os2.h"

/**
 *===========================================================================
 * SINGLETON STATE (Arch V2 Device 패턴)
 *===========================================================================
 */

static IOIF_ETH_MDIOx_t s_mdio_id = IOIF_ETH_MDIO_NOT_ALLOCATED;
static uint32_t s_is_initialized = 0;

/**
 *===========================================================================
 * PRIVATE FUNCTIONS (Arch V2: _PascalCase)
 *===========================================================================
 */

/**
 * @brief PHY 레지스터 읽기 (IOIF MDIO 래핑)
 */
static int32_t _ReadReg(uint16_t addr, uint32_t* readval)
{
    if (ioif_eth_mdio.read(s_mdio_id, RTL8201F_PHY_ADDR, addr, readval) != AGRBStatus_OK) {
        return RTL8201F_STATUS_READ_ERROR;
    }
    return RTL8201F_STATUS_OK;
}

/**
 * @brief PHY 레지스터 쓰기 (IOIF MDIO 래핑)
 */
static int32_t _WriteReg(uint16_t addr, uint32_t writeval)
{
    if (ioif_eth_mdio.write(s_mdio_id, RTL8201F_PHY_ADDR, addr, writeval) != AGRBStatus_OK) {
        return RTL8201F_STATUS_WRITE_ERROR;
    }
    return RTL8201F_STATUS_OK;
}

/**
 * @brief Read-Modify-Write 레지스터 설정
 * @param addr  레지스터 주소
 * @param mask  클리어할 비트 마스크
 * @param flag  설정할 비트 값
 */
static int32_t _RegSetting(uint16_t addr, uint16_t mask, uint16_t flag)
{
    uint32_t regval = 0;
    int32_t status;

    status = _ReadReg(addr, &regval);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }

    regval &= ~((uint32_t)mask);
    regval |= (uint32_t)flag;

    status = _WriteReg(addr, regval);

    return status;
}

/**
 * @brief PHY Software Reset (BCR bit 15)
 * @details BCR에 SOFT_RST 비트를 설정하고 자동 클리어될 때까지 대기.
 *
 * [WS5 대비 버그 수정]
 * WS5: timeout 발생 시 status = RESET_TIMEOUT 설정 후 break 없이 계속 loop → 무한 대기 가능
 * XM:  timeout 발생 시 즉시 return RESET_TIMEOUT
 *
 * @param timeout  최대 대기 시간 (ms)
 */
static int32_t _SoftReset(uint32_t timeout)
{
    uint32_t regval = 0;
    int32_t status;
    uint32_t tickstart;

    /* SW Reset 시작 */
    status = _WriteReg(RTL8201F_BCR, RTL8201F_BCR_SOFT_RST);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }

    /* Reset 완료 대기 (BCR bit 15 자동 클리어) */
    tickstart = osKernelGetTickCount();

    do {
        status = _ReadReg(RTL8201F_BCR, &regval);
        if (status != RTL8201F_STATUS_OK) {
            return status;
        }

        /* === [BugFix vs WS5] timeout 시 즉시 return === */
        if ((osKernelGetTickCount() - tickstart) > timeout) {
            return RTL8201F_STATUS_RESET_TIMEOUT;
        }

        osDelay(1);
    } while (regval & RTL8201F_BCR_SOFT_RST);

    return RTL8201F_STATUS_OK;
}

/**
 *===========================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================
 */

int32_t RTL8201F_Init(IOIF_ETH_MDIOx_t mdio_id)
{
    uint32_t regvalue = 0;
    int32_t status = RTL8201F_STATUS_OK;

    /* Singleton: MDIO ID 저장 */
    s_mdio_id = mdio_id;
    s_is_initialized = 0;

    /* 1. MDIO Clock Range 설정 (AHB 클럭 기반 분주비 자동 설정) */
    ioif_eth_mdio.set_clock_range(s_mdio_id);

    /* 2. PHY ID 확인 (PHYID1 Register = 0x001C for RTL8201F) */
    status = _ReadReg(RTL8201F_PHYI1R, &regvalue);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }
    if (regvalue != RTL8201F_PHYI1R_OUI_HIGH) {
        return RTL8201F_STATUS_ADDRESS_ERROR;
    }

    /* 3. SW Reset (BCR bit 15, 500ms timeout) */
    status = _SoftReset(RTL8201F_SW_RESET_TIMEOUT);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }

    /* 4. Page 7 선택 */
    status = _RegSetting(RTL8201F_PSR, RTL8201F_PSR_MASK, RTL8201F_PSR_P7);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }
    osDelay(RTL8201F_REG_SETTING_DELAY);

    /* 5. LED 모드 설정 */
    status = _RegSetting(RTL8201F_P7_LED, RTL8201F_P7_LED_MASK, RTL8201F_P7_LED_ACK_ALL_LINK_100);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }
    osDelay(RTL8201F_REG_SETTING_DELAY);

    /* 6. RMII 타이밍 설정 (핵심 — WS5 검증 값)
     * TX timing = 0x0500, RX timing = 0x0040
     * Why: RTL8201F REF_CLK 출력 안정화에 필요. 미설정 시 SWR TIMEOUT 발생. */
    status = _RegSetting(RTL8201F_P7_RMSR,
        RTL8201F_P7_RMSR_RMII_MODE_MASK | RTL8201F_P7_RMSR_TX_TIMING_MASK | RTL8201F_P7_RMSR_RX_TIMING_MASK,
        RTL8201F_P7_RMSR_SET_RMII_MODE  | RTL8201F_P7_RMSR_TX_TIMING      | RTL8201F_P7_RMSR_RX_TIMING);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }
    osDelay(RTL8201F_REG_SETTING_DELAY);

    /* 7. Page 0 복귀 */
    status = _RegSetting(RTL8201F_PSR, RTL8201F_PSR_MASK, RTL8201F_PSR_P0);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }

    /* 8. 안정화 대기 (WS5: 2000ms — PHY 링크 안정화 + REF_CLK 안정) */
    osDelay(RTL8201F_INIT_TIMEOUT);

    s_is_initialized = 1;
    return RTL8201F_STATUS_OK;
}


int32_t RTL8201F_GetLinkState(void)
{
    uint32_t bsrval = 0, bcrval = 0;
    uint8_t speed100 = 0, duplexfull = 0;
    int32_t status = RTL8201F_STATUS_OK;

    /* BSR 2회 읽기 (IEEE 802.3: Link Status는 latching-low, 2회 읽기로 현재 상태 확인) */
    status = _ReadReg(RTL8201F_BSR, &bsrval);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }
    status = _ReadReg(RTL8201F_BSR, &bsrval);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }

    /* Link Status 확인 */
    if ((bsrval & RTL8201F_BSR_LINK_STATUS) == 0) {
        return RTL8201F_STATUS_LINK_DOWN;
    }

    /* BCR 읽기 (Auto-nego 활성화 여부 확인) */
    status = _ReadReg(RTL8201F_BCR, &bcrval);
    if (status != RTL8201F_STATUS_OK) {
        return status;
    }

    if ((bcrval & RTL8201F_BCR_AUTONEGO_EN) != RTL8201F_BCR_AUTONEGO_EN) {
        /* Auto-nego 비활성: BCR에서 직접 speed/duplex 읽기 */
        speed100   = (bcrval & RTL8201F_BCR_SPEED_SELECT) == RTL8201F_BCR_SPEED_SELECT;
        duplexfull = (bcrval & RTL8201F_BCR_DUPLEX_MODE)  == RTL8201F_BCR_DUPLEX_MODE;

        if (speed100 && duplexfull) {
            return RTL8201F_STATUS_100MBITS_FULLDUPLEX;
        } else if (speed100) {
            return RTL8201F_STATUS_100MBITS_HALFDUPLEX;
        } else if (duplexfull) {
            return RTL8201F_STATUS_10MBITS_FULLDUPLEX;
        } else {
            return RTL8201F_STATUS_10MBITS_HALFDUPLEX;
        }
    } else {
        /* Auto-nego 활성: 완료 여부 확인 */
        if ((bsrval & RTL8201F_BSR_AUTONEGO_CPLT) == 0) {
            return RTL8201F_STATUS_AUTONEGO_NOTDONE;
        }

        /* Auto-nego 완료 → ANLPAR에서 실제 협상 결과 읽기
         * [BugFix vs WS5] WS5는 무조건 100M/Full 반환 → 10M 협상 시 MAC/PHY mismatch.
         * ANLPAR의 최고 공통 능력(Highest Common Denominator)으로 결정. */
        uint32_t anlpar = 0;
        status = _ReadReg(RTL8201F_ANLPAR, &anlpar);
        if (status != RTL8201F_STATUS_OK) {
            /* ANLPAR 읽기 실패 시 100M/Full 폴백 (WS5 호환) */
            return RTL8201F_STATUS_100MBITS_FULLDUPLEX;
        }

        if (anlpar & RTL8201F_ANLPAR_100BASE_TX_FD) {
            return RTL8201F_STATUS_100MBITS_FULLDUPLEX;
        } else if (anlpar & RTL8201F_ANLPAR_100BASE_TX) {
            return RTL8201F_STATUS_100MBITS_HALFDUPLEX;
        } else if (anlpar & RTL8201F_ANLPAR_10BASE_T_FD) {
            return RTL8201F_STATUS_10MBITS_FULLDUPLEX;
        } else if (anlpar & RTL8201F_ANLPAR_10BASE_T) {
            return RTL8201F_STATUS_10MBITS_HALFDUPLEX;
        }
        /* 매칭 실패 시 100M/Full 폴백 */
        return RTL8201F_STATUS_100MBITS_FULLDUPLEX;
    }
}
