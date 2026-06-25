/**
 * @file  agr_mw_conf.h
 * @brief  AGR_MW Configuration — SM-IMU (IMU Hub Module, STM32G474RE)
 * @details
 *   AGR_MW submodule configuration for SM-IMU module.
 *   Pattern identical to ioif_conf.h — placed in System/Config/.
 *
 * @copyright Copyright (c) Angel Robotics Inc.
 */

#ifndef AGR_MW_CONF_H_
#define AGR_MW_CONF_H_

#include <stdint.h>

/* -----------------------------------------------------------------------
 * MCU Series Auto-Detection (from CubeMX-generated defines)
 * ----------------------------------------------------------------------- */
#if defined(STM32H743xx) || defined(STM32H750xx)
    #define AGR_MW_MCU_SERIES_H7    1
#elif defined(STM32G474xx) || defined(STM32G431xx)
    #define AGR_MW_MCU_SERIES_G4    1
#else
    #error "AGR_MW: Unsupported MCU — add detection in agr_mw_conf.h"
#endif

/* -----------------------------------------------------------------------
 * Module Enable/Disable (SM-IMU — STM32G474RE)
 * ----------------------------------------------------------------------- */

/** BOOT — disabled (G4 has no dual-bank flash / bootloader) */
/* #define AGR_MW_BOOT_ENABLE */

/* -----------------------------------------------------------------------
 * DOP PDO Configuration
 * ----------------------------------------------------------------------- */

/** @brief PDO Map 최대 엔트리 수 — 6ch × 10필드 + 6 status = 66 (default 32 override) */
#define AGR_PDO_MAP_MAX_ENTRIES     72

/** @brief COBS 최대 프레임 크기 — 246B payload + 4B header + 2B CRC = 252B (default 128 override) */
#define AGR_COBS_MAX_FRAME_SIZE     280

/* -----------------------------------------------------------------------
 * DOP Transport Configuration
 * ----------------------------------------------------------------------- */

/** @brief CAN-FD Transport — XM Master와 DOP 통신 (기존) */
#define AGR_DOP_TRANSPORT_CANFD     1

/** @brief Serial Transport — USB-CDC DOP/Serial 진단 (신규) */
#define AGR_DOP_TRANSPORT_SERIAL    1

/* -----------------------------------------------------------------------
 * API Contract — DO NOT MODIFY
 * -----------------------------------------------------------------------
 * These definitions implement the AGR_MW error return contract.
 * See: docs/agr_mw_error_contract.md
 *
 * Convention:
 *   int32_t rc = AGR_Module_DoThing(...);
 *   if (!AGR_IsOk(rc)) { ... handle error ... }
 *
 *   rc == 0 : OK
 *   rc <  0 : error (module-local typed enum negative values)
 *   rc >  0 : reserved for future "ok-with-info"
 *
 * This macro must stay identical across all consumer modules. When syncing
 * this template into your System/Config/agr_mw_conf.h, preserve this block
 * verbatim.
 * ----------------------------------------------------------------------- */
#define AGR_IsOk(rc)  ((int32_t)(rc) >= 0)

#endif /* AGR_MW_CONF_H_ */