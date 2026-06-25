/**
 ******************************************************************************
 * @file    imu_node_id.h
 * @author  HyundoKim
 * @brief   IMU Hub CAN-FD Node ID Definitions — SUIT H10 System
 * @version 1.0
 * @date    Apr 16, 2026
 *
 * @details
 * IMU Hub이 속한 SUIT H10 시스템의 CAN-FD Node ID 할당.
 * CANopen CiA 301 호환 (7-bit, 0x01~0x7F).
 *
 * [역할] IMU Hub 은 Slave (XM 하위)
 * [구조] IMU Hub 은 단일 Node ID 0x0D 로 동작하며 TPDO1/TPDO2 두 개를
 *        동시 송출 (CiA 301 표준 — 한 노드가 최대 4개 TPDO 지원).
 *        6ch × {q,a,g} = 120B 를 한 CAN-FD 프레임(≤64B)에 못 담으므로:
 *          - TPDO1 (CAN-ID 0x18D): IMU 0/1/2 + metadata
 *          - TPDO2 (CAN-ID 0x28D): IMU 3/4/5 + metadata
 *        두 프레임 모두 **Node 0x0D 로부터**의 TPDO 로 해석됨.
 *
 * @note Node ID는 CAN-FD 전용이며 로봇 시스템별로 다르다.
 *       AGR_MW (middleware) 가 아닌 System Layer 에서 관리한다.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd.
 ******************************************************************************
 */

#pragma once

#ifndef IMU_NODE_ID_H
#define IMU_NODE_ID_H

#include <stdint.h>

typedef uint8_t AGR_NodeID_t;

/* Broadcast / Special */
#define AGR_NODE_ID_BROADCAST        0x00

/* Master */
#define AGR_NODE_ID_XM               0x02  /**< Extension Module (Master) */

/* Self */
#define AGR_NODE_ID_IMU_HUB          0x0D  /**< IMU Hub (자기 자신, TPDO1/TPDO2 송신) */

/* Utility */
#define AGR_NODE_ID_IS_VALID(id)     ((id) >= 0x01 && (id) <= 0x7F)
#define AGR_NODE_ID_IS_BROADCAST(id) ((id) == AGR_NODE_ID_BROADCAST)

#endif /* IMU_NODE_ID_H */
