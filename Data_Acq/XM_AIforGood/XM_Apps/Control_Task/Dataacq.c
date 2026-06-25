/**
 ******************************************************************************
 * @file    Dataacq.c
 * @brief   XM -> OnePhAI(phai-studio) 네이티브 송신.
 * @details
 *  - 시스템이 Total Data 패킷(Module 0x20, 365B)을 1kHz 로 자동 스트리밍하므로
 *    별도 전송 코드가 필요 없다. USB 는 기본 PHAI 모드를 유지한다
 *    (XM_USB_SetMode(TERMINAL) 호출 금지 → 호출 시 0x20 자동 스트림이 막힘).
 *  - 트렁크 IMU(IMU Hub sensor[0])는 0x20 안에서 int16(다운스케일)로도 전송되지만,
 *    float32 원본(XM.status.imu_hub.sensor[0])을 패킷의 user_custom float 슬롯
 *    (user_f[0..3], offset 337)에 실어 프로토콜 내 최대(float32) 해상도로 함께 보낸다.
 *
 *  OnePhAI 측: v2.6 데이터맵 사용 시 User_Custom 그룹에서 user_f[] 채널 확인 가능
 *  (tools/codegen/data_map/generated/xm_total_data_map.ts).
 *
 *  NOTE: 같은 폴더의 control_task.c 는 Control_Setup/Control_Loop 가 중복되므로
 *        STM32CubeIDE 에서 Exclude from Build 처리해야 한다.
 ******************************************************************************
 */

#include "xm_api.h"
#include "Basics.h"
#include <math.h>

/* IMU Hub 가속도(g) -> m/s^2 환산 상수 (표준 중력가속도) */
#define STD_GRAVITY_MS2   (9.80665f)

/* Trunk IMU 로 사용할 IMU Hub 센서 채널 (첫 연결 채널 = sensor[0]) */
#define TRUNK_IMU_HUB_CH  (0)

/* =================================================================
 * [필수 구현 1] Control_Setup — 전원 인가 후 1회 실행
 * ================================================================= */
void Control_Setup(void)
{
    /*
     * USB 는 기본 PHAI 모드를 유지한다 (XM_USB_SetMode(TERMINAL) 호출하지 않음).
     * -> 시스템이 Total Data(0x20) 를 1kHz 로 자동 스트리밍, OnePhAI 가 네이티브 디코딩.
     *
     * 패킷 내 user_custom float 슬롯(user_f[0..3]) 라벨을 OnePhAI 에 등록(연결 시 1회).
     * 미등록이어도 v2.6 맵 기본 라벨(user_f[0..3])로 표시되므로 선택 사항.
     */
    XM_SetUsbCustomMeta(0xE0,
        "[{\"slot\":\"f0\",\"name\":\"Trunk AccX\",\"unit\":\"m/s2\"},"
        " {\"slot\":\"f1\",\"name\":\"Trunk AccY\",\"unit\":\"m/s2\"},"
        " {\"slot\":\"f2\",\"name\":\"Trunk AccZ\",\"unit\":\"m/s2\"},"
        " {\"slot\":\"f3\",\"name\":\"Trunk GyrMag\",\"unit\":\"deg/s\"}]");
}

/* =================================================================
 * [필수 구현 2] Control_Loop — 1 ms (1 kHz) 주기 호출
 * ================================================================= */
void Control_Loop(void)
{
    // BTN_1 (좌측) 클릭 → SmartAssist on/off 토글 (제어 동작, 통신과 무관)
    UpdateSmartAssistState();

    /*
     * 트렁크 IMU(IMU Hub sensor[0]) float32 원본을 패킷의 user_f 슬롯에 실어
     * 0x20 프로토콜 내 최대(float32) 해상도로 전송한다.
     *   - user_f[0..2] : 가속도 X/Y/Z (g -> m/s^2 환산)
     *   - user_f[3]    : 자이로 크기 |gyr| (deg/s)
     * (쿼터니언/자이로 3축은 0x20 의 imu_hub_sensor[0] int16 필드로도 함께 전송됨)
     */
    const XmImuHubSensor_t* trunk = &XM.status.imu_hub.sensor[TRUNK_IMU_HUB_CH];

    if (XM.status.imu_hub.is_connected) {
        XM_UserCustom_SetFloat(0, trunk->acc_x * STD_GRAVITY_MS2);
        XM_UserCustom_SetFloat(1, trunk->acc_y * STD_GRAVITY_MS2);
        XM_UserCustom_SetFloat(2, trunk->acc_z * STD_GRAVITY_MS2);

        float gmag = sqrtf(trunk->gyr_x * trunk->gyr_x +
                           trunk->gyr_y * trunk->gyr_y +
                           trunk->gyr_z * trunk->gyr_z);
        XM_UserCustom_SetFloat(3, gmag);
    } else {
        XM_UserCustom_SetFloat(0, 0.0f);
        XM_UserCustom_SetFloat(1, 0.0f);
        XM_UserCustom_SetFloat(2, 0.0f);
        XM_UserCustom_SetFloat(3, 0.0f);
    }
}
