/**
 ******************************************************************************
 * @file    Basics.c
 * @brief   Recording / SavingData 전송 / SmartAssist 토글 등 데이터 수집 공용 로직.
 *          - SavingData_t 패킷을 채워 USB CDC로 전송 (TrunkIMU 포함)
 *          - BTN_2 클릭 또는 EXT_DIO_3 HIGH 신호로 Recording ON/OFF
 *          - BTN_1 클릭으로 SmartAssist 토글 (H10 동작 모드와 무관)
 *
 *  Rev2.0 SDK 용 (Control_Setup/Control_Loop 1 kHz). Rev1.1 (legacy) 에서 이식.
 ******************************************************************************
 */

#include "Basics.h"

void FillAndSendSavingData(void)
{
    // 1) loopCnt 채우고 증가
    SavingData.loopCnt = s_dataSaveLoopCnt++;

    // 2) payload 필드 채우기
    SavingData.h10Mode        = (uint8_t)XM.status.h10.h10Mode;
    SavingData.h10AssistLevel = XM.status.h10.h10AssistLevel;
    SavingData.SmartAssist    = SmartAssist;

    SavingData.leftHipAngle   = XM.status.h10.leftHipAngle;
    SavingData.rightHipAngle  = XM.status.h10.rightHipAngle;
    SavingData.leftThighAngle = XM.status.h10.leftThighAngle;
    SavingData.rightThighAngle= XM.status.h10.rightThighAngle;

    SavingData.leftHipTorque      = XM.status.h10.leftHipTorque;
    SavingData.rightHipTorque     = XM.status.h10.rightHipTorque;
    SavingData.leftHipMotorAngle  = XM.status.h10.leftHipMotorAngle;
    SavingData.rightHipMotorAngle = XM.status.h10.rightHipMotorAngle;

    SavingData.leftHipImuGlobalAccX = XM.status.h10.leftHipImuGlobalAccX;
    SavingData.leftHipImuGlobalAccY = XM.status.h10.leftHipImuGlobalAccY;
    SavingData.leftHipImuGlobalAccZ = XM.status.h10.leftHipImuGlobalAccZ;
    SavingData.leftHipImuGlobalGyrX = XM.status.h10.leftHipImuGlobalGyrX;
    SavingData.leftHipImuGlobalGyrY = XM.status.h10.leftHipImuGlobalGyrY;
    SavingData.leftHipImuGlobalGyrZ = XM.status.h10.leftHipImuGlobalGyrZ;

    SavingData.rightHipImuGlobalAccX = XM.status.h10.rightHipImuGlobalAccX;
    SavingData.rightHipImuGlobalAccY = XM.status.h10.rightHipImuGlobalAccY;
    SavingData.rightHipImuGlobalAccZ = XM.status.h10.rightHipImuGlobalAccZ;
    SavingData.rightHipImuGlobalGyrX = XM.status.h10.rightHipImuGlobalGyrX;
    SavingData.rightHipImuGlobalGyrY = XM.status.h10.rightHipImuGlobalGyrY;
    SavingData.rightHipImuGlobalGyrZ = XM.status.h10.rightHipImuGlobalGyrZ;

    SavingData.TrunkIMU_QuatW = XM.status.ext_imu.q_w;
    SavingData.TrunkIMU_QuatX = XM.status.ext_imu.q_x;
    SavingData.TrunkIMU_QuatY = XM.status.ext_imu.q_y;
    SavingData.TrunkIMU_QuatZ = XM.status.ext_imu.q_z;

    SavingData.TrunkIMU_LocalAccX = XM.status.ext_imu.acc_x;
    SavingData.TrunkIMU_LocalAccY = XM.status.ext_imu.acc_y;
    SavingData.TrunkIMU_LocalAccZ = XM.status.ext_imu.acc_z;
    SavingData.TrunkIMU_LocalGyrX = XM.status.ext_imu.gyr_x;
    SavingData.TrunkIMU_LocalGyrY = XM.status.ext_imu.gyr_y;
    SavingData.TrunkIMU_LocalGyrZ = XM.status.ext_imu.gyr_z;

    // 3) 헤더/CRC 설정
    SavingData.sof = TRANSMIT_SOF;
    SavingData.len = (uint16_t)sizeof(SavingData_t);

    // crc 자신을 제외한 전체에 대해 CRC 계산
    uint16_t crc = CalcCrc16((const uint8_t*)&SavingData,
                              sizeof(SavingData_t) - sizeof(SavingData.crc));
    SavingData.crc = crc;

    // 4) 전송 (리턴값은 필요하면 체크)
    (void)XM_SendUsbData(&SavingData, sizeof(SavingData_t));
}

uint16_t CalcCrc16(const uint8_t* data, uint32_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void UpdateRecordingState(void)
{
    RecordingState_t prev = Recording;

    XM_button2  = XM_GetButtonEvent(XM_BTN_2);
    sync_signal = XM_DigitalRead(XM_EXT_DIO_3);

    if (sync_signal == XM_HIGH) {
        Recording = Record_ON;   // SYNC HIGH → 강제 ON
    } else {
        if (XM_button2 == XM_BTN_CLICK) {
            // 토글
            Recording = (Recording == Record_ON) ? Record_OFF : Record_ON;
        }
        // 버튼이 없으면 기존 상태 유지
    }

    // SYNC H->L 또는 ON->OFF 시 카운터 리셋
    if (sync_signal_pre == XM_HIGH && sync_signal == XM_LOW) {
        Recording = Record_OFF;
        s_dataSaveLoopCnt = 0;
    } else if (prev == Record_ON && Recording == Record_OFF) {
        s_dataSaveLoopCnt = 0;
    }

    sync_signal_pre = sync_signal;

    if (Recording == Record_ON) {
        XM_SetLedEffect(3, XM_LED_HEARTBEAT, 1000);
        FillAndSendSavingData();
    } else {
        XM_SetLedEffect(3, XM_LED_OFF, 1000);
    }
}

void UpdateSmartAssistState(void)
{
    // BTN_1 (좌측 버튼) 짧은 클릭으로 SmartAssist 토글.
    // H10 동작 모드(STANDBY/ASSIST)와 무관하게 항상 동작한다.
    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    if (XM_button1 != XM_BTN_CLICK) {
        return;
    }

    if (SmartAssist == SmartAssist_ON) {
        SmartAssist = SmartAssist_OFF;
        XM_SetH10AssistExistingMode(false);
    } else {
        SmartAssist = SmartAssist_ON;
        XM_SetH10AssistExistingMode(true);
    }
}
