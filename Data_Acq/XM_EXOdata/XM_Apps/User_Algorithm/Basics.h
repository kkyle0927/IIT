#ifndef BASICS_H
#define BASICS_H

#include "xm_api.h"
#include "mti-630.h"
#include <stdint.h>
#include <stdbool.h>

/* --- 공통 enum / struct --- */
typedef enum {
    Record_OFF,
    Record_ON,
} RecordingState_t;

typedef enum {
    Enable_OFF,
    Enable_ON,
} Enable_t;

typedef enum {
    SmartAssist_OFF,
    SmartAssist_ON,
} SmartAssist_t;

typedef struct __attribute__((packed)) {
    uint16_t sof;
    uint16_t len;

    uint32_t loopCnt;
    uint8_t  h10Mode;
    uint8_t  h10AssistLevel;
    uint8_t  SmartAssist;

    float leftHipAngle,  rightHipAngle;
    float leftThighAngle, rightThighAngle;

    float leftHipTorque,  rightHipTorque;
    float leftHipMotorAngle, rightHipMotorAngle;

    float leftHipImuGlobalAccX, leftHipImuGlobalAccY, leftHipImuGlobalAccZ;
    float leftHipImuGlobalGyrX, leftHipImuGlobalGyrY, leftHipImuGlobalGyrZ;
    float rightHipImuGlobalAccX, rightHipImuGlobalAccY, rightHipImuGlobalAccZ;
    float rightHipImuGlobalGyrX, rightHipImuGlobalGyrY, rightHipImuGlobalGyrZ;

    float TrunkIMU_LocalAccX, TrunkIMU_LocalAccY, TrunkIMU_LocalAccZ;
    float TrunkIMU_LocalGyrX, TrunkIMU_LocalGyrY, TrunkIMU_LocalGyrZ;
    float TrunkIMU_QuatW, TrunkIMU_QuatX, TrunkIMU_QuatY, TrunkIMU_QuatZ;

    float leftFSR1, leftFSR2, leftFSR3, leftFSR4, leftFSR5, leftFSR6;
    float leftFSR7, leftFSR8, leftFSR9, leftFSR10, leftFSR11, leftFSR12;
    float leftFSR13, leftFSR14;

    float rightFSR1, rightFSR2, rightFSR3, rightFSR4, rightFSR5, rightFSR6;
    float rightFSR7, rightFSR8, rightFSR9, rightFSR10, rightFSR11, rightFSR12;
    float rightFSR13, rightFSR14;

    uint16_t crc;
} SavingData_t;

/* --- 공유 전역 변수 extern --- */
extern uint32_t      s_dataSaveLoopCnt;
extern SavingData_t  SavingData;
extern XmLogicLevel_t sync_signal;
extern XmLogicLevel_t sync_signal_pre;

extern XmBtnEvent_t XM_button1;
extern XmBtnEvent_t XM_button2;
extern XmBtnEvent_t XM_button3;

extern RecordingState_t Recording;
extern SmartAssist_t   SmartAssist;

extern bool xsensIMUenableRes;

#define TRANSMIT_SOF  (0xAA55u)

/* --- 함수 프로토타입 --- */
void     UpdateXsensImuEnable(void);
void     FillAndSendSavingData(void);
uint16_t CalcCrc16(const uint8_t* data, uint32_t length);
void     UpdateRecordingState(void);
void     UpdateSmartAssistState(void);

#endif /* BASICS_H */
