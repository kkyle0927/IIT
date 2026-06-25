/**
 ******************************************************************************
 * @file    Basics.c
 * @brief   버튼 기반 제어 로직.
 *          - BTN_1 (좌측) 짧은 클릭으로 SmartAssist on/off 토글
 *            (XM_SetH10AssistExistingMode). H10 동작 모드와 무관하게 동작.
 *
 *  NOTE: 데이터 송신은 시스템의 Total Data(0x20) 자동 스트림이 담당하므로
 *        여기서는 통신 코드를 두지 않는다. SmartAssist 토글 결과는 0x20 패킷의
 *        h10Mode / h10AssistLevel 로 OnePhAI 에 반영된다.
 ******************************************************************************
 */

#include "Basics.h"

SmartAssist_t SmartAssist = SmartAssist_OFF;

void UpdateSmartAssistState(void)
{
    // BTN_1 짧은 클릭이 아니면 상태 유지.
    if (XM_GetButtonEvent(XM_BTN_1) != XM_BTN_CLICK) {
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
