#ifndef BASICS_H
#define BASICS_H

#include "xm_api.h"

/* SmartAssist 토글 상태 (BTN_1 으로 on/off) */
typedef enum {
    SmartAssist_OFF,
    SmartAssist_ON,
} SmartAssist_t;

/* 현재 SmartAssist 상태 (다른 모듈에서 읽기용) */
extern SmartAssist_t SmartAssist;

/* BTN_1 (좌측 버튼) 짧은 클릭으로 SmartAssist on/off 토글.
 * Control_Loop 에서 매 tick 호출한다. */
void UpdateSmartAssistState(void);

#endif /* BASICS_H */
