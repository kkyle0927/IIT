/**
 ******************************************************************************
 * @file    user_compat.c
 * @author  HyundoKim
 * @brief   사용자 코드 명명 호환 shim — Control_* ↔ User_*
 * @details
 * 2026-05-15 이전 자산 (User_Setup / User_Loop 함수명 사용) 을
 * Control_Setup / Control_Loop 통일 명명으로 자동 forward.
 *
 * 동작 방식 (weak symbol overlay):
 *   - core_process.c 는 항상 Control_Setup / Control_Loop 만 호출.
 *   - 본 파일의 weak Control_Setup / Control_Loop 가 같은 weak 인
 *     User_Setup / User_Loop 로 디스패치.
 *   - 사용자가 control_task.c 에 strong Control_Setup / Control_Loop 정의 시
 *     본 weak 는 link 시 무시됨 (정상 경로).
 *   - 기존 자산이 strong User_Setup / User_Loop 만 가지고 있다면 본 weak
 *     Control_Setup / Control_Loop 가 살아남아 User_* 로 forward.
 *
 * 같은 파일 안에 두면 strong 정의가 같은 .o 의 weak 를 override 해 죽은 코드가
 * 되므로 — translation unit 분리 (별도 .c) 가 필수.
 *
 * @warning  Backward-compat shim — 다음 major 릴리즈에서 제거 예정.
 *           가능한 빨리 함수명을 Control_Setup / Control_Loop 로 마이그레이션.
 *
 * @version 1.0
 * @date    2026-05-15
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

/* User_* default 정의 — 학생이 둘 다 미정의 시 빌드 통과를 위한 빈 함수. */
__attribute__((weak)) void User_Setup(void) {}
__attribute__((weak)) void User_Loop(void)  {}

/* Control_* default 정의 — 학생이 control_task.c 에 strong 정의 시 무시.
 * 학생이 정의하지 않으면 본 default 가 User_* 로 forward (1 릴리즈 호환). */
__attribute__((weak)) void Control_Setup(void) { User_Setup(); }
__attribute__((weak)) void Control_Loop(void)  { User_Loop();  }
