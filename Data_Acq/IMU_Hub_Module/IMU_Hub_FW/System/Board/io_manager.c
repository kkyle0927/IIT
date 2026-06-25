/**
 ******************************************************************************
 * @file    io_manager.c
 * @author  HyundoKim
 * @brief   [System/BOARD] I/O (LED & Button) 통합 관리자 구현
 * @version 1.0
 * @date    Dec 2, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "io_manager.h"
#include "ioif_agrb_gpio.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define BUTTON_DEBOUNCE_MS    50    /**< 디바운싱 시간 */
#define BUTTON_LONG_PRESS_MS  1000  /**< 롱프레스 기준 시간 */

/**
 *-----------------------------------------------------------
 * PRIVATE TYPES
 *-----------------------------------------------------------
 */

/* Button State Machine */
typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_DEBOUNCE,
    BTN_STATE_PRESSED,
    BTN_STATE_LONG_HOLD,
} ButtonState_t;

typedef struct {
    IOIF_GPIOx_t gpio_id;
    ButtonState_t state;
    uint32_t start_time;
    IO_ButtonEvent_t last_event;
} ButtonCtx_t;

/* LED Context */
typedef struct {
    IOIF_GPIOx_t gpio_id;
    IO_LedMode_t mode;
    uint32_t period_ms;
    uint32_t last_toggle_time;
    bool is_on;
    uint32_t start_time;
} LedCtx_t;

/* Status RGB LED */
typedef struct {
    IOIF_GPIOx_t r_id;
    IOIF_GPIOx_t g_id;
    IOIF_GPIOx_t b_id;
    IO_LinkState_t current_state;
    IO_LedMode_t mode;
    uint32_t period_ms;              /* BLINK/FLICKERING용 반주기 */
    uint32_t last_toggle_time;
    bool blink_state;
    uint8_t color_mask;              /* R(bit0) | G(bit1) | B(bit2) */
    /* Phase-based Flash 패턴 */
    uint8_t phase;                   /* 현재 phase index */
    uint8_t phase_count;             /* 총 phase 수 */
    const uint16_t* phase_durations; /* phase별 duration 배열 */
} StatusLedCtx_t;

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

/* GPIO IDs */
static IOIF_GPIOx_t s_power_led_id = IOIF_GPIO_NOT_INITIALIZED;

/* Status LED (RGB) */
static StatusLedCtx_t s_status_led;

/* Function Buttons (2개) */
static ButtonCtx_t s_buttons[2];

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static void _UpdateButton(ButtonCtx_t* btn, uint32_t now);
static void _UpdateStatusLed(uint32_t now);
static void _ApplyRgbColor(uint8_t mask);

/* ===== CiA 303-3 Flash Phase Durations ===== */
/* Phase 0=ON, 1=OFF, 2=ON, ... (교대) */
static const uint16_t s_single_flash_phases[] = { 200, 1000 };
static const uint16_t s_double_flash_phases[] = { 200, 200, 200, 1000 };

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

void IO_Manager_Init(void)
{
    /* ===== GPIO 핀 할당 ===== */
    
    /* Power LED (PC6) - Output */
    IOIF_GPIO_INITIALIZE(s_power_led_id, GPIOC, GPIO_PIN_6, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_Initialize_t power_led_config = {
        .mode = IOIF_GPIO_Mode_Output,
        .pull = IOIF_GPIO_Floating,
        .init_state = false,  /* 초기 OFF */
    };
    IOIF_GPIO_REINITIALIZE(s_power_led_id, &power_led_config);
    
    /* Status LED RGB (PC7, PC8, PC9) - Output */
    IOIF_GPIO_INITIALIZE(s_status_led.r_id, GPIOC, GPIO_PIN_7, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_status_led.g_id, GPIOC, GPIO_PIN_8, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_status_led.b_id, GPIOC, GPIO_PIN_9, IOIF_GPIO_Mode_Output);
    
    IOIF_GPIO_Initialize_t rgb_config = {
        .mode = IOIF_GPIO_Mode_Output,
        .pull = IOIF_GPIO_Floating,
        .init_state = false,
    };
    IOIF_GPIO_REINITIALIZE(s_status_led.r_id, &rgb_config);
    IOIF_GPIO_REINITIALIZE(s_status_led.g_id, &rgb_config);
    IOIF_GPIO_REINITIALIZE(s_status_led.b_id, &rgb_config);
    
    /* Function Buttons (PB4, PB5) - Input with Pull-up */
    IOIF_GPIO_INITIALIZE(s_buttons[0].gpio_id, GPIOB, GPIO_PIN_4, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_buttons[1].gpio_id, GPIOB, GPIO_PIN_5, IOIF_GPIO_Mode_Input);
    
    IOIF_GPIO_Initialize_t btn_config = {
        .mode = IOIF_GPIO_Mode_Input,
        .pull = IOIF_GPIO_PullUp,  /* Active Low (누르면 0) */
        .init_state = false,
    };
    IOIF_GPIO_REINITIALIZE(s_buttons[0].gpio_id, &btn_config);
    IOIF_GPIO_REINITIALIZE(s_buttons[1].gpio_id, &btn_config);
    
    /* ===== 상태 초기화 ===== */
    
    /* Status LED */
    s_status_led.current_state = (IO_LinkState_t)-1;  /* 강제 업데이트 */
    IO_SetLinkState(IO_LINK_STATE_BOOT_UP);
    
    /* Buttons */
    for (int i = 0; i < 2; i++) {
        s_buttons[i].state = BTN_STATE_IDLE;
        s_buttons[i].last_event = IO_BTN_NONE;
    }
    
    /* Power LED 켜기 (초기화 완료) */
    IOIF_GPIO_SET(s_power_led_id);
}

void IO_Manager_Update(uint32_t timestamp_ms)
{
    /* 1. Status LED 업데이트 */
    _UpdateStatusLed(timestamp_ms);
    
    /* 2. Button 업데이트 */
    for (int i = 0; i < 2; i++) {
        _UpdateButton(&s_buttons[i], timestamp_ms);
    }
}

/* ===== Status LED API ===== */
void IO_SetLinkState(IO_LinkState_t state)
{
    if (s_status_led.current_state == state) {
        return;  /* 상태 변경 없음 */
    }
    
    s_status_led.current_state = state;
    
    /* CiA 303-3 상태 → 색상/패턴 매핑 */
    switch (state) {
        case IO_LINK_STATE_BOOT_UP:
            s_status_led.mode = IO_LED_SOLID;
            s_status_led.color_mask = 0x07;  /* White (R+G+B) */
            break;

        case IO_LINK_STATE_DISCONNECTED:
            s_status_led.mode = IO_LED_SOLID;
            s_status_led.color_mask = 0x01;  /* Red */
            break;

        case IO_LINK_STATE_PRE_OPERATIONAL:
            s_status_led.mode = IO_LED_SINGLE_FLASH;
            s_status_led.color_mask = 0x02;  /* Green */
            s_status_led.phase_durations = s_single_flash_phases;
            s_status_led.phase_count = 2;
            break;

        case IO_LINK_STATE_OPERATIONAL:
            s_status_led.mode = IO_LED_SOLID;
            s_status_led.color_mask = 0x02;  /* Green */
            break;

        case IO_LINK_STATE_HEARTBEAT_LOST:
            s_status_led.mode = IO_LED_DOUBLE_FLASH;
            s_status_led.color_mask = 0x01;  /* Red */
            s_status_led.phase_durations = s_double_flash_phases;
            s_status_led.phase_count = 4;
            break;

        case IO_LINK_STATE_ERROR:
            s_status_led.mode = IO_LED_BLINK;
            s_status_led.period_ms = 200;    /* 200ms Blinking (2.5Hz) */
            s_status_led.color_mask = 0x01;  /* Red */
            break;

        default:
            /* 정의되지 않은 상태 — 빠른 적색 깜박임으로 경고 */
            s_status_led.mode = IO_LED_BLINK;
            s_status_led.period_ms = 100;
            s_status_led.color_mask = 0x01;  /* Red */
            break;
    }

    /* Phase/Blink 상태 초기화 */
    s_status_led.phase = 0;
    s_status_led.last_toggle_time = 0;
    s_status_led.blink_state = true;

    /* 즉시 적용 (ON 상태로 시작) */
    _ApplyRgbColor(s_status_led.color_mask);
}

/* ===== Function Button API ===== */

bool IO_GetButtonState(uint8_t btn_index)
{
    if (btn_index < 1 || btn_index > 2) return false;
    
    /* Active Low (누르면 0) */
    return (IOIF_GPIO_READ(s_buttons[btn_index - 1].gpio_id) == GPIO_PIN_RESET);
}

IO_ButtonEvent_t IO_GetButtonEvent(uint8_t btn_index)
{
    if (btn_index < 1 || btn_index > 2) return IO_BTN_NONE;
    
    /* Read & Clear */
    IO_ButtonEvent_t evt = s_buttons[btn_index - 1].last_event;
    s_buttons[btn_index - 1].last_event = IO_BTN_NONE;
    return evt;
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS
 *-----------------------------------------------------------
 */

static void _UpdateButton(ButtonCtx_t* btn, uint32_t now)
{
    /* Active Low (누르면 0) */
    bool is_pressed = (IOIF_GPIO_READ(btn->gpio_id) == GPIO_PIN_RESET);
    
    switch (btn->state) {
        case BTN_STATE_IDLE:
            if (is_pressed) {
                btn->start_time = now;
                btn->state = BTN_STATE_DEBOUNCE;
            }
            break;
            
        case BTN_STATE_DEBOUNCE:
            if (is_pressed) {
                if (now - btn->start_time >= BUTTON_DEBOUNCE_MS) {
                    btn->state = BTN_STATE_PRESSED;
                    btn->last_event = IO_BTN_PRESSED;
                    btn->start_time = now;  /* Long Press 타이머 시작 */
                }
            } else {
                btn->state = BTN_STATE_IDLE;  /* 노이즈 */
            }
            break;
            
        case BTN_STATE_PRESSED:
            if (!is_pressed) {
                /* 짧게 누름 (Click) */
                btn->state = BTN_STATE_IDLE;
                btn->last_event = IO_BTN_CLICK;
            } else {
                /* Long Press 체크 */
                if (now - btn->start_time >= BUTTON_LONG_PRESS_MS) {
                    btn->state = BTN_STATE_LONG_HOLD;
                    btn->last_event = IO_BTN_LONG_PRESS;
                }
            }
            break;
            
        case BTN_STATE_LONG_HOLD:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
                btn->last_event = IO_BTN_RELEASED;
            }
            break;
    }
}

static void _UpdateStatusLed(uint32_t now)
{
    switch (s_status_led.mode) {
        case IO_LED_OFF:
        case IO_LED_SOLID:
            /* SetLinkState()에서 이미 적용됨 */
            break;

        case IO_LED_BLINK:       /* 200ms ON/OFF */
        case IO_LED_FLICKERING:  /* 50ms ON/OFF */
        {
            uint32_t half_period = (s_status_led.mode == IO_LED_FLICKERING) ? 50 : s_status_led.period_ms;
            if (now - s_status_led.last_toggle_time >= half_period) {
                s_status_led.last_toggle_time = now;
                s_status_led.blink_state = !s_status_led.blink_state;
                _ApplyRgbColor(s_status_led.blink_state ? s_status_led.color_mask : 0);
            }
            break;
        }

        case IO_LED_SINGLE_FLASH:
        case IO_LED_DOUBLE_FLASH:
        case IO_LED_TRIPLE_FLASH:
        {
            /* Phase-based: phase 짝수=ON, 홀수=OFF */
            uint16_t duration = s_status_led.phase_durations[s_status_led.phase];
            if (now - s_status_led.last_toggle_time >= duration) {
                s_status_led.last_toggle_time = now;
                s_status_led.phase = (s_status_led.phase + 1) % s_status_led.phase_count;
                bool is_on = (s_status_led.phase % 2 == 0);
                _ApplyRgbColor(is_on ? s_status_led.color_mask : 0);
            }
            break;
        }
    }
}

static void _ApplyRgbColor(uint8_t mask)
{
    /* R(bit0), G(bit1), B(bit2) */
    (mask & 0x01) ? IOIF_GPIO_SET(s_status_led.r_id) : IOIF_GPIO_RESET(s_status_led.r_id);
    (mask & 0x02) ? IOIF_GPIO_SET(s_status_led.g_id) : IOIF_GPIO_RESET(s_status_led.g_id);
    (mask & 0x04) ? IOIF_GPIO_SET(s_status_led.b_id) : IOIF_GPIO_RESET(s_status_led.b_id);
}

