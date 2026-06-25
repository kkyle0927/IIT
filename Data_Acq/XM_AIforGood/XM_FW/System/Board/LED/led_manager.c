/**
 ******************************************************************************
 * @file    led_manager.c
 * @author  HyundoKim
 * @brief   [System Layer] 온보드 LED + PCA9957 채널 LED 통합 관리 구현부
 * @details
 * - CM-XM Link Status LED: RGB GPIO On/Off (CiA 303-3 패턴)
 * - User Function LED: Single GPIO On/Off (Blink/Heartbeat/OneShot)
 * - Channel LED: PCA9957 24ch RGB PWM (Device State → 패턴 → PWM)
 * @version 2.0.0
 * @date    2026-03-02
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "led_manager.h"
#include "pca9957.h"
#include "ioif_agrb_gpio.h"
#include <stdbool.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/** PCA9957 총 채널 수 */
#define PCA9957_TOTAL_CHANNELS  24

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/** User LED 컨텍스트 */
typedef struct {
    IOIF_GPIOx_t id;
    LedMode_t mode;
    uint32_t period;
    uint32_t last_toggle;
    bool is_on;
    uint32_t start_time;
} LedContext_t;

/** 채널 LED 컨텍스트 (PCA9957 RGB) */
typedef struct {
    LedMode_t green_mode;
    LedMode_t red_mode;
    LedMode_t blue_mode;    /**< Blue LED 패턴 (MSC 등, 기본 OFF) */
    bool rgb_override;      /**< true일 때 패턴 무시, 직접 RGB 값 사용 */
    uint8_t override_r;
    uint8_t override_g;
    uint8_t override_b;
    uint8_t current_r;      /**< 현재 출력 R PWM */
    uint8_t current_g;      /**< 현재 출력 G PWM */
    uint8_t current_b;      /**< 현재 출력 B PWM */
} ChannelLedContext_t;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* --- Link Status LED (RGB GPIO) --- */
static struct {
    IOIF_GPIOx_t r_id;
    IOIF_GPIOx_t g_id;
    IOIF_GPIOx_t b_id;
    LinkState_t current_state;
    LedMode_t mode;
    uint32_t period;
    uint32_t last_toggle;
    bool blink_state;
    uint8_t color_mask;     /**< R(bit0)|G(bit1)|B(bit2) */
} s_link_led;

/* --- User LEDs (3개, GPIO) --- */
static LedContext_t s_user_leds[3];

/* --- Channel LEDs (7개, PCA9957 RGB) --- */
static ChannelLedContext_t s_ch_leds[CH_LED_COUNT];
static bool s_ch_leds_initialized = false;
/* PCA9957 channel LED suspend flag — SPI LED stimulus (write 토글) 활성 동안
 * led_manager 가 PCA9957_UpdateAll() 호출을 skip 하도록 게이트. internal
 * s_pwm_buffer 는 평시처럼 갱신되어 resume 시점에 마지막 상태 SPI write 가능.
 * stimulus.c: XM_Stimulus_SetEnabled(XM_STIM_SPI_LED, on) 에서 자동 호출.
 */
static bool s_channel_suspended = false;

/* --- PCA9957 Batch Update Buffer --- */
static uint8_t s_pwm_buffer[PCA9957_TOTAL_CHANNELS];
static bool s_pwm_dirty = false;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _ApplyRgbColor(uint8_t mask);
static void _UpdateSingleLed(LedContext_t* led, uint32_t now);
static void _UpdateLinkLed(uint32_t now);
static void _UpdateChannelLeds(uint32_t now);
static bool _CalcPatternOnOff(LedMode_t mode, uint32_t now);

/* Device state → pattern lookup */
static void _ApplyCanDevicePattern(ChannelLedId_t ch, ChannelDevState_t state);
static void _ApplyGrfSensorPattern(ChannelLedId_t ch, GrfSensorState_t state);
static void _ApplyUsbPattern(UsbLedState_t state);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* ==================== Link Status LED ==================== */

void LedManager_InitLinkStatusLeds(IOIF_GPIOx_t r_id, IOIF_GPIOx_t g_id, IOIF_GPIOx_t b_id)
{
    s_link_led.r_id = r_id;
    s_link_led.g_id = g_id;
    s_link_led.b_id = b_id;
    s_link_led.current_state = (LinkState_t)-1;

    LedManager_SetLinkState(LINK_STATE_INITIALISING);
}

void LedManager_SetLinkState(LinkState_t state)
{
    if (s_link_led.current_state == state) return;
    s_link_led.current_state = state;

    switch (state) {
        case LINK_STATE_INITIALISING:
            s_link_led.mode = LED_MODE_SOLID;
            s_link_led.color_mask = 0x07; /* White (R+G+B) */
            break;
        case LINK_STATE_PRE_OPERATIONAL:
            s_link_led.mode = LED_MODE_BLINK;
            s_link_led.period = 500;
            s_link_led.color_mask = 0x02; /* Green */
            break;
        case LINK_STATE_OPERATIONAL:
            s_link_led.mode = LED_MODE_SOLID;
            s_link_led.color_mask = 0x02; /* Green (04 doc: Blue→Green) */
            break;
        case LINK_STATE_STOPPED:
            s_link_led.mode = LED_MODE_SOLID;
            s_link_led.color_mask = 0x01; /* Red */
            break;
        case LINK_STATE_DETECTING:
            s_link_led.mode = LED_MODE_FLICKERING;
            s_link_led.color_mask = 0x02; /* Green Flickering */
            break;
        case LINK_STATE_HEARTBEAT_LOST:
            s_link_led.mode = LED_MODE_DOUBLE_FLASH;
            s_link_led.color_mask = 0x01; /* Red Double Flash */
            break;
        case LINK_STATE_ERROR:
            s_link_led.mode = LED_MODE_BLINK;
            s_link_led.period = 200;
            s_link_led.color_mask = 0x01; /* Red Blinking */
            break;
        case LINK_STATE_FW_UPDATE:
            s_link_led.mode = LED_MODE_SOLID;
            s_link_led.color_mask = 0x04; /* Blue */
            break;
        default:
            s_link_led.mode = LED_MODE_OFF;
            s_link_led.color_mask = 0x00;
            break;
    }

    s_link_led.last_toggle = 0;
    s_link_led.blink_state = true;

    if (s_link_led.mode == LED_MODE_SOLID) {
        _ApplyRgbColor(s_link_led.color_mask);
    } else if (s_link_led.mode == LED_MODE_OFF) {
        _ApplyRgbColor(0);
    } else {
        _ApplyRgbColor(s_link_led.color_mask);
    }
}

/* ==================== User Function LED ==================== */

void LedManager_InitUserLeds(IOIF_GPIOx_t led1_id, IOIF_GPIOx_t led2_id, IOIF_GPIOx_t led3_id)
{
    s_user_leds[0].id = led1_id;
    s_user_leds[1].id = led2_id;
    s_user_leds[2].id = led3_id;

    for (int i = 0; i < 3; i++) {
        s_user_leds[i].mode = LED_MODE_OFF;
        IOIF_GPIO_RESET(s_user_leds[i].id);
    }
}

void LedManager_SetUserLedMode(uint8_t led_index, LedMode_t mode, uint32_t period_ms)
{
    if (led_index < 1 || led_index > 3) return;
    int idx = led_index - 1;

    s_user_leds[idx].mode = mode;
    s_user_leds[idx].period = period_ms;
    s_user_leds[idx].start_time = 0;

    if (mode == LED_MODE_SOLID) {
        IOIF_GPIO_SET(s_user_leds[idx].id);
        s_user_leds[idx].is_on = true;
    } else if (mode == LED_MODE_OFF) {
        IOIF_GPIO_RESET(s_user_leds[idx].id);
        s_user_leds[idx].is_on = false;
    }
}

/* ==================== Channel LED (PCA9957) ==================== */

void LedManager_InitChannelLeds(void)
{
    memset(s_ch_leds, 0, sizeof(s_ch_leds));
    memset(s_pwm_buffer, 0, sizeof(s_pwm_buffer));
    s_pwm_dirty = true;
    s_ch_leds_initialized = true;

    /* 초기 상태: 모든 채널 Off → PCA9957에 반영 */
    if (!s_channel_suspended) {
        PCA9957_UpdateAll(s_pwm_buffer);
    }
    PCA9957_Enable();
}

void LedManager_SetChannelSuspend(bool suspend)
{
    /* SPI LED stimulus (write 토글) 활성 시 평시 led_manager 가 SPI5 bus 점유
     * 시도를 멈추도록 게이트. resume 시점에 s_pwm_dirty=true 강제로 다음
     * LedManager_Update() 가 자동 reflush → 평시 status LED 패턴 복원.
     */
    if (s_channel_suspended == suspend) return;
    s_channel_suspended = suspend;
    if (!suspend && s_ch_leds_initialized) {
        s_pwm_dirty = true;
    }
}

bool LedManager_IsChannelSuspended(void)
{
    return s_channel_suspended;
}

void LedManager_SetChannelPattern(ChannelLedId_t ch, LedMode_t green, LedMode_t red)
{
    if (ch >= CH_LED_COUNT) return;
    s_ch_leds[ch].green_mode = green;
    s_ch_leds[ch].red_mode = red;
    s_ch_leds[ch].blue_mode = LED_MODE_OFF;  /* Blue 초기화 (하위호환) */
    s_ch_leds[ch].rgb_override = false;
}

void LedManager_SetChannelFullPattern(ChannelLedId_t ch,
                                       LedMode_t green,
                                       LedMode_t red,
                                       LedMode_t blue)
{
    if (ch >= CH_LED_COUNT) return;
    s_ch_leds[ch].green_mode = green;
    s_ch_leds[ch].red_mode = red;
    s_ch_leds[ch].blue_mode = blue;
    s_ch_leds[ch].rgb_override = false;
}

void LedManager_SetChannelRGB(ChannelLedId_t ch, uint8_t r, uint8_t g, uint8_t b)
{
    if (ch >= CH_LED_COUNT) return;
    s_ch_leds[ch].rgb_override = true;
    s_ch_leds[ch].override_r = r;
    s_ch_leds[ch].override_g = g;
    s_ch_leds[ch].override_b = b;
}

/* ==================== Device State Setters ==================== */

void LedManager_SetCanDeviceState(ChannelLedId_t ch, ChannelDevState_t state)
{
    if (ch >= CH_LED_COUNT) return;
    _ApplyCanDevicePattern(ch, state);
}

void LedManager_SetGrfSensorState(ChannelLedId_t ch, GrfSensorState_t state)
{
    if (ch >= CH_LED_COUNT) return;
    _ApplyGrfSensorPattern(ch, state);
}

void LedManager_SetUsbLedState(UsbLedState_t state)
{
    _ApplyUsbPattern(state);
}

/* ==================== Tick Update ==================== */

void LedManager_Update(uint32_t timestamp_ms)
{
    /* 1. Link Status LED */
    _UpdateLinkLed(timestamp_ms);

    /* 2. User LEDs */
    for (int i = 0; i < 3; i++) {
        _UpdateSingleLed(&s_user_leds[i], timestamp_ms);
    }

    /* 3. Channel LEDs (PCA9957) */
    if (s_ch_leds_initialized) {
        _UpdateChannelLeds(timestamp_ms);
    }
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/* ===== RGB GPIO 제어 ===== */

static void _ApplyRgbColor(uint8_t mask)
{
    (mask & 0x01) ? IOIF_GPIO_SET(s_link_led.r_id) : IOIF_GPIO_RESET(s_link_led.r_id);
    (mask & 0x02) ? IOIF_GPIO_SET(s_link_led.g_id) : IOIF_GPIO_RESET(s_link_led.g_id);
    (mask & 0x04) ? IOIF_GPIO_SET(s_link_led.b_id) : IOIF_GPIO_RESET(s_link_led.b_id);
}

/* ===== CiA 303-3 패턴 On/Off 계산 ===== */

/**
 * @brief 현재 시간(now)에 대해 해당 패턴이 ON인지 OFF인지 계산
 * @return true=ON, false=OFF
 */
static bool _CalcPatternOnOff(LedMode_t mode, uint32_t now)
{
    uint32_t phase;
    switch (mode) {
        case LED_MODE_OFF:
            return false;

        case LED_MODE_SOLID:
            return true;

        case LED_MODE_BLINK:
            /* 500ms ON / 500ms OFF (1s 주기) */
            return ((now % 1000) < 500);

        case LED_MODE_HEARTBEAT:
            /* 두근-두근: 0~100 ON, 200~300 ON, 나머지 OFF */
            phase = (now % 1000);
            return (phase < 100) || (phase > 200 && phase < 300);

        case LED_MODE_ONE_SHOT:
            return false; /* Channel LED에서는 사용 안함 */

        case LED_MODE_FLICKERING:
            /* 50ms ON / 50ms OFF (10Hz) */
            return ((now % 100) < 50);

        case LED_MODE_SINGLE_FLASH:
            /* 200ms ON / 1000ms OFF → 1200ms 주기 */
            return ((now % 1200) < 200);

        case LED_MODE_DOUBLE_FLASH:
            /* 200ms ON / 200ms OFF / 200ms ON / 1000ms OFF → 1600ms 주기 */
            phase = (now % 1600);
            return (phase < 200) || (phase >= 400 && phase < 600);

        case LED_MODE_TRIPLE_FLASH:
            /* 200ms ON / 200ms OFF / 200ms ON / 200ms OFF / 200ms ON / 1000ms OFF → 2000ms 주기 */
            phase = (now % 2000);
            return (phase < 200) || (phase >= 400 && phase < 600) || (phase >= 800 && phase < 1000);

        default:
            return false;
    }
}

/* ===== Link Status LED Update ===== */

static void _UpdateLinkLed(uint32_t now)
{
    if (s_link_led.mode == LED_MODE_SOLID || s_link_led.mode == LED_MODE_OFF) {
        return; /* 이미 SetLinkState()에서 적용됨 */
    }

    bool should_on = _CalcPatternOnOff(s_link_led.mode, now);

    if (should_on != s_link_led.blink_state) {
        s_link_led.blink_state = should_on;
        _ApplyRgbColor(should_on ? s_link_led.color_mask : 0);
    }
}

/* ===== User Single LED Update ===== */

static void _UpdateSingleLed(LedContext_t* led, uint32_t now)
{
    if (led->start_time == 0) led->start_time = now;

    switch (led->mode) {
        case LED_MODE_BLINK:
            if (now - led->last_toggle >= led->period) {
                led->last_toggle = now;
                led->is_on = !led->is_on;
                (led->is_on) ? IOIF_GPIO_SET(led->id) : IOIF_GPIO_RESET(led->id);
            }
            break;

        case LED_MODE_HEARTBEAT:
            {
                uint32_t phase = (now % 1000);
                bool should_be_on = (phase < 100) || (phase > 200 && phase < 300);
                if (led->is_on != should_be_on) {
                    led->is_on = should_be_on;
                    (should_be_on) ? IOIF_GPIO_SET(led->id) : IOIF_GPIO_RESET(led->id);
                }
            }
            break;

        case LED_MODE_ONE_SHOT:
            if (now - led->start_time < led->period) {
                if (!led->is_on) {
                    IOIF_GPIO_SET(led->id);
                    led->is_on = true;
                }
            } else {
                if (led->is_on) {
                    IOIF_GPIO_RESET(led->id);
                    led->is_on = false;
                    led->mode = LED_MODE_OFF;
                }
            }
            break;

        case LED_MODE_FLICKERING:
        case LED_MODE_SINGLE_FLASH:
        case LED_MODE_DOUBLE_FLASH:
        case LED_MODE_TRIPLE_FLASH:
            {
                bool should_on = _CalcPatternOnOff(led->mode, now);
                if (led->is_on != should_on) {
                    led->is_on = should_on;
                    (should_on) ? IOIF_GPIO_SET(led->id) : IOIF_GPIO_RESET(led->id);
                }
            }
            break;

        default:
            break;
    }
}

/* ===== Channel LED Update (PCA9957 PWM) ===== */

static void _UpdateChannelLeds(uint32_t now)
{
    /* s_pwm_dirty 는 함수 시작 시 reset 하지 않는다 — 외부 (LedManager_SetChannelSuspend
     * resume) 가 set 한 dirty flag 를 보존해야 stimulus OFF 직후 reflush 가능.
     *
     * 이전 버그: 함수 시작 첫 줄에서 s_pwm_dirty=false 로 무조건 reset 했음.
     * SI Toggle LED ON 시 stimulus 가 PCA9957 PWM register 를 0xFF/0x00 으로 직접
     * 변경했지만 s_ch_leds 의 current_r/g/b cache 는 그대로. OFF 후 SetChannelSuspend
     * 가 s_pwm_dirty=true set → 다음 _UpdateChannelLeds 진입 시 즉시 false → 변경
     * 감지 비교에서 new == current → dirty 재 set 안 됨 → reflush 누락 → 평시
     * USB/CAN/GRF status LED 복원 안 됨 (사용자 보고 2026-05-13 23:xx).
     *
     * 수정 후: 변경 감지에서만 dirty=true (보강), 마지막 SPI write 성공 시에만
     * dirty=false. 외부 set 은 다음 _Update 호출까지 보존되어 강제 reflush 가능.
     */
    for (int i = 0; i < CH_LED_COUNT; i++) {
        ChannelLedContext_t* ch = &s_ch_leds[i];
        uint8_t new_r, new_g, new_b;

        if (ch->rgb_override) {
            new_r = ch->override_r;
            new_g = ch->override_g;
            new_b = ch->override_b;
        } else {
            /* 패턴 기반 R/G/B PWM 계산 */
            bool green_on = _CalcPatternOnOff(ch->green_mode, now);
            bool red_on   = _CalcPatternOnOff(ch->red_mode, now);
            bool blue_on  = _CalcPatternOnOff(ch->blue_mode, now);

            new_r = red_on   ? 255 : 0;
            new_g = green_on ? 255 : 0;
            new_b = blue_on  ? 255 : 0;
        }

        /* 변경 감지 */
        if (new_r != ch->current_r || new_g != ch->current_g || new_b != ch->current_b) {
            ch->current_r = new_r;
            ch->current_g = new_g;
            ch->current_b = new_b;

            /* PCA9957 그룹 슬롯에 기록 (group = ch, 3ch per group: R,G,B) */
            uint8_t base = (uint8_t)(i * 3);
            s_pwm_buffer[base + 0] = new_r;
            s_pwm_buffer[base + 1] = new_g;
            s_pwm_buffer[base + 2] = new_b;
            s_pwm_dirty = true;
        }
    }

    /* Dirty 시에만 SPI 전송. suspend 중이면 buffer 만 유지, SPI write skip. */
    if (s_pwm_dirty && !s_channel_suspended) {
        if (PCA9957_UpdateAll(s_pwm_buffer) == AGRBStatus_OK) {
            s_pwm_dirty = false;  /* SPI write 성공 후에만 clear */
        }
        /* SPI 실패 시 dirty 유지 → 다음 tick 재시도 */
    }
}

/* ===== Device State → Pattern Lookup Tables ===== */

/**
 * @brief CAN-FD PnP 디바이스 상태 → Green/Red 패턴 매핑
 */
static void _ApplyCanDevicePattern(ChannelLedId_t ch, ChannelDevState_t state)
{
    LedMode_t green = LED_MODE_OFF;
    LedMode_t red   = LED_MODE_OFF;

    switch (state) {
        case CH_DEV_NOT_CONNECTED:
            green = LED_MODE_OFF;
            red   = LED_MODE_OFF;
            break;
        case CH_DEV_DETECTING:
            green = LED_MODE_FLICKERING;
            red   = LED_MODE_OFF;
            break;
        case CH_DEV_PRE_OPERATIONAL:
            green = LED_MODE_SINGLE_FLASH;
            red   = LED_MODE_OFF;
            break;
        case CH_DEV_OPERATIONAL:
            green = LED_MODE_SOLID;
            red   = LED_MODE_OFF;
            break;
        case CH_DEV_HEARTBEAT_LOST:
            green = LED_MODE_OFF;
            red   = LED_MODE_DOUBLE_FLASH;
            break;
        case CH_DEV_DEVICE_ERROR:
            green = LED_MODE_OFF;
            red   = LED_MODE_SOLID;
            break;
        case CH_DEV_CAN_WARNING:
            /* Amber = Red + Green 동시 Blinking */
            green = LED_MODE_BLINK;
            red   = LED_MODE_BLINK;
            break;
        case CH_DEV_CAN_BUS_OFF:
            green = LED_MODE_OFF;
            red   = LED_MODE_BLINK;
            break;
        case CH_DEV_WRONG_DEVICE:
            green = LED_MODE_OFF;
            red   = LED_MODE_TRIPLE_FLASH;
            break;
    }

    LedManager_SetChannelPattern(ch, green, red);
}

/**
 * @brief GRF UART 센서 상태 → Green/Red 패턴 매핑
 */
static void _ApplyGrfSensorPattern(ChannelLedId_t ch, GrfSensorState_t state)
{
    LedMode_t green = LED_MODE_OFF;
    LedMode_t red   = LED_MODE_OFF;

    switch (state) {
        case GRF_NOT_CONNECTED:
            green = LED_MODE_OFF;
            red   = LED_MODE_OFF;
            break;
        case GRF_DETECTING:
            green = LED_MODE_FLICKERING;
            red   = LED_MODE_OFF;
            break;
        case GRF_DATA_ACTIVE:
            green = LED_MODE_SOLID;
            red   = LED_MODE_OFF;
            break;
        case GRF_DATA_TIMEOUT:
            /* Amber Single Flash */
            green = LED_MODE_SINGLE_FLASH;
            red   = LED_MODE_SINGLE_FLASH;
            break;
        case GRF_LOW_BATTERY:
            /* Amber Heartbeat */
            green = LED_MODE_HEARTBEAT;
            red   = LED_MODE_HEARTBEAT;
            break;
        case GRF_CRITICAL_BATTERY:
            green = LED_MODE_OFF;
            red   = LED_MODE_BLINK;
            break;
        case GRF_SENSOR_ERROR:
            green = LED_MODE_OFF;
            red   = LED_MODE_SOLID;
            break;
    }

    LedManager_SetChannelPattern(ch, green, red);
}

/**
 * @brief USB LED 상태 → 패턴 매핑
 * @details CDC=Green, MSC=Blue, Error=Red (04_LED_Indication_Design.md Section 4.4)
 */
static void _ApplyUsbPattern(UsbLedState_t state)
{
    switch (state) {
        case USB_LED_NOT_CONNECTED:
            LedManager_SetChannelPattern(CH_LED_USB, LED_MODE_OFF, LED_MODE_OFF);
            break;
        case USB_LED_CDC_CONNECTED:
            LedManager_SetChannelPattern(CH_LED_USB, LED_MODE_SOLID, LED_MODE_OFF);
            break;
        case USB_LED_CDC_DATA_ACTIVE:
            LedManager_SetChannelPattern(CH_LED_USB, LED_MODE_FLICKERING, LED_MODE_OFF);
            break;
        case USB_LED_MSC_READY:
            LedManager_SetChannelFullPattern(CH_LED_USB, LED_MODE_OFF, LED_MODE_OFF, LED_MODE_SOLID);
            break;
        case USB_LED_MSC_LOGGING:
            LedManager_SetChannelFullPattern(CH_LED_USB, LED_MODE_OFF, LED_MODE_OFF, LED_MODE_FLICKERING);
            break;
        case USB_LED_MSC_NEARLY_FULL:
            LedManager_SetChannelFullPattern(CH_LED_USB, LED_MODE_OFF, LED_MODE_SINGLE_FLASH, LED_MODE_SOLID);
            break;
        case USB_LED_ERROR:
            LedManager_SetChannelPattern(CH_LED_USB, LED_MODE_OFF, LED_MODE_SOLID);
            break;
    }
}
