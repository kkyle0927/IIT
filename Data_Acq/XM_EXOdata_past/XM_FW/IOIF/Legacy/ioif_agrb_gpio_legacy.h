#include "ioif_agrb_defs.h"

#if !defined(AGRB_IOIF_GPIO_DISABLE)

#ifndef _IOIF_AGRB_GPIO_EX_H
#define _IOIF_AGRB_GPIO_EX_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_gpio.h"


typedef uint32_t IOIF_GPIOx_t;

#define IOIF_GPIO_MAX_INSTANCES     (32) //Max 32
#define IOIF_GPIO_ID_NOT_ALLOCATED  (0xFFFFFFFF)
#define IOIF_GPIO_NOT_INITIALIZED (0xFFFFFFFF)

#define IOIF_GPIO_INITIALIZE(id, port, pin, mode)               ioif_gpio_assign_instance(&(id), (port), (pin), (mode))
#define IOIF_GPIO_REINITIALIZE(id, config)                      ioif_gpio_reinitialize((id), (config))
#define IOIF_GPIO_SET(id)                                       ioif_gpio_set((id))
#define IOIF_GPIO_RESET(id)                                     ioif_gpio_reset((id))
#define IOIF_GPIO_TOGGLE(id)                                    ioif_gpio_toggle((id))
#define IOIF_GPIO_GET_STATE(id, state)                          ioif_gpio_get_state((id), (state))

//Callback function prototype
typedef void (*IOIF_GPIO_Callback_t)(void);

typedef enum
{
    IOIF_GPIO_InterruptDetectionMode_Rising,
    IOIF_GPIO_InterruptDetectionMode_Falling,
    IOIF_GPIO_InterruptDetectionMode_RisingFalling,

} IOIF_GPIO_InterruptDetectionMode_e;

typedef enum
{
    IOIF_GPIO_Mode_Input,
    IOIF_GPIO_Mode_Output,
    IOIF_GPIO_Mode_Alternate,
    IOIF_GPIO_Mode_EXTI,

} IOIF_GPIO_Mode_e;

typedef enum
{
    IOIF_GPIO_Floating = 0x00U,   /*!< Input Floating Mode                   */
    IOIF_GPIO_PullUp = 0x01U,     /*!< Pull-up activation                    */
    IOIF_GPIO_PullDown = 0x02U,   /*!< Pull-down activation                  */

} IOIF_GPIO_Pull_e;

// 기존 STM32 IOC와 충돌하는 설정이 존재하는 경우, 이를 덮어쓰는 동작을 수행하는 구조체
typedef struct
{
    IOIF_GPIO_Mode_e mode;    
    IOIF_GPIO_Pull_e pull;
    bool init_state; //초기 상태 (Output 모드일 때만 의미 있음)

    struct {
        IOIF_GPIO_InterruptDetectionMode_e detection_mode;
        IOIF_GPIO_Callback_t callback;
    } interrupt;

} IOIF_GPIO_Initialize_t;


AGRBStatusDef ioif_gpio_assign_instance(IOIF_GPIOx_t* id, GPIO_TypeDef* port, uint16_t pin, IOIF_GPIO_Mode_e mode);

AGRBStatusDef ioif_gpio_reinitialize(IOIF_GPIOx_t id, IOIF_GPIO_Initialize_t* config);

AGRBStatusDef ioif_gpio_set(IOIF_GPIOx_t id);
AGRBStatusDef ioif_gpio_reset(IOIF_GPIOx_t id);
AGRBStatusDef ioif_gpio_toggle(IOIF_GPIOx_t id);

AGRBStatusDef ioif_gpio_get_state(IOIF_GPIOx_t id, bool* state); 

#endif


#endif // AGRB_IOIF_GPIO_DISABLE