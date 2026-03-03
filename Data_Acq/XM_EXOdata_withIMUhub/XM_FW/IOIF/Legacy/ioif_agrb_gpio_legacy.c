#include "ioif_agrb_gpio.h"

#if !defined(AGRB_IOIF_GPIO_DISABLE)

//#define __USE_DEBRUIJN_SEQUENCE__

#define __IOIF_GPIO_EXTI_MAX_PIN_NUMBER__   (16)

typedef struct 
{
    GPIO_TypeDef* port;
    uint16_t pin;
    IOIF_GPIO_Mode_e mode;

    IOIF_GPIO_Initialize_t config;

} IOIF_GPIO_Instance_t;

static uint32_t _bit_to_index32(uint32_t input);

static IOIF_GPIO_Callback_t _gpio_interrupt_callbacks[__IOIF_GPIO_EXTI_MAX_PIN_NUMBER__] = {0};
static bool _gpio_interrupt_initialized[__IOIF_GPIO_EXTI_MAX_PIN_NUMBER__] = {0};

static IOIF_GPIO_Instance_t _gpio_instances[IOIF_GPIO_MAX_INSTANCES];
static uint32_t _gpio_instances_count = 0;

AGRBStatusDef ioif_gpio_assign_instance(IOIF_GPIOx_t* id, GPIO_TypeDef* port, uint16_t pin, IOIF_GPIO_Mode_e mode)
{
    if (id == NULL || port == NULL) return AGRBStatus_PARAM_ERROR;

    *id = IOIF_GPIO_ID_NOT_ALLOCATED;

    if (_gpio_instances_count >= IOIF_GPIO_MAX_INSTANCES) {
        return AGRBStatus_ERROR; // No available instance slots
    }

    for (IOIF_GPIOx_t i = 0; i < _gpio_instances_count; i++) {
        if (_gpio_instances[i].port == port && _gpio_instances[i].pin == pin) {
            return AGRBStatus_BUSY; // Already assigned 
        }
    }

    //Assign new instance
    IOIF_GPIO_Instance_t* instance = &_gpio_instances[_gpio_instances_count];
    memset(instance, 0, sizeof(IOIF_GPIO_Instance_t));
    instance->port = port;
    instance->pin = pin;
    instance->mode = mode;

    *id = _gpio_instances_count++;

    return AGRBStatus_OK;
}

GPIO_InitTypeDef _debug_gpio_reinit;
AGRBStatusDef ioif_gpio_reinitialize(IOIF_GPIOx_t id, IOIF_GPIO_Initialize_t* config)
{
    if (id >= _gpio_instances_count) return AGRBStatus_ERROR; // Invalid instance ID
    if ( config == NULL ) return AGRBStatus_PARAM_ERROR;

    IOIF_GPIO_Instance_t* instance = &_gpio_instances[id];

    memcpy(&instance->config, config, sizeof(IOIF_GPIO_Initialize_t));

    if ( instance->port == NULL ) return AGRBStatus_ERROR; // Not initialized properly

    HAL_GPIO_DeInit(instance->port, instance->pin);
    //if already callback has registered, remove that
    if ( instance->mode == IOIF_GPIO_Mode_EXTI ) {
        uint32_t pin_index = _bit_to_index32(instance->pin);
        if ( _gpio_interrupt_initialized[pin_index] ) {
            _gpio_interrupt_callbacks[pin_index] = NULL;
            _gpio_interrupt_initialized[pin_index] = false;
        }
    }

    GPIO_InitTypeDef _init = {0}; 
    
    if ( config->mode == IOIF_GPIO_Mode_Input || config->mode == IOIF_GPIO_Mode_Output ) {
        //Open Drain 설정은 지원하지 않는다.
        _init.Mode = (config->mode == IOIF_GPIO_Mode_Output) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
        _init.Pull = (config->pull == IOIF_GPIO_PullUp) ? GPIO_PULLUP : ( (config->pull == IOIF_GPIO_PullDown) ? GPIO_PULLDOWN : GPIO_NOPULL );
        _init.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //최대 속도로 설정
        _init.Alternate = 0; //Alternate function not used
        _init.Pin = instance->pin;

        memcpy(&_debug_gpio_reinit, &_init, sizeof(GPIO_InitTypeDef));        
    } 
    else if ( config->mode == IOIF_GPIO_Mode_EXTI )
    {
        if ( instance->pin > __IOIF_GPIO_EXTI_MAX_PIN_NUMBER__ ) return AGRBStatus_ERROR; // EXTI pin number exceeded
        if ( config->interrupt.callback == NULL ) return AGRBStatus_PARAM_ERROR; // Callback function must be provided

        uint32_t pin_index = _bit_to_index32(instance->pin);

        //Register callback
        {
            if ( _gpio_interrupt_initialized[pin_index] ) return AGRBStatus_BUSY; // Already initialized
            _gpio_interrupt_callbacks[pin_index] = config->interrupt.callback;
            _gpio_interrupt_initialized[pin_index] = true;
        }

        _init.Mode = (config->interrupt.detection_mode == IOIF_GPIO_InterruptDetectionMode_Rising) ? GPIO_MODE_IT_RISING :
                     (config->interrupt.detection_mode == IOIF_GPIO_InterruptDetectionMode_Falling) ? GPIO_MODE_IT_FALLING :
                     (config->interrupt.detection_mode == IOIF_GPIO_InterruptDetectionMode_RisingFalling) ? GPIO_MODE_IT_RISING_FALLING :
                     0;
        _init.Pull = (config->pull == IOIF_GPIO_PullUp) ? GPIO_PULLUP : ( (config->pull == IOIF_GPIO_PullDown) ? GPIO_PULLDOWN : GPIO_NOPULL );
        _init.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //최대 속도로 설정
        _init.Alternate = 0; //Alternate function not used
        _init.Pin = instance->pin; 

        if ( _init.Mode == 0 ) return AGRBStatus_PARAM_ERROR; // Invalid EXTI mode
    }

    HAL_GPIO_Init(instance->port, &_init);

    return AGRBStatus_OK;
}



AGRBStatusDef ioif_gpio_set(IOIF_GPIOx_t id)
{
    if (id >= _gpio_instances_count) return AGRBStatus_ERROR; // Invalid instance ID

    IOIF_GPIO_Instance_t* instance = &_gpio_instances[id];

    if (instance->mode != IOIF_GPIO_Mode_Output) {
        return AGRBStatus_NOT_ALLOWED; // Not an output pin
    }

    HAL_GPIO_WritePin(instance->port, instance->pin, GPIO_PIN_SET);

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_gpio_reset(IOIF_GPIOx_t id)
{
    if (id >= _gpio_instances_count) return AGRBStatus_ERROR; // Invalid instance ID

    IOIF_GPIO_Instance_t* instance = &_gpio_instances[id];

    if (instance->mode != IOIF_GPIO_Mode_Output) {
        return AGRBStatus_NOT_ALLOWED; // Not allowed to reset if not output mode
    }

    HAL_GPIO_WritePin(instance->port, instance->pin, GPIO_PIN_RESET);

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_gpio_toggle(IOIF_GPIOx_t id)
{
    if (id >= _gpio_instances_count) return AGRBStatus_ERROR; // Invalid instance ID

    IOIF_GPIO_Instance_t* instance = &_gpio_instances[id];

    if (instance->mode != IOIF_GPIO_Mode_Output) {
        return AGRBStatus_NOT_ALLOWED; // Not allowed to toggle if not output mode
    }

    HAL_GPIO_TogglePin(instance->port, instance->pin);
    
    return AGRBStatus_OK;
}

AGRBStatusDef ioif_gpio_get_state(IOIF_GPIOx_t id, bool* state)
{
    if (id >= _gpio_instances_count) return AGRBStatus_ERROR; // Invalid instance ID
    if ( state == NULL ) return AGRBStatus_PARAM_ERROR;

    IOIF_GPIO_Instance_t* instance = &_gpio_instances[id];

    GPIO_PinState pin_state = HAL_GPIO_ReadPin(instance->port, instance->pin);
    *state = (pin_state == GPIO_PIN_SET ? true : false);

    return AGRBStatus_OK;
}

static inline uint32_t _bit_to_index32(uint32_t input)
{
#if defined(__USE_DEBRUIJN_SEQUENCE__)

    static const int debruijn32_table[32] = {
        0,  1,  28, 2,  29, 14, 24, 3,  
        30, 22, 20, 15, 25, 17, 4,  8, 
        31, 27, 13, 23, 21, 19, 16, 7,  
        26, 12, 18, 6,  11, 5,  10, 9
    };

    const uint32_t debruijn32 = 0x077CB531U;
    return debruijn32_table[((input & -input) * debruijn32) >> 27];
#else
    return __builtin_ctz(input);
#endif
}

/********* CALLBACK_FUNCTION ************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Find the index of the pin
    uint32_t pin_index = _bit_to_index32(GPIO_Pin);
    if ( pin_index >= __IOIF_GPIO_EXTI_MAX_PIN_NUMBER__ ) return; // Invalid pin index

    // Call the registered callback if exists
    if ( _gpio_interrupt_initialized[pin_index] && _gpio_interrupt_callbacks[pin_index] != NULL ) return;

    _gpio_interrupt_callbacks[pin_index]();
}


#endif // AGRB_IOIF_GPIO_DISABLE