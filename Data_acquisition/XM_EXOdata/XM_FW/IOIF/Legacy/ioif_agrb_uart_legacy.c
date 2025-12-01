#include "ioif_agrb_uart.h"
#if defined(AGRB_IOIF_UART_ENABLE)
#include <string.h>
//add cmsis-v2 for osThreadId_t
#include "cmsis_os2.h"


#define UART_RX_TASK_STACK_SIZE (2048) // Double Word 단위 4바이트
#define IOIF_UART_RX_BOUNCE_BUFFER_SIZE     (512)   //UART 수신용 바운스 버퍼 크기, 태스크에서 사용됨

#if ((IOIF_UART_RX_BOUNCE_BUFFER_SIZE * 2) > UART_RX_TASK_STACK_SIZE)
	#error "Buffer Size가 Stack 사이즈 대비 너무 큼"
#endif
 
#if defined(USE_FREERTOS_DMA)
__attribute__((section(IOIF_DMA_SECTION), aligned(32)))
static uint8_t _uart_tx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_TX_DMA_BUFFER_SIZE];
__attribute__((section(IOIF_DMA_SECTION), aligned(32)))
static uint8_t _uart_rx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_RX_DMA_BUFFER_SIZE];


#define IOIF_UART_ACQUIRE_TX_SEMAPHORE(instance) do { \
    if (instance->tx_semaphore != NULL) { \
        if (xSemaphoreTake(instance->tx_semaphore, pdMS_TO_TICKS(5000)) != pdTRUE) { \
            return AGRBStatus_TIMEOUT; \
        } \
    } else { \
        return AGRBStatus_ERROR; \
    } \
} while(0)

#define IOIF_UART_RELEASE_TX_SEMAPHORE(instance) do { \
    if (instance->tx_semaphore != NULL) { \
        xSemaphoreGive(instance->tx_semaphore); \
    } \
} while(0)

#define IOIF_UART_RELEASE_TX_SEMAPHORE_ISR(instance) do { \
    if (instance->tx_semaphore != NULL) { \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; \
        xSemaphoreGiveFromISR(instance->tx_semaphore, &xHigherPriorityTaskWoken); \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); \
    } \
} while(0)

#define IOIF_UART_WAIT_RX_COMPLETE(instance) do { \
    if (instance->rx_semaphore != NULL) { \
        if (xSemaphoreTake(instance->rx_semaphore, pdMS_TO_TICKS(5000)) != pdTRUE) { \
            IOIF_UART_RELEASE_TX_SEMAPHORE(instance); \
            return AGRBStatus_TIMEOUT; \
        } \
    } else { \
        IOIF_UART_RELEASE_TX_SEMAPHORE(instance); \
        return AGRBStatus_ERROR; \
    } \
} while(0)

#define IOIF_UART_SIGNAL_RX_COMPLETE(instance) do { \
    if (instance->rx_semaphore != NULL) { \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; \
        xSemaphoreGiveFromISR(instance->rx_semaphore, &xHigherPriorityTaskWoken); \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); \
    } \
} while(0)


#else
#define IOIF_UART_ACQUIRE_TX_SEMAPHORE(instance)
#define IOIF_UART_RELEASE_TX_SEMAPHORE(instance)
#define IOIF_UART_RELEASE_TX_SEMAPHORE_ISR(instance)

#define IOIF_UART_WAIT_RX_COMPLETE(instance)
#define IOIF_UART_SIGNAL_RX_COMPLETE(instance)


#endif



typedef struct {
    bool allocated;
    IOIF_UARTx_t id;

    UART_HandleTypeDef* huart;
    IOIF_UART_Config_t config;

    #if defined(USE_FREERTOS_DMA)
    SemaphoreHandle_t tx_semaphore; 
    SemaphoreHandle_t rx_semaphore;
    osThreadId_t rx_task_handle;
    struct{
        //uint32_t head;
        uint32_t tail;
    }ringbuffer;
    uint32_t error_count;  // Error counter for debugging
    uint8_t* rx_dma;
    #endif

} IOIF_UART_Instance_t;


static IOIF_UART_Instance_t _uart_instances[IOIF_UART_MAX_INSTANCES] = {0};
static uint32_t _uart_instance_count = 0;

// Helper function declarations
static inline AGRBStatusDef _convert_hal_status(HAL_StatusTypeDef hal_status);
static inline IOIF_UART_Instance_t* _get_instance(UART_HandleTypeDef* huart);
static uint32_t _get_baudrate_value(IOIF_UART_Baudrate_e baudrate);

#if defined(USE_FREERTOS_DMA)
static void _uart_rx_task(void* argument);
#endif

/////////////

static inline AGRBStatusDef _convert_hal_status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return AGRBStatus_OK;
        case HAL_ERROR:
            return AGRBStatus_ERROR;
        case HAL_BUSY:
            return AGRBStatus_BUSY;
        case HAL_TIMEOUT:
            return AGRBStatus_TIMEOUT;
        default:
            return AGRBStatus_ERROR;
    }
}

static inline IOIF_UART_Instance_t* _get_instance(UART_HandleTypeDef* huart)
{
    if (huart == NULL) return NULL;

    for (uint32_t i = 0; i < _uart_instance_count; i++) {
        if (_uart_instances[i].huart == huart && _uart_instances[i].allocated) {
            return &_uart_instances[i];
        }
    }

    return NULL; // Not found
}

static uint32_t _get_baudrate_value(IOIF_UART_Baudrate_e baudrate)
{
    switch (baudrate) {
        case IOIF_UART_Baudrate_9600:   return 9600;
        case IOIF_UART_Baudrate_19200:  return 19200;
        case IOIF_UART_Baudrate_38400:  return 38400;
        case IOIF_UART_Baudrate_57600:  return 57600;
        case IOIF_UART_Baudrate_115200: return 115200;
        case IOIF_UART_Baudrate_230400: return 230400;
        case IOIF_UART_Baudrate_460800: return 460800;
        case IOIF_UART_Baudrate_921600: return 921600;
        default: return 115200;
    }
}

AGRBStatusDef ioif_uart_update_rx_callback(IOIF_UARTx_t id, IOIF_UART_RxEventCallback_t callback)
{
    //ID 검사 하고
    if (id >= IOIF_UART_MAX_INSTANCES) return AGRBStatus_ERROR;
    if (!_uart_instances[id].allocated) return AGRBStatus_ERROR;
    // CALLBACK NULL 체크... 는 안해도 됨
    // RX_SEMA_ACQ()
    {
        _uart_instances[id].config.rx_event_callback = callback;
    }
    // RX_SEMA_REL()
}

static const osThreadAttr_t task_attributes = {
    .name = "uart_rx_task",
    .priority = IOIF_UART_TASK_PRIORITY,
    .stack_size = UART_RX_TASK_STACK_SIZE
};

AGRBStatusDef ioif_uart_assign_instance(IOIF_UARTx_t* id, UART_HandleTypeDef* huart, IOIF_UART_Config_t* config)
{
    // Early parameter validation
    if (id == NULL || huart == NULL || config == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }

    *id = IOIF_UART_ID_NOT_ALLOCATED;

    if (_uart_instance_count >= IOIF_UART_MAX_INSTANCES) {
        return AGRBStatus_INITIAL_FAILED; // No available instance slots
    }

    // Check if already assigned
    for (IOIF_UARTx_t i = 0; i < _uart_instance_count; i++) {
        if (_uart_instances[i].huart == huart && _uart_instances[i].allocated) {
            return AGRBStatus_BUSY; // UART handle already assigned
        }
    }

    IOIF_UART_Instance_t* instance = &_uart_instances[_uart_instance_count];

    memset(instance, 0, sizeof(IOIF_UART_Instance_t));

    // Assign the UART handle and config
    instance->huart = huart;
    // huart->Init // TODO: DMA_CIRCULAR 인지 체크
    memcpy(&(instance->config), config, sizeof(IOIF_UART_Config_t));

    instance->id = _uart_instance_count;

    #if defined(USE_FREERTOS_DMA)
    // Initialize ring buffer positions
    instance->ringbuffer.tail = 0;

    // Initialize error counter
    instance->error_count = 0;

    // Create semaphores
    instance->tx_semaphore = xSemaphoreCreateBinary();
    instance->rx_semaphore = xSemaphoreCreateBinary();

    if (instance->tx_semaphore == NULL || instance->rx_semaphore == NULL) {
        // Clean up if semaphore creation failed
        if (instance->tx_semaphore != NULL) {
            vSemaphoreDelete(instance->tx_semaphore);
            instance->tx_semaphore = NULL;
        }
        if (instance->rx_semaphore != NULL) {
            vSemaphoreDelete(instance->rx_semaphore);
            instance->rx_semaphore = NULL;
        }
        instance->allocated = false;
        return AGRBStatus_SEMAPHORE_ERROR;
    }

    // Release tx semaphore initially
    IOIF_UART_RELEASE_TX_SEMAPHORE(instance);

    instance->rx_dma = _uart_rx_dma_buffer[_uart_instance_count];

    instance->rx_task_handle = osThreadNew(_uart_rx_task, instance, &task_attributes);
    if (instance->rx_task_handle == NULL) {
        vSemaphoreDelete(instance->tx_semaphore);
        vSemaphoreDelete(instance->rx_semaphore);
        instance->tx_semaphore = NULL;
        instance->rx_semaphore = NULL;
        instance->allocated = false;
        return AGRBStatus_ERROR;
    }

    #endif

    // TODO: baudrate 설정 ioif_uart_reset_baudrate 호출
    // // Set baudrate
    // instance->huart->Init.BaudRate = _get_baudrate_value(config->baudrate);
    // if (HAL_UART_Init(instance->huart) != HAL_OK) {
    //     #if defined(USE_FREERTOS_DMA)
    //     // Proper cleanup order: DMA -> Task -> Semaphores
    //     HAL_UART_DMAStop(instance->huart);
    //     if (instance->rx_task_handle != NULL) {
    //         osThreadTerminate(instance->rx_task_handle);
    //         instance->rx_task_handle = NULL;
    //     }
    //     if (instance->tx_semaphore != NULL) {
    //         vSemaphoreDelete(instance->tx_semaphore);
    //         instance->tx_semaphore = NULL;
    //     }
    //     if (instance->rx_semaphore != NULL) {
    //         vSemaphoreDelete(instance->rx_semaphore);
    //         instance->rx_semaphore = NULL;
    //     }
    //     #endif
    //     instance->allocated = false;
    //     return AGRBStatus_INITIAL_FAILED;
    // }

    // Return the assigned instance ID and increment the instance count

    instance->allocated = true;

    *id = _uart_instance_count++;

    #if defined(USE_FREERTOS_DMA)
    {
        // Start UART DMA reception
        AGRBStatusDef start_status = ioif_uart_start(instance->id);
        if (start_status != AGRBStatus_OK) {
            // Cleanup on failure
            HAL_UART_DMAStop(instance->huart);
            if (instance->rx_task_handle != NULL) {
                osThreadTerminate(instance->rx_task_handle);
                instance->rx_task_handle = NULL;
            }
            vSemaphoreDelete(instance->tx_semaphore);
            vSemaphoreDelete(instance->rx_semaphore);
            instance->tx_semaphore = NULL;
            instance->rx_semaphore = NULL;
            instance->allocated = false;
            return start_status;
        }
    }
    #endif

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_uart_start(IOIF_UARTx_t id)
{
    #if defined(USE_FREERTOS_DMA)
    if (id > _uart_instance_count) return AGRBStatus_PARAM_ERROR;

    IOIF_UART_Instance_t* instance = &_uart_instances[id];

    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;

    // Clear RX buffer before starting
    memset(instance->rx_dma, 0, IOIF_UART_RX_DMA_BUFFER_SIZE);

    // Reset ring buffer position
    instance->ringbuffer.tail = 0;

    // Clear UART error flags before starting DMA
    if (__HAL_UART_GET_FLAG(instance->huart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(instance->huart, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(instance->huart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(instance->huart, UART_FLAG_PE)) {
        __HAL_UART_CLEAR_OREFLAG(instance->huart);
        __HAL_UART_CLEAR_FEFLAG(instance->huart);
        __HAL_UART_CLEAR_NEFLAG(instance->huart);
        __HAL_UART_CLEAR_PEFLAG(instance->huart);
        HAL_UART_AbortReceive(instance->huart);
    }

    // Start DMA reception in circular mode
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(instance->huart, instance->rx_dma, IOIF_UART_RX_DMA_BUFFER_SIZE);

    if (status != HAL_OK) {
        return _convert_hal_status(status);
    }
    return AGRBStatus_OK;

    #else
    return AGRBStatus_NOT_SUPPORTED;

    #endif

}

AGRBStatusDef ioif_uart_reset_baudrate(IOIF_UARTx_t id, IOIF_UART_Baudrate_e baudrate)
{
}

AGRBStatusDef ioif_uart_write(IOIF_UARTx_t id, uint8_t* tx_buf, uint32_t size)
{
    if (id >= IOIF_UART_MAX_INSTANCES) return AGRBStatus_PARAM_ERROR; // Invalid instance ID
    if (tx_buf == NULL || size == 0) return AGRBStatus_PARAM_ERROR; // Invalid parameters

    IOIF_UART_Instance_t* instance = &_uart_instances[id];

    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;

    HAL_StatusTypeDef hal_status;

    #if defined(USE_FREERTOS_DMA)

    if (size > IOIF_UART_TX_DMA_BUFFER_SIZE) return AGRBStatus_BUFFER_OVERFLOW;

    IOIF_UART_ACQUIRE_TX_SEMAPHORE(instance);
    {
        // Clear and copy to DMA buffer
        memset(_uart_tx_dma_buffer[id], 0, IOIF_UART_TX_DMA_BUFFER_SIZE);
        memcpy(_uart_tx_dma_buffer[id], tx_buf, size);

        // Clean D-Cache for DMA
        SCB_CleanDCache_by_Addr((uint32_t*)_uart_tx_dma_buffer[id], size);

        // Transmit via DMA
        hal_status = HAL_UART_Transmit_DMA(instance->huart, _uart_tx_dma_buffer[id], size);

    }
    // Semaphore will be released in TX complete callback

    #else
    // Blocking mode without DMA
    hal_status = HAL_UART_Transmit(instance->huart, tx_buf, size, 5000);
   
    #endif
    return _convert_hal_status(hal_status);
}

AGRBStatusDef ioif_uart_read(IOIF_UARTx_t id, uint8_t* rx_buf, uint32_t size)
{
    #if !defined(USE_FREERTOS_DMA)
    HAL_StatusTypeDef hal_status = HAL_UART_Receive(instance->huart, rx_buf, size, 5000);
    return _convert_hal_status(hal_status);
    #else
    // In DMA mode, reading should be done via callback
    return AGRBStatus_NOT_SUPPORTED;
    #endif
}

AGRBStatusDef ioif_uart_flush(IOIF_UARTx_t id)
{
    #if defined(USE_FREERTOS_DMA)
    if (id >= _uart_instance_count) return AGRBStatus_ERROR;

    IOIF_UART_Instance_t* instance = &_uart_instances[id];

    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;
    //TODO: 
    return AGRBStatus_OK;
    #else
    return AGRBStatus_NOT_SUPPORTED;
    #endif

}

AGRBStatusDef ioif_uart_reset(IOIF_UARTx_t id)
{
}

AGRBStatusDef ioif_uart_close(IOIF_UARTx_t id)
{
    return AGRBStatus_ERROR;
}

#if defined(USE_FREERTOS_DMA)
static uint32_t _debug_rx_count=0;
static uint32_t _debug_stage=0;
static uint32_t _debug_current_pos=0;
static void _uart_rx_task(void* argument)
{
	_debug_stage = 10;
    IOIF_UART_Instance_t* instance = (IOIF_UART_Instance_t*)argument;
    uint8_t bounce_buffer[IOIF_UART_RX_BOUNCE_BUFFER_SIZE];
    //IOIF_UART_RxEventCallback_t callback = instance->config.rx_event_callback;
    uint8_t* dma = instance->rx_dma;

    // [수정] vTaskDelayUntil을 위한 변수 초기화
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 1ms 주기
    while (1) {
        // osDelay(1); //UART로 들어오는 Traffic에 따라 변경 가능
        // 1ms 주기를 먼저 대기하여, 데이터가 없어도 CPU를 점유하지 않도록 합니다.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        _debug_rx_count++;
        _debug_stage = 1;
        // Wait for RX data notification
        // Get instance ID for buffer access
        // RX_SEMA_ACQ()
        // Invalidate D-Cache before reading DMA buffer (critical for cache-enabled systems)
    //    SCB_InvalidateDCache_by_Addr((uint32_t*)dma, IOIF_UART_RX_DMA_BUFFER_SIZE);

        // Calculate received data size
        uint32_t current_pos = IOIF_UART_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(instance->huart->hdmarx);
        uint32_t remain = 0;
        _debug_current_pos=current_pos;
        if (current_pos >= instance->ringbuffer.tail) {
            remain = current_pos - instance->ringbuffer.tail;
        } else {
            remain = (IOIF_UART_RX_DMA_BUFFER_SIZE - instance->ringbuffer.tail) + current_pos;
        }
        _debug_stage = 2;

        if (remain == 0) continue;
        
        //TODO: bounce buffer에다가 data를 복사함
        while(remain > 0){
            _debug_stage = 3;

            uint32_t copy_length = remain;
            if (remain > IOIF_UART_RX_BOUNCE_BUFFER_SIZE) copy_length = IOIF_UART_RX_BOUNCE_BUFFER_SIZE;

            //bounce buffer랑 남은 링버퍼 크기 비교 등 로직 구현
            //복사할 데이터가 ringbuff max에 도달하지 않았을때
            // [버그!] dma[instance->ringbuffer.tail] 주소가 아닌 
            // dma 주소에서 512바이트를 무효화하고 있습니다.
            // SCB_InvalidateDCache_by_Addr(dma, IOIF_UART_RX_BOUNCE_BUFFER_SIZE);

            if ((instance->ringbuffer.tail + copy_length) <= IOIF_UART_RX_DMA_BUFFER_SIZE) {

                // [수정] 1. 복사할 소스 주소와 크기만큼만 캐시를 무효화
                SCB_InvalidateDCache_by_Addr((uint32_t*)&dma[instance->ringbuffer.tail], copy_length);

                memcpy(bounce_buffer, &dma[instance->ringbuffer.tail], copy_length);
                instance->ringbuffer.tail = (instance->ringbuffer.tail + copy_length); //% IOIF_UART_RX_DMA_BUFFER_SIZE;
            }
            // 복사할 데이터가 ringbuff max에 넘겼을때
            else {
                // First part: tail부터 버퍼 끝까지
                uint32_t first_part = IOIF_UART_RX_DMA_BUFFER_SIZE - instance->ringbuffer.tail;
                uint32_t second_part = copy_length - first_part;

                // [수정] 1. 두 조각으로 나누어 각각 캐시를 무효화
                SCB_InvalidateDCache_by_Addr((uint32_t*)&dma[instance->ringbuffer.tail], first_part);
                SCB_InvalidateDCache_by_Addr((uint32_t*)&dma[0], second_part);

                // Second part: 버퍼 시작부터 나머지
                memcpy(bounce_buffer, &dma[instance->ringbuffer.tail], first_part);
                memcpy(&bounce_buffer[first_part], &dma[0], second_part);

                instance->ringbuffer.tail = second_part;
            }
            remain -= copy_length;
            _debug_stage = 4;

            if (instance->config.rx_event_callback != NULL) 
                instance->config.rx_event_callback( \
                    bounce_buffer,                  \
                    copy_length,                    \
                    instance->id );
                    
        }
        _debug_stage = 5;
    }
        //    RX_SEMA_REL();
}
#endif

/***************************
 * HAL UART Callbacks
 ****************************/

#if defined(USE_FREERTOS_DMA)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance == NULL) return;

    // Release tx semaphore to allow next transmission
    IOIF_UART_RELEASE_TX_SEMAPHORE_ISR(instance);
}
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance == NULL) return;

    // Signal RX task that new data has arrived
    IOIF_UART_SIGNAL_RX_COMPLETE(instance);

    // Restart DMA reception (circular mode alternative)
    uint32_t id = instance - _uart_instances;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, _uart_rx_dma_buffer[id], IOIF_UART_RX_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance == NULL) return;

    // Increment error counter
    instance->error_count++;

    // Clear UART error flags
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(huart);  // Overrun error
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FEFLAG(huart);   // Framing error
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) {
        __HAL_UART_CLEAR_NEFLAG(huart);   // Noise error
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE)) {
        __HAL_UART_CLEAR_PEFLAG(huart);   // Parity error
    }

    // Restart DMA reception on error
    uint32_t id = instance - _uart_instances;
    HAL_UART_DMAStop(huart);
    instance->ringbuffer.tail = 0;

    // Clear RX buffer before restart
    memset(_uart_rx_dma_buffer[id], 0, IOIF_UART_RX_DMA_BUFFER_SIZE);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, _uart_rx_dma_buffer[id], IOIF_UART_RX_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    // Release tx semaphore
    IOIF_UART_RELEASE_TX_SEMAPHORE_ISR(instance);
}
    */
#endif

#endif // AGRB_IOIF_UART_ENABLE
