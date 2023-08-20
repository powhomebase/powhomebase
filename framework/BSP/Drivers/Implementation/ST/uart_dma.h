/***********************************************************************************************************************
 * Description: STM32F1 and STM32WL55 UART DMA driver header file.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project related */
#include CMSIS_device_header
#include <Driver_DMA.h>
#include <Driver_GPIO.h>
#include <Driver_USART.h>
#include <ring_buffer.h>
#include <uart.h>

/**********************************************************************************************************************/
/* Macros	 											                                                              */
/**********************************************************************************************************************/

#define _HAL_RCC_USART_DMA_CLK_SLEEP_DISABLE(num_) __HAL_RCC_USART##num_##_CLK_SLEEP_DISABLE()
#define _HAL_RCC_USART_DMA_CLK_SLEEP_ENABLE(num_)  __HAL_RCC_USART##num_##_CLK_SLEEP_ENABLE()

#define UART_DMA_GENERATE_OBJECT(uart_num, dma_tx_handle, dma_tx_driver, dma_rx_handle, dma_rx_driver)                 \
    _UART_DMA_GENERATE_OBJECT(uart_num, dma_tx_handle, dma_tx_driver, dma_rx_handle, dma_rx_driver, UART);

#define USART_DMA_GENERATE_OBJECT(uart_num, dma_tx_handle, dma_tx_driver, dma_rx_handle, dma_rx_driver)                \
    _UART_DMA_GENERATE_OBJECT(uart_num, dma_tx_handle, dma_tx_driver, dma_rx_handle, dma_rx_driver, USART);

#define _UART_DMA_GENERATE_OBJECT(uart_num, _dma_tx_handle, dma_tx_driver, _dma_rx_handle, dma_rx_driver, type)        \
    _UART_DMA_GENERATE_CLKS(uart_num)                                                                                  \
    extern uart_dma_resources_t type##uart_num##_RESOURCES;                                                            \
    static int32_t              type##uart_num##_initialize(ARM_USART_SignalEvent_t cb_event)                          \
    {                                                                                                                  \
        return uart_dma_initialize(&type##uart_num##_RESOURCES, cb_event);                                             \
    }                                                                                                                  \
    static int32_t type##uart_num##_uninitialize(void)                                                                 \
    {                                                                                                                  \
        return uart_dma_uninitialize(&type##uart_num##_RESOURCES);                                                     \
    }                                                                                                                  \
    static int32_t type##uart_num##_power_control(ARM_POWER_STATE state)                                               \
    {                                                                                                                  \
        return uart_dma_power_control(&type##uart_num##_RESOURCES, state);                                             \
    }                                                                                                                  \
    static int32_t type##uart_num##_send(const void* data, uint32_t num)                                               \
    {                                                                                                                  \
        return uart_dma_send(&type##uart_num##_RESOURCES, data, num);                                                  \
    }                                                                                                                  \
    static int32_t type##uart_num##_receive(void* data, uint32_t num)                                                  \
    {                                                                                                                  \
        return uart_dma_receive(&type##uart_num##_RESOURCES, data, num);                                               \
    }                                                                                                                  \
    static int32_t type##uart_num##_stream(void* data)                                                                 \
    {                                                                                                                  \
        return uart_dma_stream(&type##uart_num##_RESOURCES, (ring_buff_writer_t*)data);                                \
    }                                                                                                                  \
    static int32_t type##uart_num##_transfer(const void* data_out, void* data_in, uint32_t num)                        \
    {                                                                                                                  \
        return uart_dma_transfer(&type##uart_num##_RESOURCES, data_out, data_in, num);                                 \
    }                                                                                                                  \
    static uint32_t type##uart_num##_get_tx_count(void)                                                                \
    {                                                                                                                  \
        return uart_dma_get_tx_count(&type##uart_num##_RESOURCES);                                                     \
    }                                                                                                                  \
    static uint32_t type##uart_num##_get_rx_count(void)                                                                \
    {                                                                                                                  \
        return uart_dma_get_rx_count(&type##uart_num##_RESOURCES);                                                     \
    }                                                                                                                  \
    static int32_t type##uart_num##_control(uint32_t control, uint32_t arg)                                            \
    {                                                                                                                  \
        return uart_dma_control(&type##uart_num##_RESOURCES, control, arg);                                            \
    }                                                                                                                  \
    static ARM_USART_STATUS type##uart_num##_get_status(void)                                                          \
    {                                                                                                                  \
        return uart_dma_get_status(&type##uart_num##_RESOURCES);                                                       \
    }                                                                                                                  \
    void USART##uart_num##_IRQHandler(void)                                                                            \
    {                                                                                                                  \
        return uart_dma_irq_handler(&type##uart_num##_RESOURCES);                                                      \
    }                                                                                                                  \
                                                                                                                       \
    ARM_DRIVER_USART Driver_##type##uart_num = {                                                                       \
        type##uart_num##_initialize,                                                                                   \
        type##uart_num##_uninitialize,                                                                                 \
        type##uart_num##_power_control,                                                                                \
        type##uart_num##_send,                                                                                         \
        type##uart_num##_receive,                                                                                      \
        type##uart_num##_stream,                                                                                       \
        type##uart_num##_transfer,                                                                                     \
        type##uart_num##_get_tx_count,                                                                                 \
        type##uart_num##_get_rx_count,                                                                                 \
        type##uart_num##_control,                                                                                      \
        type##uart_num##_get_status,                                                                                   \
    };                                                                                                                 \
                                                                                                                       \
    uart_dma_resources_t type##uart_num##_RESOURCES = {                                                                \
        .base =                                                                                                        \
            {                                                                                                          \
                .m = &uart_dma_methods,                                                                                \
            },                                                                                                         \
        .p_user_conf = &type##uart_num##_USER_CONF,                                                                    \
        .p_gpios =                                                                                                     \
            &(uart_dma_gpios_t){                                                                                       \
                &GPIO_##type##uart_num##_RX,                                                                           \
                GPIO_##type##uart_num##_RX_CONF,                                                                       \
                &GPIO_##type##uart_num##_TX,                                                                           \
                GPIO_##type##uart_num##_TX_CONF,                                                                       \
                type##uart_num##_GPIO_AF,                                                                              \
            },                                                                                                         \
        .p_conf =                                                                                                      \
            &(uart_dma_conf_t){                                                                                        \
                .clock_enable  = enable_usart##uart_num##_clock,                                                       \
                .clock_disable = disable_usart##uart_num##_clock,                                                      \
                .dma_tx        = &dma_tx_driver,                                                                       \
                .dma_rx        = &dma_rx_driver,                                                                       \
                .dma_tx_handle = &_dma_tx_handle,                                                                      \
                .dma_rx_handle = &_dma_rx_handle,                                                                      \
                .irq           = USART##uart_num##_IRQn,                                                               \
            },                                                                                                         \
        .state = UART_DMA_NOT_INITIALIZED,                                                                             \
    };

#define _UART_DMA_GENERATE_CLKS(num_)                                                                                  \
    static void enable_usart##num_##_clock(void)                                                                       \
    {                                                                                                                  \
        __HAL_RCC_USART##num_##_CLK_ENABLE();                                                                          \
        _HAL_RCC_USART_DMA_CLK_SLEEP_ENABLE(num_);                                                                     \
    }                                                                                                                  \
    static void disable_usart##num_##_clock(void)                                                                      \
    {                                                                                                                  \
        _HAL_RCC_USART_DMA_CLK_SLEEP_DISABLE(num_);                                                                    \
        __HAL_RCC_USART##num_##_CLK_DISABLE();                                                                         \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef void (*clock_control)(void);

typedef enum {
    UART_DMA_NOT_INITIALIZED,
    UART_DMA_INITIALIZED,
    UART_DMA_NOT_POWERED,
    UART_DMA_POWERED,
} uart_dma_state_t;

typedef struct {
    USART_TypeDef* instance;
    uint32_t       baudrate;
    uint32_t       word_length;
    uint32_t       stop_bits;
    uint32_t       parity;
    uint32_t       oversampling;
    uint32_t       mode;
    uint32_t       hw_control;
#if defined(STM32WL) || defined(STM32U5) 
    uint32_t one_bit_sampling;
    uint32_t prescaler;
#endif
} uart_dma_user_conf_t;

typedef struct {
    ARM_GPIO_PIN* rx_gpio;
    uint32_t      rx_conf;
    ARM_GPIO_PIN* tx_gpio;
    uint32_t      tx_conf;
    uint8_t       alternate_fun;
} uart_dma_gpios_t;

typedef struct {
    clock_control      clock_enable;
    clock_control      clock_disable;
    ARM_DRIVER_DMA*    dma_rx;
    ARM_DRIVER_DMA*    dma_tx;
    DMA_HandleTypeDef* dma_rx_handle;
    DMA_HandleTypeDef* dma_tx_handle;
    IRQn_Type          irq;
} const uart_dma_conf_t;

typedef struct {
    uart_resources_t base;

    uart_dma_user_conf_t* const p_user_conf;
    uart_dma_gpios_t* const     p_gpios;
    uart_dma_conf_t* const      p_conf;
    uart_dma_state_t            state;

    ARM_USART_STATUS        status;
    ARM_USART_SignalEvent_t callback;

    ring_buff_writer_t writer;
    uint32_t           rx_count;
} uart_dma_resources_t;

/**********************************************************************************************************************/
/* Interface Functions Declarations                                                                                   */
/**********************************************************************************************************************/

int32_t uart_dma_initialize(uart_dma_resources_t* p_this, ARM_USART_SignalEvent_t cb_event);

int32_t uart_dma_uninitialize(uart_dma_resources_t* p_this);

int32_t uart_dma_power_control(uart_dma_resources_t* p_this, ARM_POWER_STATE state);

int32_t uart_dma_send(uart_dma_resources_t* p_this, const void* data, uint32_t num);

int32_t uart_dma_receive(uart_dma_resources_t* p_this, void* data, uint32_t num);

int32_t uart_dma_stream(uart_dma_resources_t* p_this, ring_buff_writer_t* writer);

int32_t uart_dma_transfer(uart_dma_resources_t* p_this, const void* data_out, void* data_in, uint32_t num);

uint32_t uart_dma_get_tx_count(uart_dma_resources_t* p_this);

uint32_t uart_dma_get_rx_count(uart_dma_resources_t* p_this);

int32_t uart_dma_control(uart_dma_resources_t* p_this, uint32_t control, uint32_t arg);

ARM_USART_STATUS uart_dma_get_status(uart_dma_resources_t* p_this);

void uart_dma_irq_handler(uart_dma_resources_t* p_this);

/**********************************************************************************************************************/
/* Virtual Table                                                                                                      */
/**********************************************************************************************************************/

extern uart_methods_t const uart_dma_methods;
