/***********************************************************************************************************************
 * Description : STM32 USART interrupt interface header
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Private headers */
#include "uart.h"

/* Project related */
#include <Driver_GPIO.h>
#include <Driver_USART.h>
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#if !defined(STM32F1)
#define _HAL_RCC_USART_CLK_SLEEP_DISABLE(type_, num_) __HAL_RCC_##type_##num_##_CLK_SLEEP_DISABLE();
#define _HAL_RCC_USART_CLK_SLEEP_ENABLE(type_, num_)  __HAL_RCC_##type_##num_##_CLK_SLEEP_ENABLE();
#else
/* No support in STM32F1 */
#define _HAL_RCC_USART_CLK_SLEEP_DISABLE(type_, num_) {}
#define _HAL_RCC_USART_CLK_SLEEP_ENABLE(type_, num_) {}
#endif /* STM32F1 */

/***********************************************************************************************************************
 * Description : Generate a USART interrupt object.
 * Input       : type_       - USART type. One of:
 *                             1. USART
 *                             2. UART
 *                             3. LPUART
 *               num_        - USART interface number.
 *               user_conf_  - struct holding the USART user configuration.
 *               gpios_conf_ - struct holding the USART gpios configuration.
 **********************************************************************************************************************/
#define USART_INTERRUPT_GENERATE_OBJECT(type_, num_, user_conf_, gpios_conf_)                                          \
    extern uart_int_resources_t type_##num_##_RESOURCES;                                                               \
    static int32_t              type_##num_##_Initialize(ARM_USART_SignalEvent_t cb_event)                             \
    {                                                                                                                  \
        return uart_int_Initialize(&type_##num_##_RESOURCES, cb_event);                                                \
    }                                                                                                                  \
    static int32_t type_##num_##_Uninitialize(void)                                                                    \
    {                                                                                                                  \
        return uart_int_Uninitialize(&type_##num_##_RESOURCES);                                                        \
    }                                                                                                                  \
    static int32_t type_##num_##_PowerControl(ARM_POWER_STATE state)                                                   \
    {                                                                                                                  \
        return uart_int_PowerControl(&type_##num_##_RESOURCES, state);                                                 \
    }                                                                                                                  \
    static int32_t type_##num_##_Send(const void *data, uint32_t num)                                                  \
    {                                                                                                                  \
        return uart_int_Send(&type_##num_##_RESOURCES, data, num);                                                     \
    }                                                                                                                  \
    static int32_t type_##num_##_Receive(void *data, uint32_t num)                                                     \
    {                                                                                                                  \
        return uart_int_Receive(&type_##num_##_RESOURCES, data, num);                                                  \
    }                                                                                                                  \
    static int32_t type_##num_##_Stream(void *data)                                                                    \
    {                                                                                                                  \
        return uart_int_Stream(&type_##num_##_RESOURCES, data);                                                        \
    }                                                                                                                  \
    static int32_t type_##num_##_Transfer(const void *data_out, void *data_in, uint32_t num)                           \
    {                                                                                                                  \
        return uart_int_Transfer(&type_##num_##_RESOURCES, data_out, data_in, num);                                    \
    }                                                                                                                  \
    static uint32_t type_##num_##_GetTxCount(void)                                                                     \
    {                                                                                                                  \
        return uart_int_GetTxCount(&type_##num_##_RESOURCES);                                                          \
    }                                                                                                                  \
    static uint32_t type_##num_##_GetRxCount(void)                                                                     \
    {                                                                                                                  \
        return uart_int_GetRxCount(&type_##num_##_RESOURCES);                                                          \
    }                                                                                                                  \
    static int32_t type_##num_##_Control(uint32_t control, uint32_t arg)                                               \
    {                                                                                                                  \
        return uart_int_Control(&type_##num_##_RESOURCES, control, arg);                                               \
    }                                                                                                                  \
    static ARM_USART_STATUS type_##num_##_GetStatus(void)                                                              \
    {                                                                                                                  \
        return uart_int_GetStatus(&type_##num_##_RESOURCES);                                                           \
    }                                                                                                                  \
    void type_##num_##_IRQHandler(void)                                                                                \
    {                                                                                                                  \
        uart_int_UART_IRQHandler(&type_##num_##_RESOURCES);                                                            \
    }                                                                                                                  \
                                                                                                                       \
    ARM_DRIVER_USART Driver_##type_##num_ = {                                                                          \
        type_##num_##_Initialize,                                                                                      \
        type_##num_##_Uninitialize,                                                                                    \
        type_##num_##_PowerControl,                                                                                    \
        type_##num_##_Send,                                                                                            \
        type_##num_##_Receive,                                                                                         \
        type_##num_##_Stream,                                                                                          \
        type_##num_##_Transfer,                                                                                        \
        type_##num_##_GetTxCount,                                                                                      \
        type_##num_##_GetRxCount,                                                                                      \
        type_##num_##_Control,                                                                                         \
        type_##num_##_GetStatus,                                                                                       \
    };                                                                                                                 \
                                                                                                                       \
    static void enable_##type_##num_##_clock(void)                                                                     \
    {                                                                                                                  \
        __HAL_RCC_##type_##num_##_CLK_ENABLE();                                                                        \
        _HAL_RCC_USART_CLK_SLEEP_ENABLE(type_, num_);                                                                  \
    }                                                                                                                  \
    static void disable_##type_##num_##_clock(void)                                                                    \
    {                                                                                                                  \
        __HAL_RCC_##type_##num_##_CLK_DISABLE();                                                                       \
        _HAL_RCC_USART_CLK_SLEEP_DISABLE(type_, num_);                                                                 \
    }                                                                                                                  \
                                                                                                                       \
    uart_int_resources_t type_##num_##_RESOURCES = {                                                                   \
        .base =                                                                                                        \
            {                                                                                                          \
                .m = &uart_int_methods,                                                                                \
            },                                                                                                         \
        .clock_enable  = enable_##type_##num_##_clock,                                                                 \
        .clock_disable = disable_##type_##num_##_clock,                                                                \
        .user_conf     = &user_conf_,                                                                                  \
        .gpios         = &gpios_conf_,                                                                                 \
        .state         = UART_NOT_INITIALIZED,                                                                         \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*clock_control)(void);

typedef struct {
    ARM_GPIO_PIN *rx;
    ARM_GPIO_PIN *tx;
    ARM_GPIO_PIN *cts;
    ARM_GPIO_PIN *rts;
    uint32_t      tx_io_conf;
    uint32_t      rx_io_conf;
    uint32_t      rts_io_conf;
    uint32_t      cts_io_conf;
    uint8_t       alternate_f;
} const uart_int_gpios_t;

typedef enum {
    UART_NOT_INITIALIZED         = 0x00,
    UART_INITIALIZED_NOT_POWERED = 0x01,
    UART_POWERED                 = 0x03,
} uart_state_e;

typedef const struct {
    USART_TypeDef *instance; /* UART instance*/

    uint32_t baudrate;     /* UART baudrate*/
    uint32_t word_length;  /* Number of UART data bits */
    uint32_t stop_bits;    /* Number of stop bits */
    uint32_t parity;       /* UART parity bit */
    uint32_t oversampling; /* UART oversampling selection */
    uint32_t mode;         /* UART mode */
    uint32_t hw_control;   /* UART HW control */

    IRQn_Type irq; /* UART IRQ Handler */

#if defined(STM32L4) || defined(STM32WL)
    uint32_t                    one_bit_sampling; /* One bit sampling enable */
    UART_AdvFeatureInitTypeDef *advanced_features;
#endif
} uart_int_user_conf_t;

typedef struct {
    uart_resources_t base;

    clock_control clock_enable;  /* Clock enable function */
    clock_control clock_disable; /* Clock disable function */

    uart_state_e          state;     /* Uart driver state */
    uart_int_user_conf_t *user_conf; /* UART user configurations */
    uart_int_gpios_t *    gpios;     /* UART GPIOs */
    volatile uint32_t     rx_count;  /* Number of bytes received */

    ARM_USART_STATUS        status;   /* USART status */
    ARM_USART_SignalEvent_t callback; /* Callback function */
} uart_int_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

/* Initialize USART interface */
int32_t uart_int_Initialize(uart_int_resources_t *p_this, ARM_USART_SignalEvent_t cb_event);

/* De-initialize USART interface */
int32_t uart_int_Uninitialize(uart_int_resources_t *p_this);

/* Control USART Interface Power */
int32_t uart_int_PowerControl(uart_int_resources_t *p_this, ARM_POWER_STATE state);

/* Start sending data to USART transmitter */
int32_t uart_int_Send(uart_int_resources_t *p_this, const void *data, uint32_t num);

/* Start receiving data from USART receiver */
int32_t uart_int_Receive(uart_int_resources_t *p_this, void *data, uint32_t num);

/* Start receiving data from USART receiver in streaming mode */
int32_t uart_int_Stream(uart_int_resources_t *p_this, void *data);

/* Start sending/receiving data to/from USART transmitter/receiver */
int32_t uart_int_Transfer(uart_int_resources_t *p_this, const void *data_out, void *data_in, uint32_t num);

/* Get transmitted data count */
uint32_t uart_int_GetTxCount(uart_int_resources_t *p_this);

/* Get received data count */
uint32_t uart_int_GetRxCount(uart_int_resources_t *p_this);

/* Control USART Interface */
int32_t uart_int_Control(uart_int_resources_t *p_this, uint32_t control, uint32_t arg);

/* Get UART status */
ARM_USART_STATUS uart_int_GetStatus(uart_int_resources_t *p_this);

/* UART IRQHandler private implementation */
void uart_int_UART_IRQHandler(uart_int_resources_t *p_this);

/**********************************************************************************************************************/
/* Virtual Table                                                                                                      */
/**********************************************************************************************************************/

extern uart_methods_t const uart_int_methods;