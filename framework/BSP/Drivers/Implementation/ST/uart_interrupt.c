/***********************************************************************************************************************
 * Description : Implementation of STM32 USART interrupt.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header file */
#include "uart_interrupt.h"

/* Project related */
#if defined(STM32F1)
#include <stm32f1xx_hal_uart.h>
#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_usart.h>
#elif defined(STM32L4)
#include <stm32l4xx_hal_uart.h>
#include <stm32l4xx_ll_usart.h>
#elif defined(STM32F4)
#include <stm32f4xx_hal_uart.h>
#include <stm32f4xx_ll_usart.h>
#elif defined(STM32WL)
#include <stm32wlxx_hal_uart.h>
#include <stm32wlxx_ll_usart.h>
#elif defined(STM32U5)
#include <stm32u5xx_hal_uart.h>
#include <stm32u5xx_ll_usart.h>
#endif
#include <global.h>
#include <proj_assert.h>
#include <syscalls.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#define IS_UART_INITIALIZED(_state) (((_state)&UART_INITIALIZED_NOT_POWERED) == UART_INITIALIZED_NOT_POWERED)
#define IS_UART_POWERED(_state)     (((_state)&UART_POWERED) == UART_POWERED)

#define IS_UART_BUSY(_this) (IS_UART_BUSY_RX(_this) || IS_UART_BUSY_TX(_this))
#define IS_UART_BUSY_RX(_this)                                                                                         \
    ((HAL_UART_GetState(&(_this)->base.uart_handle) & HAL_UART_STATE_BUSY_RX) == HAL_UART_STATE_BUSY_RX)
#define IS_UART_BUSY_TX(_this)                                                                                         \
    ((HAL_UART_GetState(&(_this)->base.uart_handle) & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX)

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* ----------------------------------------------- Callback Methods ------------------------------------------------- */

static void tx_complete(uart_resources_t *p_base);
static void rx_complete(uart_resources_t *p_base);
static void error(uart_resources_t *p_base);

/**********************************************************************************************************************/
/* Interface Functions Implementations                                                                                */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description : Initialize UART interrupt interface
 * Input       : p_this   - Pointer to self
 *               cb_event - Pointer to callback function
 * Return      : Execution status
 **********************************************************************************************************************/
int32_t uart_int_Initialize(uart_int_resources_t *p_this, ARM_USART_SignalEvent_t cb_event)
{
    if (p_this->state != UART_NOT_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    /* Set UART instance */
    p_this->base.uart_handle.Instance = p_this->user_conf->instance;

    /* Set UART baudrate */
    p_this->base.uart_handle.Init.BaudRate = p_this->user_conf->baudrate;

    /* Set number of UART data bits */
    p_this->base.uart_handle.Init.WordLength = p_this->user_conf->word_length;

    /* Set UART stop bit */
    p_this->base.uart_handle.Init.StopBits = p_this->user_conf->stop_bits;

    /* Set UART parity */
    p_this->base.uart_handle.Init.Parity = p_this->user_conf->parity;

    /* Set UART mode */
    p_this->base.uart_handle.Init.Mode = p_this->user_conf->mode;

    /* Set UART oversampling */
    p_this->base.uart_handle.Init.OverSampling = p_this->user_conf->oversampling;

    /* Set UART HW control */
    p_this->base.uart_handle.Init.HwFlowCtl = p_this->user_conf->hw_control;

    /* Set UART advanced features struct */
#if defined(STM32L4) || defined(STM32WL) || defined(STM32U5) 
    /* Set UART one bit sampling flag */
    p_this->base.uart_handle.Init.OneBitSampling = p_this->user_conf->one_bit_sampling;

    if (p_this->user_conf->advanced_features != NULL)
    {
        p_this->base.uart_handle.AdvancedInit = *(p_this->user_conf->advanced_features);
    }
#endif

    /* Set callback function in driver resources */
    p_this->callback = cb_event;

    /* Configure GPIOs */
    /* Set GPIOs to ALTERNATE FUNCTION mode */
    gpio_control(p_this->gpios->rx, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->rx_io_conf);
    gpio_control(p_this->gpios->tx, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->tx_io_conf);

    /* Set GPIOs alternate function to UART's alternate function */
    gpio_control(
        p_this->gpios->rx, ARM_GPIO_SET_AF, ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));
    gpio_control(
        p_this->gpios->tx, ARM_GPIO_SET_AF, ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));

    /* Configure RTS pin if RTS is enabled */
    if (p_this->user_conf->hw_control & UART_HWCONTROL_RTS)
    {
        gpio_control(p_this->gpios->rts, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->rts_io_conf);
        gpio_control(p_this->gpios->rts,
                     ARM_GPIO_SET_AF,
                     ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));
    }

    /* Configure CTS pin if CTS is enabled */
    if (p_this->user_conf->hw_control & UART_HWCONTROL_CTS)
    {
        gpio_control(p_this->gpios->cts, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->cts_io_conf);
        gpio_control(p_this->gpios->cts,
                     ARM_GPIO_SET_AF,
                     ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));
    }

    /* Close gpios to explicitly enforce them to close mode */
    gpio_control(p_this->gpios->rx, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->gpios->tx, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->gpios->cts, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->gpios->rts, ARM_GPIO_SET_STATE, CLOSE);

    p_this->rx_count = 0;
    p_this->state    = UART_INITIALIZED_NOT_POWERED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : Uninitialize UART interrupt interface
 * Input       : p_this - Pointer to self
 * Return      : Execution status
 **********************************************************************************************************************/
int32_t uart_int_Uninitialize(uart_int_resources_t *p_this)
{
    p_this->state = UART_NOT_INITIALIZED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : Control UART interrupt Interface Power
 * Input       : p_this - Pointer to self
 *               state  - Power state
 * Return      : Execution status
 **********************************************************************************************************************/
int32_t uart_int_PowerControl(uart_int_resources_t *p_this, ARM_POWER_STATE state)
{
    /* Driver must be at least initialized */
    if (!IS_UART_INITIALIZED(p_this->state))
    {
        return ARM_DRIVER_ERROR;
    }

    /* Driver cannot be busy when preforming power control */
    if (IS_UART_BUSY(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (UART_INITIALIZED_NOT_POWERED == p_this->state)
            {
                return ARM_DRIVER_OK;
            }

            p_this->state = UART_INITIALIZED_NOT_POWERED;

            /* Call UART HAL uninitialize function */
            HAL_UART_DeInit(&p_this->base.uart_handle);

            /* Close GPIOs */
            gpio_control(p_this->gpios->rx, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(p_this->gpios->tx, ARM_GPIO_SET_STATE, CLOSE);

            /* Disable peripheral clock */
            p_this->clock_disable();

            if (p_this->user_conf->hw_control & UART_HWCONTROL_RTS)
            {
                gpio_control(p_this->gpios->rts, ARM_GPIO_SET_STATE, CLOSE);
            }

            if (p_this->user_conf->hw_control & UART_HWCONTROL_CTS)
            {
                gpio_control(p_this->gpios->cts, ARM_GPIO_SET_STATE, CLOSE);
            }

            /* Disable peripheral interrupts */
            nvic_enable_irq(p_this->user_conf->irq, false);

            return ARM_DRIVER_OK;
        }
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        case ARM_POWER_FULL:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (UART_POWERED == p_this->state)
            {
                return ARM_DRIVER_OK;
            }

            /* Enable UART interrupts */
            nvic_clear_irq(p_this->user_conf->irq);
            nvic_enable_irq(p_this->user_conf->irq, true);

            p_this->state = UART_POWERED;

            /* Enable UART clock */
            p_this->clock_enable();

            /* Open GPIOs */
            gpio_control(p_this->gpios->rx, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(p_this->gpios->tx, ARM_GPIO_SET_STATE, OPEN);

            /* Configure RTS pin if RTS is enabled */
            if (p_this->user_conf->hw_control & UART_HWCONTROL_RTS)
            {
                gpio_control(p_this->gpios->rts, ARM_GPIO_SET_STATE, OPEN);
            }

            /* Configure CTS pin if CTS is enabled */
            if (p_this->user_conf->hw_control & UART_HWCONTROL_CTS)
            {
                gpio_control(p_this->gpios->cts, ARM_GPIO_SET_STATE, OPEN);
            }

            /* Call UART HAL initialization function */
            HAL_UART_Init(&p_this->base.uart_handle);

            return ARM_DRIVER_OK;
        }

        /* Parameter error */
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}

/***********************************************************************************************************************
 * Description : Start sending data to UART interrupt transmitter
 * Input       : p_this - Pointer to self
 *               data   - Pointer to buffer with data to send
 *               num    - Number of data items to send
 * Return      : Execution status
 **********************************************************************************************************************/
int32_t uart_int_Send(uart_int_resources_t* p_this, const void* data, uint32_t num)
{
    if (num == 0)
    {
        proj_assert(false);

        return ARM_DRIVER_OK;
    }

    /* Cannot call Send if the driver is not powered or is currently busy sending an earlier message */
    if (!IS_UART_POWERED(p_this->state))
    {
        return ARM_DRIVER_ERROR;
    }

    if (IS_UART_BUSY_TX(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    HAL_StatusTypeDef hal_transmit_result =
        HAL_UART_Transmit_IT(&p_this->base.uart_handle, (uint8_t *)data, (uint16_t)num);

    /* Return execution code according to the UART HAL execution code */
    switch (hal_transmit_result)
    {
        case HAL_OK:
        {
            return ARM_DRIVER_OK;
        }
        case HAL_BUSY:
        {
            return ARM_DRIVER_ERROR_BUSY;
        }
        case HAL_TIMEOUT:
        {
            return ARM_DRIVER_ERROR_TIMEOUT;
        }
        default:
        {
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
 * Description : Start receiving data from UART interrupt receiver
 * Input       : p_this - Pointer to self
 *               data   - Pointer to buffer for data to receive
 *               num    - Number of data items to receive
 * Return      : Execution status
 **********************************************************************************************************************/
int32_t uart_int_Receive(uart_int_resources_t* p_this, void* data, uint32_t num)
{
    if (num == 0)
    {
        proj_assert(false);

        return ARM_DRIVER_OK;
    }

    /* Cannot call Receive if the driver is not powered or is currently busy receiving an earlier message */
    if (!IS_UART_POWERED(p_this->state))
    {
        return ARM_DRIVER_ERROR;
    }

    if (IS_UART_BUSY_RX(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    HAL_StatusTypeDef hal_receive_result =
        HAL_UART_Receive_IT(&p_this->base.uart_handle, (uint8_t *)data, (uint16_t)num);

    /* Return execution code according to the UART HAL execution code */
    switch (hal_receive_result)
    {
        case HAL_OK:
        {
            p_this->rx_count = 0;

            /* Enable interrupt that signals end of data. This event is used to handle receiving timeout.
               Must be enabled after calling HAL_UART_Receive_IT to avoid IDLE interrupt handle execution before update
               of new receive param */
            __HAL_UART_CLEAR_FLAG(&p_this->base.uart_handle, UART_FLAG_IDLE);
            __HAL_UART_ENABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);

            return ARM_DRIVER_OK;
        }
        case HAL_BUSY:
        {
            return ARM_DRIVER_ERROR_BUSY;
        }
        case HAL_TIMEOUT:
        {
            return ARM_DRIVER_ERROR_TIMEOUT;
        }
        default:
        {
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
 * Description : Start receiving data from UART interrupt receiver in non-streaming mode
 * Input       : p_this - Pointer to self
 *               data   - Pointer to buffer with data to send
 * Return      : Execution status
 ***********************************************************************************************************************/
int32_t uart_int_Stream(uart_int_resources_t *p_this, void *data)
{
    ARGUMENT_UNUSED(data);
    ARGUMENT_UNUSED(p_this);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description : Start sending/receiving data to/from UART interrupt transmitter/receiver
 * Input       : p_this   - Pointer to self
 *               data_out - Pointer to buffer with data to send
 *               data_in  - Pointer to buffer for data to receive
 *               num      - Number of data items to transfer
 * Return      : Execution status
 ***********************************************************************************************************************/
int32_t uart_int_Transfer(uart_int_resources_t *p_this, const void *data_out, void *data_in, uint32_t num)
{
    ARGUMENT_UNUSED(data_out);
    ARGUMENT_UNUSED(data_in);
    ARGUMENT_UNUSED(num);
    ARGUMENT_UNUSED(p_this);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description : Get transferred data count
 * Input       : p_this - Pointer to self
 * Return      : Number of bytes transmitted
 **********************************************************************************************************************/
uint32_t uart_int_GetTxCount(uart_int_resources_t *p_this)
{
    return p_this->base.uart_handle.TxXferCount;
}

/***********************************************************************************************************************
 * Description : Get received data count
 * Input       : p_this - Pointer to self
 * Return      : Number of bytes received
 **********************************************************************************************************************/
uint32_t uart_int_GetRxCount(uart_int_resources_t *p_this)
{
    return p_this->rx_count;
}

/***********************************************************************************************************************
 * Description : Control UART interrupt Interface
 * Input       : p_this  - Pointer to self
 *               control - Operation
 *               arg     - Argument of operation (optional)
 * Return      : Execution status
 **********************************************************************************************************************/
int32_t uart_int_Control(uart_int_resources_t *p_this, uint32_t control, uint32_t arg)
{
    control &= ARM_USART_CONTROL_Msk;

    if (!IS_UART_POWERED(p_this->state) && control != ARM_USART_MODE_ASYNCHRONOUS)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        /* In case we want to set the baudrate in asynchronous mode */
        case ARM_USART_MODE_ASYNCHRONOUS:
        {
            /* Baudrate change is currently only used by stm32f1 projects and isn't supported for other families */
#if defined(STM32F1)
            LL_RCC_ClocksTypeDef rcc_clocks;
            uint32_t             freq = 0;
            LL_RCC_GetSystemClocksFreq(&rcc_clocks);
            if (p_this->user_conf->instance == USART1)
            {
                freq = rcc_clocks.PCLK2_Frequency;
            }
            else
            {
                freq = rcc_clocks.PCLK1_Frequency;
            }

            LL_USART_SetBaudRate(p_this->user_conf->instance, freq, arg);
            return ARM_DRIVER_OK;
#else
            ARGUMENT_UNUSED(arg);
            proj_assert(false);

            return ARM_DRIVER_ERROR_UNSUPPORTED;
#endif
        }
        case ARM_USART_ABORT_RECEIVE:
        {
            if (HAL_OK != HAL_UART_AbortReceive_IT(&p_this->base.uart_handle))
            {
                return ARM_DRIVER_ERROR;
            }

            return ARM_DRIVER_OK;
        }
        case ARM_USART_ABORT_SEND:
        {
            if (HAL_OK != HAL_UART_AbortTransmit_IT(&p_this->base.uart_handle))
            {
                return ARM_DRIVER_ERROR;
            }

            return ARM_DRIVER_OK;
        }
        case ARM_USART_ABORT_TRANSFER:
        {
            if (HAL_OK != HAL_UART_Abort_IT(&p_this->base.uart_handle))
            {
                return ARM_DRIVER_ERROR;
            }

            return ARM_DRIVER_OK;
        }
        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }
}

/***********************************************************************************************************************
 * Description : Get UART status
 * Input       : p_this - Pointer to self
 * Return      : UART status
 **********************************************************************************************************************/
ARM_USART_STATUS uart_int_GetStatus(uart_int_resources_t *p_this)
{
    /* Get UART handler state */
    HAL_UART_StateTypeDef uart_state = HAL_UART_GetState(&p_this->base.uart_handle);

    /* Configure status according to UART handler state */
    switch (uart_state)
    {
        case HAL_UART_STATE_BUSY_RX:
        {
            p_this->status.rx_busy = true;
            p_this->status.tx_busy = false;
            break;
        }
        case HAL_UART_STATE_BUSY_TX:
        {
            p_this->status.tx_busy = true;
            p_this->status.rx_busy = false;
            break;
        }
        case HAL_UART_STATE_BUSY_TX_RX:
        {
            p_this->status.rx_busy = true;
            p_this->status.tx_busy = true;
            break;
        }
        case HAL_UART_STATE_READY:
        case HAL_UART_STATE_RESET:
        {
            p_this->status.rx_busy = false;
            p_this->status.tx_busy = false;
        }
        default:
        {
            /* States not defined in the CMSIS USART status */
            break;
        }
    }

    /* In case there is a parity error */
    if (p_this->base.uart_handle.ErrorCode & HAL_UART_ERROR_PE)
    {
        p_this->status.rx_parity_error = true;
    }

    /* In case there is a frame error */
    if (p_this->base.uart_handle.ErrorCode & HAL_UART_ERROR_FE)
    {
        p_this->status.rx_framing_error = true;
    }

    return p_this->status;
}

/***********************************************************************************************************************
 * Description : UART IRQHandler private implementation
 * Input       : p_this - Pointer to interface resources
 * Return      : None
 * Note        : We handle the IDLE separately, since it's not supported by HAL_UART_IRQHandler.
 **********************************************************************************************************************/
void uart_int_UART_IRQHandler(uart_int_resources_t *p_this)
{
    bool is_transmit_cplt = __HAL_UART_GET_FLAG(&p_this->base.uart_handle, UART_FLAG_TC);
    bool is_receive_data  = __HAL_UART_GET_FLAG(&p_this->base.uart_handle, UART_FLAG_RXNE);

    /* Transmit complete or receive data interrupt has occurred. We want to handle it first */
    if (is_transmit_cplt || is_receive_data)
    {
        HAL_UART_IRQHandler(&p_this->base.uart_handle);
    }

    bool is_timeout = __HAL_UART_GET_FLAG(&p_this->base.uart_handle, UART_FLAG_IDLE);

    /* IDLE flag may be set even if we're not in RX, just because data is sent on the RX IO */
    if (is_timeout && IS_UART_BUSY_RX(p_this))
    {
        uint32_t rx_count = p_this->base.uart_handle.RxXferSize - p_this->base.uart_handle.RxXferCount;

        /* Timeout interrupt has occurred and we didn't handle this data yet */
        if (rx_count != 0)
        {
            p_this->rx_count = rx_count;
            __HAL_UART_DISABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);
            HAL_UART_AbortReceive(&p_this->base.uart_handle);


            if (p_this->callback != NULL)
            {
                p_this->callback(ARM_USART_EVENT_RX_TIMEOUT);
            }
        }
    }
    /* Error interrupt has occurred */
    else if (!is_transmit_cplt && !is_receive_data)
    {
        HAL_UART_IRQHandler(&p_this->base.uart_handle);
    }

    if (is_timeout)
    {
        /* Anyway we need to clear timeout flag */
        __HAL_UART_CLEAR_IDLEFLAG(&p_this->base.uart_handle);
    }
}

/**********************************************************************************************************************/
/* Protected Function Implementations                                                                                 */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Tx transfer completed callback.
 * Input      : p_base - pointer base
 * Return     : None
 **********************************************************************************************************************/
static void tx_complete(uart_resources_t *p_base)
{
    uart_int_resources_t *p_this = (uart_int_resources_t *)p_base;

    __HAL_UART_CLEAR_FLAG(&p_this->base.uart_handle, UART_FLAG_TC);

    if (p_this->callback != NULL)
    {
        p_this->callback(ARM_USART_EVENT_TX_COMPLETE);
    }
}

/***********************************************************************************************************************
 * Description: Rx transfer completed callback.
 * Input      : p_base - pointer base
 * Return     : None
 **********************************************************************************************************************/
static void rx_complete(uart_resources_t *p_base)
{
    uart_int_resources_t *p_this = (uart_int_resources_t *)p_base;

    p_this->rx_count = p_this->base.uart_handle.RxXferSize - p_this->base.uart_handle.RxXferCount;

    __HAL_UART_DISABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&p_this->base.uart_handle);

    if (p_this->callback != NULL)
    {
        p_this->callback(ARM_USART_EVENT_RECEIVE_COMPLETE);
    }
}

/***********************************************************************************************************************
 * Description: Error transfer callback.
 * Input      : p_base - pointer base
 * Return     : None
 **********************************************************************************************************************/
void error(uart_resources_t *p_base)
{
    uart_int_resources_t *p_this = (uart_int_resources_t *)p_base;

    /* Figure what event occured by checking the USART status after handling the interrupt */
    uint32_t hal_error = HAL_UART_GetError(&p_this->base.uart_handle);
    uint32_t event     = 0;

    if (hal_error & HAL_UART_ERROR_PE)
    {
        event |= ARM_USART_EVENT_RX_PARITY_ERROR;
    }

    if (hal_error & HAL_UART_ERROR_FE)
    {
        event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
    }

    if (hal_error & HAL_UART_ERROR_ORE)
    {
        event |= ARM_USART_EVENT_RX_OVERFLOW;
    }

    /* Call UART callback if it is not NULL*/
    if (p_this->callback != NULL)
    {
        p_this->callback(event);
    }
}

/**********************************************************************************************************************/
/* Virtual Table                                                                                                      */
/**********************************************************************************************************************/

uart_methods_t const uart_int_methods = {
    .TxComplete     = tx_complete,
    .RxComplete     = rx_complete,
    .RxHalfComplete = NULL,
    .Error          = error,
};