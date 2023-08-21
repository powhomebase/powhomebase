/***********************************************************************************************************************
 * Description: STM32F1 and STM32WL55 UART DMA driver implementation
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header file */
#include "uart_dma.h"

/* Project related */
#include <dma.h>
#include <global.h>
#include <proj_exception.h>
#if defined(STM32F1)
#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_usart.h>
#elif defined(STM32F4)
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_usart.h>
#elif defined(STM32L4)
#include <stm32l4xx_ll_rcc.h>
#include <stm32l4xx_ll_usart.h>
#elif defined(STM32U5)
#include <stm32u5xx_ll_rcc.h>
#include <stm32u5xx_ll_usart.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_rcc.h>
#include <stm32wlxx_ll_usart.h>
#endif
#include <string.h>
#include <syscalls.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#ifndef DMA_CIRCULAR
#define DMA_CIRCULAR DMA_LINKEDLIST_CIRCULAR
#endif

/**********************************************************************************************************************/
/* Private Functions                                                                                                  */
/**********************************************************************************************************************/

static void     receive_complete(uart_dma_resources_t* p_this);
static uint32_t dma_transfer_count(uart_dma_resources_t* p_this);

/* --------------------------------------------- Callback Methods --------------------------------------------------- */

static void tx_complete(uart_resources_t* p_base);
static void rx_complete(uart_resources_t* p_base);
static void rx_half_complete(uart_resources_t* p_base);
static void error(uart_resources_t* p_base);

/**********************************************************************************************************************/
/* Interface Functions Implementations                                                                                */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Initialize UART DMA interface
 * Input          : p_this   - Pointer to self
                    cb_event - Pointer to callback function
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_initialize(uart_dma_resources_t* p_this, ARM_USART_SignalEvent_t cb_event)
{
    if (p_this->state != UART_DMA_NOT_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    p_this->base.uart_handle.Instance          = p_this->p_user_conf->instance;
    p_this->base.uart_handle.Init.BaudRate     = p_this->p_user_conf->baudrate;
    p_this->base.uart_handle.Init.WordLength   = p_this->p_user_conf->word_length;
    p_this->base.uart_handle.Init.StopBits     = p_this->p_user_conf->stop_bits;
    p_this->base.uart_handle.Init.Parity       = p_this->p_user_conf->parity;
    p_this->base.uart_handle.Init.Mode         = p_this->p_user_conf->mode;
    p_this->base.uart_handle.Init.OverSampling = p_this->p_user_conf->oversampling;
    p_this->base.uart_handle.Init.HwFlowCtl    = p_this->p_user_conf->hw_control;
#if defined(STM32WL)
    p_this->base.uart_handle.Init.OneBitSampling = p_this->p_user_conf->one_bit_sampling;
    p_this->base.uart_handle.Init.ClockPrescaler = p_this->p_user_conf->prescaler;
#endif

    p_this->base.uart_handle.hdmatx = p_this->p_conf->dma_tx_handle;
    p_this->base.uart_handle.hdmarx = p_this->p_conf->dma_rx_handle;

    p_this->callback = cb_event;

    /* Set GPIOs to ALTERNATE FUNCTION mode */
    gpio_control(p_this->p_gpios->rx_gpio, ARM_GPIO_CONFIGURE_STATE, p_this->p_gpios->rx_conf);
    gpio_control(p_this->p_gpios->tx_gpio, ARM_GPIO_CONFIGURE_STATE, p_this->p_gpios->tx_conf);

    /* Set GPIOs alternate function to UART's alternate function */
    gpio_control(p_this->p_gpios->rx_gpio,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(p_this->p_gpios->alternate_fun, p_this->p_gpios->alternate_fun));
    gpio_control(p_this->p_gpios->tx_gpio,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(p_this->p_gpios->alternate_fun, p_this->p_gpios->alternate_fun));

    /* Initialize DMAs and register UART DMA IRQ as a callback function to the DMAs lower layer */
    p_this->p_conf->dma_tx->Initialize(NULL);
    p_this->p_conf->dma_rx->Initialize(NULL);

    p_this->p_conf->clock_enable();

    /* Call UART HAL initialization function */
    HAL_StatusTypeDef res = HAL_UART_Init(&p_this->base.uart_handle);
    if (res != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    p_this->p_conf->clock_disable();

    /* Close gpios */
    gpio_control(p_this->p_gpios->rx_gpio, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->p_gpios->tx_gpio, ARM_GPIO_SET_STATE, CLOSE);

    p_this->state = UART_DMA_INITIALIZED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description    : De-initialize UART DMA interface
 * Input          : p_this - Pointer to self
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_uninitialize(uart_dma_resources_t* p_this)
{
    p_this->state = UART_DMA_NOT_INITIALIZED;

    p_this->p_conf->clock_enable();

    HAL_StatusTypeDef res = HAL_UART_DeInit(&p_this->base.uart_handle);
    if (res != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    p_this->p_conf->clock_disable();

    p_this->p_conf->dma_tx->Uninitialize();
    p_this->p_conf->dma_rx->Uninitialize();

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description    : Control UART DMA Interface Power
 * Input          : p_this - Pointer to self
                    state  - Power state
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_power_control(uart_dma_resources_t* p_this, ARM_POWER_STATE state)
{
    if (p_this->state == UART_DMA_NOT_INITIALIZED)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        /* Turn off driver */
        case ARM_POWER_OFF:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (UART_DMA_NOT_POWERED == p_this->state)
            {
                return ARM_DRIVER_OK;
            }

            p_this->state = UART_DMA_NOT_POWERED;

            p_this->p_conf->clock_disable();

            /* Disable DMAs */
            p_this->p_conf->dma_tx->PowerControl(ARM_POWER_OFF);
            p_this->p_conf->dma_rx->PowerControl(ARM_POWER_OFF);

            /* Close GPIOs */
            gpio_control(p_this->p_gpios->rx_gpio, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(p_this->p_gpios->tx_gpio, ARM_GPIO_SET_STATE, CLOSE);

            /* Disable peripheral interrupts */
            nvic_enable_irq(p_this->p_conf->irq, false);

            __HAL_UART_DISABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);

            return ARM_DRIVER_OK;
        }

        /* Low power mode - NOT SUPPORTED */
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Turn on driver */
        case ARM_POWER_FULL:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (UART_DMA_POWERED == p_this->state)
            {
                return ARM_DRIVER_OK;
            }

            /* Open GPIOs */
            gpio_control(p_this->p_gpios->rx_gpio, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(p_this->p_gpios->tx_gpio, ARM_GPIO_SET_STATE, OPEN);

            /* Enable DMAs */
            p_this->p_conf->dma_tx->PowerControl(ARM_POWER_FULL);
            p_this->p_conf->dma_rx->PowerControl(ARM_POWER_FULL);

            /* Enable UART interrupts */
            nvic_clear_irq(p_this->p_conf->irq);
            nvic_enable_irq(p_this->p_conf->irq, true);

            __HAL_UART_ENABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);

            p_this->state = UART_DMA_POWERED;

            p_this->p_conf->clock_enable();

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
 * Description    : Start sending data to UART DMA transmitter.
 * Input          : p_this - Pointer to self
                    data   - Pointer to buffer with data to send
                    num    - Number of data items to send
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_send(uart_dma_resources_t* p_this, void const* data, uint32_t num)
{
    if (p_this->state == UART_DMA_NOT_INITIALIZED || p_this->state == UART_DMA_NOT_POWERED)
    {
        return ARM_DRIVER_ERROR;
    }

    proj_assert(data != NULL);

    /* Send using DMA with the given UART HAL function */
    HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&p_this->base.uart_handle, (uint8_t*)data, (uint16_t)num);

    /* Return execution code according to the UART HAL execution code */
    switch (res)
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
 * Description    : Start receive data with UART DMA.
 * Input          : p_this - Pointer to self
                    data   - Pointer to buffer with data to receive
                    num    - Number of data items to receive
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_receive(uart_dma_resources_t* p_this, void* data, uint32_t num)
{
    if (p_this->state == UART_DMA_NOT_INITIALIZED || p_this->state == UART_DMA_NOT_POWERED)
    {
        return ARM_DRIVER_ERROR;
    }

    HAL_UART_AbortReceive(&p_this->base.uart_handle);

    proj_assert(data != NULL);

    HAL_StatusTypeDef res = HAL_UART_Receive_DMA(&p_this->base.uart_handle, (uint8_t*)data, (uint16_t)num);
    if (res != HAL_OK)
    {
        goto exit;
    }

    p_this->rx_count = 0;

    /* Enable interrupt that signals end of data. This event is used to handle receiving timeout.
       Must be enabled after calling HAL_UART_Receive_IT to avoid IDLE interrupt handle execution before update of new
       receive param */
    __HAL_UART_CLEAR_IDLEFLAG(&p_this->base.uart_handle);
    __HAL_UART_ENABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);

exit:

    switch (res)
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
 * Description    : Start receive data with UART DMA in streaming mode
 * Input          : p_this - Pointer to self
                    writer - Pointer to ring buffer writer
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_stream(uart_dma_resources_t* p_this, ring_buff_writer_t* writer)
{
    if (p_this->state == UART_DMA_NOT_INITIALIZED || p_this->state == UART_DMA_NOT_POWERED)
    {
        return ARM_DRIVER_ERROR;
    }

    HAL_UART_AbortReceive(&p_this->base.uart_handle);

    proj_assert(writer != NULL);

    /* Save pointer to writer */
    p_this->writer = *writer;

    /* Receive using DMA with the given UART HAL function */
    HAL_StatusTypeDef res = HAL_UART_Receive_DMA(
        &p_this->base.uart_handle, (uint8_t*)p_this->writer.ring_buff->buff, (uint16_t)p_this->writer.ring_buff->size);
    if (res != HAL_OK)
    {
        goto exit;
    }

    /* Enable interrupt that signals end of data. This event is used to handle receiving timeout. */
    __HAL_UART_CLEAR_IDLEFLAG(&p_this->base.uart_handle);
    __HAL_UART_ENABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);

exit:

    switch (res)
    {
        case HAL_OK:
        {
            return ARM_DRIVER_OK;
        }
        case HAL_ERROR:
        {
            return ARM_DRIVER_ERROR;
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
            return ARM_DRIVER_OK;
        }
    }
}

/***********************************************************************************************************************
 * Description    : Start sending/receiving data to/from UART DMA transmitter/receiver
 * Input          : p_this   - Pointer to self
                    data_out - Pointer to buffer with data to send
                    data_in  - Pointer to buffer for data to receive
                    num      - Number of data items to transfer
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_transfer(uart_dma_resources_t* p_this, const void* data_out, void* data_in, uint32_t num)
{
    ARGUMENT_UNUSED(data_out);
    ARGUMENT_UNUSED(data_in);
    ARGUMENT_UNUSED(num);
    ARGUMENT_UNUSED(p_this);
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description    : Get transmitted data count.
 * Input          : p_this - Pointer to self
 * Return         : number of bytes transmitted
 **********************************************************************************************************************/
uint32_t uart_dma_get_tx_count(uart_dma_resources_t* p_this)
{
    return p_this->base.uart_handle.TxXferCount;
}

/***********************************************************************************************************************
 * Description    : Get received data count.
 * Input          : p_this - Pointer to interface resources
 * Return         : number of bytes received
 **********************************************************************************************************************/
uint32_t uart_dma_get_rx_count(uart_dma_resources_t* p_this)
{
    return p_this->rx_count;
}

/***********************************************************************************************************************
 * Description    : Control UART DMA Interface
 * Input          : p_this  - Pointer to self
                    control - Operation
                    arg     - Argument of operation (optional)
 * Return         : execution status
 **********************************************************************************************************************/
int32_t uart_dma_control(uart_dma_resources_t* p_this, uint32_t control, uint32_t arg)
{
    control &= ARM_USART_CONTROL_Msk;

    if (p_this->state != UART_DMA_NOT_INITIALIZED && control != ARM_USART_MODE_ASYNCHRONOUS)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        /* In case we want to set the baudrate in asynchronous mode */
        case ARM_USART_MODE_ASYNCHRONOUS:
        {
            /* Baudrate change is currently only used by stm32f1 projects and isn't supported for other families */
            LL_RCC_ClocksTypeDef rcc_clocks;
            uint32_t             freq = 0;
            LL_RCC_GetSystemClocksFreq(&rcc_clocks);
            if (p_this->p_user_conf->instance == USART1)
            {
                freq = rcc_clocks.PCLK2_Frequency;
            }
            else
            {
                freq = rcc_clocks.PCLK1_Frequency;
            }

#if defined(STM32F1)
            LL_USART_SetBaudRate(p_this->p_user_conf->instance, freq, arg);
#elif defined(STM32F4) || defined(STM32L4)
            LL_USART_SetBaudRate(p_this->p_user_conf->instance, freq, p_this->p_user_conf->oversampling, arg);
#elif defined(STM32WL) || defined(STM32U5)
            LL_USART_SetBaudRate(p_this->p_user_conf->instance,
                                 freq,
                                 p_this->p_user_conf->prescaler,
                                 p_this->p_user_conf->oversampling,
                                 arg);
#endif

            return ARM_DRIVER_OK;
        }
        case ARM_USART_ABORT_RECEIVE:
        {
            if (HAL_OK != HAL_UART_DMAStop(&p_this->base.uart_handle))
            {
                return ARM_DRIVER_ERROR;
            }

            return ARM_DRIVER_OK;
        }
        case ARM_USART_ABORT_SEND:
        {
            if (HAL_OK != HAL_UART_DMAStop(&p_this->base.uart_handle))
            {
                return ARM_DRIVER_ERROR;
            }

            return ARM_DRIVER_OK;
        }
        case ARM_USART_ABORT_TRANSFER:
        {
            if (HAL_OK != HAL_UART_DMAStop(&p_this->base.uart_handle))
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
 * Description    : Transfer from HAL_DMA_StateTypeDef to ARM_USART_STATUS
 * Input          : p_this - Pointer to self
 * Return         : USART status
 **********************************************************************************************************************/
ARM_USART_STATUS uart_dma_get_status(uart_dma_resources_t* p_this)
{
    HAL_UART_StateTypeDef state = HAL_UART_GetState(&p_this->base.uart_handle);

    /* Configure status according to UART handler state */
    switch (state)
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

    /* In case there is a parity erro */
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
 * Description    : UART DMA IRQHandler private implementation
 * Input          : p_this - Pointer to interface resources
 * Return         : None
 **********************************************************************************************************************/
void uart_dma_irq_handler(uart_dma_resources_t* p_this)
{
    bool is_transmit_cplt = __HAL_UART_GET_FLAG(&p_this->base.uart_handle, UART_FLAG_TC);
    /* Transmit complete interrupt has occurred. We want to handle it first */
    if (is_transmit_cplt)
    {
        HAL_UART_IRQHandler(&p_this->base.uart_handle);
    }

    bool is_timeout = __HAL_UART_GET_FLAG(&p_this->base.uart_handle, UART_FLAG_IDLE);
    if (is_timeout)
    {
        receive_complete(p_this);

        if (p_this->callback != NULL)
        {
            p_this->callback(ARM_USART_EVENT_RX_TIMEOUT);
        }
    }
    else if (!is_transmit_cplt)
    {
        HAL_UART_IRQHandler(&p_this->base.uart_handle);
    }
}

/**********************************************************************************************************************/
/* Private Functions Implementations */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Receive complete
 * Input          : p_this - Pointer to interface resources
 * Return         : None
 **********************************************************************************************************************/
static void receive_complete(uart_dma_resources_t* p_this)
{
    if (p_this->p_conf->dma_rx_handle->Init.Mode == DMA_CIRCULAR)
    {
        uint32_t transfer_count  = dma_transfer_count(p_this);
        uint32_t committed_bytes = ring_buff_commit(&p_this->writer, transfer_count);

        if (committed_bytes != transfer_count)
        {
            /* The DMA has overwritten unread bytes in the DMA buffer - reset required */
            proj_exception();
        }
    }
    else
    {
        uint32_t transfer_count =
            p_this->base.uart_handle.RxXferSize - __HAL_DMA_GET_COUNTER(p_this->p_conf->dma_rx_handle);
        if (transfer_count != 0)
        {
            p_this->rx_count = transfer_count;
            /* Disable timeout flag since receive is completed */
            __HAL_UART_DISABLE_IT(&p_this->base.uart_handle, UART_IT_IDLE);
            HAL_UART_AbortReceive(&p_this->base.uart_handle);
        }
    }
    __HAL_UART_CLEAR_IDLEFLAG(&p_this->base.uart_handle);
}

/***********************************************************************************************************************
 * Description    : Uart dma transfer count
 * Input          : p_this - Pointer to interface resources
 * Return         : Transfer count
 **********************************************************************************************************************/
static uint32_t dma_transfer_count(uart_dma_resources_t* p_this)
{
    return p_this->writer.ring_buff->size - p_this->writer.ring_buff->tail
           - __HAL_DMA_GET_COUNTER(p_this->p_conf->dma_rx_handle);
}

/**********************************************************************************************************************/
/* Protected Function Implementations                                                                                 */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Tx transfer completed callback.
 * Input      : p_base - pointer base
 * Return     : None
 **********************************************************************************************************************/
static void tx_complete(uart_resources_t* p_base)
{
    uart_dma_resources_t* p_this = (uart_dma_resources_t*)p_base;

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
static void rx_complete(uart_resources_t* p_base)
{
    uart_dma_resources_t* p_this = (uart_dma_resources_t*)p_base;

    receive_complete(p_this);

    if (p_this->callback != NULL)
    {
        p_this->callback(ARM_USART_EVENT_RECEIVE_COMPLETE);
    }
}

/***********************************************************************************************************************
 * Description: Rx half transfer completed callback.
 * Input      : p_base - pointer base
 * Return     : None
 **********************************************************************************************************************/
static void rx_half_complete(uart_resources_t* p_base)
{
    uart_dma_resources_t* p_this = (uart_dma_resources_t*)p_base;

    if (p_this->p_conf->dma_rx_handle->Init.Mode == DMA_CIRCULAR)
    {
        uint32_t transfer_count  = dma_transfer_count(p_this);
        uint32_t committed_bytes = ring_buff_commit(&p_this->writer, transfer_count);

        if (committed_bytes != transfer_count)
        {
            /* The DMA has overwritten unread bytes in the DMA buffer - reset required */
            proj_exception();
        }

        if (p_this->callback != NULL)
        {
            p_this->callback(ARM_USART_EVENT_RECEIVE_COMPLETE);
        }
    }
}

/***********************************************************************************************************************
 * Description: Error callback.
 * Input      : p_base - pointer base
 * Return     : None
 **********************************************************************************************************************/
static void error(uart_resources_t* p_base)
{
    uart_dma_resources_t* p_this = (uart_dma_resources_t*)p_base;

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

    if (p_this->callback != NULL)
    {
        p_this->callback(event);
    }
}

/**********************************************************************************************************************/
/* Virtual Table                                                                                                      */
/**********************************************************************************************************************/

uart_methods_t const uart_dma_methods = {
    .TxComplete     = tx_complete,
    .RxComplete     = rx_complete,
    .RxHalfComplete = rx_half_complete,
    .Error          = error,
};