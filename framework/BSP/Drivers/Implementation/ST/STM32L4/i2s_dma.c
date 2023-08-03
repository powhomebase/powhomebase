/***********************************************************************************************************************
 * Description: STM32L4 I2S (SAI).
 *              I2S using DMA interafce implementation for ST STM32L4.
 *              To use this you should:
 *                  - generate 4 GPIOs: I2S1_SCK, I2S1_SD, I2S1_FS, I2S1_MCLK (master clk is optional, but must
 *                    have this object initialized. you can enable this feature in the sai user config object)
 *                  - generate DMA channel, and put its CMSIS number in the sai CMSIS object generator
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "i2s_dma.h"

/* Project related headers */
#include <proj_exception.h>
#include <syscalls.h>

/**********************************************************************************************************************/
/* Private Function Declarations                                                                                      */
/**********************************************************************************************************************/

void sai_clock_enable(i2s_dma_resources_t* i2s_resources);
void sai_clock_disable(i2s_dma_resources_t* i2s_resources);
void handle_hal_callback(SAI_HandleTypeDef* hsai, uint32_t events);

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Flag indicating whether the SAI clocks and PLLs are configured and enabled */
bool is_sai_clocks_enabled = false;

/**********************************************************************************************************************/
/* Public Functions Implementations                                                                                   */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Initialize I2S interface
 * Input      : cb_event      - pointer to callback function
                i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t i2s_dma_Initialize(ARM_SAI_SignalEvent_t cb_event, i2s_dma_resources_t* i2s_resources)
{
    /* Check if the driver has already been initialized */
    if (i2s_resources->state != SAI_I2S_NOT_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    /* Set I2S instance address */
    i2s_resources->i2s_handle.Instance = i2s_resources->user_conf->instance;

    /* Set I2S init object */
    i2s_resources->i2s_handle.Init = *(i2s_resources->user_conf->init);

    if (i2s_resources->user_conf->init->AudioMode == SAI_MODEMASTER_RX
        || i2s_resources->user_conf->init->AudioMode == SAI_MODESLAVE_RX)
    {
        i2s_resources->transmitDirection = RX;
        i2s_resources->i2s_handle.hdmarx = i2s_resources->hdma;
    }
    else
    {
        i2s_resources->transmitDirection = TX;
        i2s_resources->i2s_handle.hdmatx = i2s_resources->hdma;
    }

    /* Set callback function in driver resources */
    i2s_resources->callback = cb_event;

    /* Configure GPIOs */
    gpio_control(
        i2s_resources->gpios->sd,
        ARM_GPIO_CONFIGURE_STATE,
        ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_PULL_DISABLED, ARM_GPIO_DISABLED));

    gpio_control(i2s_resources->gpios->sd,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(i2s_resources->gpios->alternate_f, i2s_resources->gpios->alternate_f));

    gpio_control(
        i2s_resources->gpios->sck,
        ARM_GPIO_CONFIGURE_STATE,
        ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_PULL_DISABLED, ARM_GPIO_DISABLED));

    gpio_control(i2s_resources->gpios->sck,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(i2s_resources->gpios->alternate_f, i2s_resources->gpios->alternate_f));

    gpio_control(
        i2s_resources->gpios->fs,
        ARM_GPIO_CONFIGURE_STATE,
        ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_PULL_DISABLED, ARM_GPIO_DISABLED));

    gpio_control(i2s_resources->gpios->fs,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(i2s_resources->gpios->alternate_f, i2s_resources->gpios->alternate_f));

    if (i2s_resources->user_conf->enable_master_clk)
    {
        gpio_control(i2s_resources->gpios->mclk,
                     ARM_GPIO_CONFIGURE_STATE,
                     ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_PULL_DISABLED,
                                        ARM_GPIO_DISABLED));

        gpio_control(i2s_resources->gpios->mclk,
                     ARM_GPIO_SET_AF,
                     ARM_GPIO_AF_MAKE(i2s_resources->gpios->alternate_f, i2s_resources->gpios->alternate_f));
    }

    /* Open GPIOs */
    gpio_control(i2s_resources->gpios->sd, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(i2s_resources->gpios->sck, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(i2s_resources->gpios->fs, ARM_GPIO_SET_STATE, CLOSE);
    if (i2s_resources->user_conf->enable_master_clk)
    {
        gpio_control(i2s_resources->gpios->mclk, ARM_GPIO_SET_STATE, CLOSE);
    }

    /* Initialzie Rx DMA channel */
    if (i2s_resources->dma->Initialize(NULL) != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Enable peripheral clock */
    sai_clock_enable(i2s_resources);

    /* Init I2S. */
    if (HAL_SAI_InitProtocol(&i2s_resources->i2s_handle,
                             i2s_resources->user_conf->protocol,
                             i2s_resources->user_conf->dataSize,
                             i2s_resources->user_conf->num_of_slots)
        != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Disable peripheral clock */
    sai_clock_disable(i2s_resources);

    // FIXME: implement low power
    /* Register driver to low power manager */
    // i2s_resources->lpm.module_id = low_power_register(0);

    // FIXME: implement low power
    /* Set power mode to lowest */
    // low_power_update(i2s_resources->lpm.module_id, POWER_MODE_LOWEST);

    /* Set driver state to I2S_INITIALIZED */
    i2s_resources->state = SAI_I2S_INITIALIZED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: De-initialize I2S interface
 * Input      : i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t i2s_dma_Uninitialize(i2s_dma_resources_t* i2s_resources)
{
    ARGUMENT_UNUSED(i2s_resources);
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description: Control I2S Interface Power
 * Input      : state - power state
 *              i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t i2s_dma_PowerControl(ARM_POWER_STATE state, i2s_dma_resources_t* i2s_resources)
{
    /* In case the driver has not yet been initialized */
    if (i2s_resources->state == SAI_I2S_NOT_INITIALIZED)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        /* Turn off driver */
        case ARM_POWER_OFF:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (i2s_resources->state == SAI_I2S_INITIALIZED)
            {
                return ARM_DRIVER_OK;
            }

            if (HAL_SAI_DMAStop(&i2s_resources->i2s_handle) != HAL_OK)
            {
                return ARM_DRIVER_ERROR;
            }

            /* Open Rx DMA */
            if (i2s_resources->dma->PowerControl(ARM_POWER_OFF) != ARM_DRIVER_OK)
            {
                /* Return error code if an error occured when openning DMA */
                return ARM_DRIVER_ERROR;
            }

            /* Close GPIOs */
            gpio_control(i2s_resources->gpios->sd, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(i2s_resources->gpios->sck, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(i2s_resources->gpios->fs, ARM_GPIO_SET_STATE, CLOSE);
            if (i2s_resources->user_conf->enable_master_clk)
            {
                gpio_control(i2s_resources->gpios->mclk, ARM_GPIO_SET_STATE, CLOSE);
            }

            /* Disable peripheral clock */
            sai_clock_disable(i2s_resources);

            /* Set driver state to UART_INITIALIZED */
            i2s_resources->state = SAI_I2S_INITIALIZED;

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
            if (i2s_resources->state == SAI_I2S_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            /* Enable peripheral clock */
            sai_clock_enable(i2s_resources);

            /* Open GPIOs */
            gpio_control(i2s_resources->gpios->sd, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(i2s_resources->gpios->sck, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(i2s_resources->gpios->fs, ARM_GPIO_SET_STATE, OPEN);
            if (i2s_resources->user_conf->enable_master_clk)
                gpio_control(i2s_resources->gpios->mclk, ARM_GPIO_SET_STATE, OPEN);

            /* Open Rx DMA */
            if (i2s_resources->dma->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK)
                /* Return error code if an error occured when openning DMA */
                return ARM_DRIVER_ERROR;

            /* Set driver state to SAI_I2S_POWERED */
            i2s_resources->state = SAI_I2S_POWERED;

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
 * Description:  Start sending data to I2S transmitter
 * Input      : data          - pointer to buffer with data to send
 *              num           - number of samples
 *              i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t i2s_dma_Send(const void* data, uint32_t num, i2s_dma_resources_t* i2s_resources)
{
    static void* send_buffer;

    /* Check if I2S has been powered before sending data */
    if (i2s_resources->state != SAI_I2S_POWERED || i2s_resources->transmitDirection == RX)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Return BUSY error status if the driver is busy from a previous send operation */
    if (i2s_resources->status.tx_busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Save pointer to data */
    send_buffer = (void*)data;

    // FIXME: implement low power
    /* Set power mode to active */
    // low_power_update(i2s_resources->lpm.module_id, i2s_resources->lpm.active_mode);

    if (HAL_SAI_Transmit_DMA(&i2s_resources->i2s_handle, send_buffer, num) != HAL_OK)
    {
        // FIXME: implement low power
        /* Set power mode to lowest */
        // low_power_update(i2s_resources->lpm.module_id, POWER_MODE_LOWEST);

        return ARM_DRIVER_ERROR;
    }

    /* Set busy flag true */
    i2s_resources->status.tx_busy = true;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Start receiving data from I2S receiver
 * Input      : data          - pointer to buffer with data to receive
 *              num           - number of samples
 *              i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t i2s_dma_Receive(void* data, uint32_t num, i2s_dma_resources_t* i2s_resources)
{
    static void* receive_buffer;

    /* Check if I2S has been powered before sending data */
    if (i2s_resources->state != SAI_I2S_POWERED || i2s_resources->transmitDirection == TX)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Return BUSY error status if the driver is busy from a previous send operation */
    if (i2s_resources->status.rx_busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Save pointer to data */
    receive_buffer = (void*)data;

    // FIXME: implement low power
    /* Set power mode to active */
    // low_power_update(i2s_resources->lpm.module_id, i2s_resources->lpm.active_mode);

    if (HAL_SAI_Receive_DMA(&i2s_resources->i2s_handle, receive_buffer, num) != HAL_OK)
    {
        // FIXME: implement low power
        /* Set power mode to lowest */
        // low_power_update(i2s_resources->lpm.module_id, POWER_MODE_LOWEST);

        return ARM_DRIVER_ERROR;
    }

    /* Set busy flag true */
    i2s_resources->status.rx_busy = true;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Get transmitted data count
 * Input      : i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
uint32_t i2s_dma_GetTxCount(i2s_dma_resources_t* i2s_resources)
{
    ARGUMENT_UNUSED(i2s_resources);
    return 0;
}

/***********************************************************************************************************************
 * Description: Get received data count
 * Input      : i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
uint32_t i2s_dma_GetRxCount(i2s_dma_resources_t* i2s_resources)
{
    ARGUMENT_UNUSED(i2s_resources);
    return 0;
}

/***********************************************************************************************************************
 * Description: Control I2S Interface
 * Input      : control       - operation
 *              arg1          - argument (optional)
 *              arg2          - argument (optional)
 *              i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t i2s_dma_Control(uint32_t control, uint32_t arg1, uint32_t arg2, i2s_dma_resources_t* i2s_resources)
{
    ARGUMENT_UNUSED(arg1);

    /* Find out which control was set */
    switch (control & ARM_SAI_CONTROL_Msk)
    {
        case ARM_SAI_CONFIGURE_RX:
        {
            /* check freq validity */
            if (arg2 & (~ARM_SAI_AUDIO_FREQ_Msk))
            {
                return ARM_DRIVER_ERROR;
            }

            /* de initialize SAI */
            if (HAL_SAI_DeInit(&i2s_resources->i2s_handle) != HAL_OK)
            {
                return ARM_DRIVER_ERROR;
            }

            /* change init obj audio freq field */
            i2s_resources->i2s_handle.Init.AudioFrequency = arg2;

            /* re-Init I2S. */
            if (HAL_SAI_InitProtocol(&i2s_resources->i2s_handle,
                                     i2s_resources->user_conf->protocol,
                                     i2s_resources->user_conf->dataSize,
                                     i2s_resources->user_conf->num_of_slots)
                != HAL_OK)
            {
                return ARM_DRIVER_ERROR;
            }

            break;
        }
        /* Control flag given is not a valid control parameter */
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Get I2S status
 * Input      : i2s_resources - pointer to interface resources
 * Return     : Execution status
 **********************************************************************************************************************/
ARM_SAI_STATUS i2s_dma_GetStatus(i2s_dma_resources_t* i2s_resources)
{
    return i2s_resources->status;
}

/**********************************************************************************************************************/
/* Private Functions Implementations                                                                                  */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Enable the SAI's PLL
 * Input      : i2s_resources - pointer to interface resources
 **********************************************************************************************************************/
void sai_clock_enable(i2s_dma_resources_t* i2s_resources)
{
    if (is_sai_clocks_enabled)
    {
        proj_exception();
    }

    /* set peripheral pll clock */
    if (HAL_OK != HAL_RCCEx_PeriphCLKConfig(i2s_resources->RCC_PeriphCLKInitStruct_sai))
    {
        proj_exception();
    }

    switch (i2s_resources->sai_num)
    {
#if defined(SAI1)
        case SAI1_e:
        {
            __HAL_RCC_SAI1_CLK_ENABLE();
            __HAL_RCC_SAI1_CLK_SLEEP_ENABLE();
            break;
        }
#endif
#if defined(SAI2)
        case SAI2_e:
        {
            __HAL_RCC_SAI2_CLK_ENABLE();
            __HAL_RCC_SAI2_CLK_SLEEP_ENABLE();
            break;
        }
#endif
        default:
        {
            /* Illegal value / selected SAI is not supported */
            proj_exception();
            break;
        }
    }

    is_sai_clocks_enabled = true;
}

/***********************************************************************************************************************
 * Description: Disable the SAI's PLL
 * Input      : i2s_resources - pointer to interface resources
 **********************************************************************************************************************/
void sai_clock_disable(i2s_dma_resources_t* i2s_resources)
{
    if (!is_sai_clocks_enabled)
    {
        proj_exception();
    }

    switch (i2s_resources->saipll_num)
    {
#if defined(RCC_PLLSAI1_SUPPORT)
        case PLL1_e:
        {
            if (HAL_OK != HAL_RCCEx_DisablePLLSAI1())
            {
                proj_exception();
            }
            break;
        }
#endif
#if defined(RCC_PLLSAI2_SUPPORT)
        case PLL2_e:
        {
            if (HAL_OK != HAL_RCCEx_DisablePLLSAI2())
            {
                proj_exception();
            }
            break;
        }
#endif
        default:
        {
            /* Illegal value / selected SAI is not supported */
            proj_exception();
            break;
        }
    }

    switch (i2s_resources->sai_num)
    {
#if defined(SAI1)
        case SAI1_e:
        {
            __HAL_RCC_SAI1_CLK_SLEEP_DISABLE();
            __HAL_RCC_SAI1_CLK_DISABLE();
            break;
        }
#endif
#if defined(SAI2)
        case SAI2_e:
        {
            __HAL_RCC_SAI2_CLK_SLEEP_DISABLE();
            __HAL_RCC_SAI2_CLK_DISABLE();
            break;
        }
#endif
        default:
        {
            /* Illegal value / selected SAI is not supported */
            proj_exception();
            break;
        }
    }

    is_sai_clocks_enabled = false;
}

/***********************************************************************************************************************
 * Description: handle HAL callback and raise event
 * Input      : hsai   - sai handle
 *              events - events to raise
 **********************************************************************************************************************/
void handle_hal_callback(SAI_HandleTypeDef* hsai, uint32_t events)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    i2s_dma_resources_t* p_i2s_resources = (i2s_dma_resources_t*)hsai;

    // FIXME: implement low power
    /* Set power mode to lowest */
    // low_power_update(p_i2s_resources->lpm.module_id, POWER_MODE_LOWEST);

    if (events & ARM_SAI_EVENT_SEND_COMPLETE || events & ARM_SAI_EVENT_FRAME_ERROR)
    {
        p_i2s_resources->status.tx_busy = false;
    }

    if (events & ARM_SAI_EVENT_RECEIVE_COMPLETE || events & ARM_SAI_EVENT_FRAME_ERROR)
    {
        p_i2s_resources->status.rx_busy = false;
    }

    if (p_i2s_resources->callback != NULL)
    {
        p_i2s_resources->callback(events);
    }
}

/**********************************************************************************************************************/
/* HAL weak functions implementation                                                                                  */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: HAL weak callback implementation for Tx complete.
 * Input      : hsai - SAI_HandleTypeDef, Pointer to the HAL handle object.
 **********************************************************************************************************************/
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai)
{
    handle_hal_callback(hsai, ARM_SAI_EVENT_SEND_COMPLETE);
}

/***********************************************************************************************************************
 * Description: HAL weak callback implementation for Tx half complete.
 * Input      : hsai - SAI_HandleTypeDef, Pointer to the HAL handle object.
 **********************************************************************************************************************/
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    // FIXME: Do we want to raise event of SEND_COMPLETE in TxHalfCplt?
    handle_hal_callback(hsai, ARM_SAI_EVENT_SEND_COMPLETE);
}

/***********************************************************************************************************************
 * Description: HAL weak callback implementation for Rx complete.
 * Input      : hsai - SAI_HandleTypeDef, Pointer to the HAL handle object.
 **********************************************************************************************************************/
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef* hsai)
{
    handle_hal_callback(hsai, ARM_SAI_EVENT_RECEIVE_COMPLETE);
}

/***********************************************************************************************************************
 * Description: HAL weak callback implementation for Rx half complete.
 * Input      : hsai - SAI_HandleTypeDef, Pointer to the HAL handle object.
 **********************************************************************************************************************/
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    handle_hal_callback(hsai, ARM_SAI_EVENT_RECEIVE_COMPLETE);
}


/***********************************************************************************************************************
 * Description: HAL weak callback implementation for error handling.
 * Input      : hsai - SAI_HandleTypeDef, Pointer to the HAL handle object.
 **********************************************************************************************************************/
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef* hsai)
{
    // FIXME: FIX ACCORDING TO TYPE OF ERROR
    handle_hal_callback(hsai, ARM_SAI_EVENT_FRAME_ERROR);
}
