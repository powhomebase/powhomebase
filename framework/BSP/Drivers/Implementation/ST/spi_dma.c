/***********************************************************************************************************************
 * Description : STM32 SPI DMA driver implementation.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "spi_dma.h"

/* Related Project's headers */
#include <proj_assert.h>
#include <syscalls.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* Error value for prescaler calculation */
#define SPI_PRESCALER_ERROR (-1)

#define IS_SPI_INITIALIZED(_this) (((_this->state) & SPI_NOT_POWERED) == SPI_NOT_POWERED)
#define IS_SPI_POWERED(_this)     (((_this->state) & SPI_POWERED) == SPI_POWERED)

#define IS_SPI_BUSY(_this) (IS_SPI_BUSY_RX(_this) || IS_SPI_BUSY_TX(_this) || IS_SPI_BUSY_TX_RX(_this))
#define IS_SPI_BUSY_RX(_this)                                                                                          \
    ((HAL_SPI_GetState(&(_this)->spi_handle) & HAL_SPI_STATE_BUSY_RX) == HAL_SPI_STATE_BUSY_RX)
#define IS_SPI_BUSY_TX(_this)                                                                                          \
    ((HAL_SPI_GetState(&(_this)->spi_handle) & HAL_SPI_STATE_BUSY_TX) == HAL_SPI_STATE_BUSY_TX)
#define IS_SPI_BUSY_TX_RX(_this)                                                                                       \
    ((HAL_SPI_GetState(&(_this)->spi_handle) & HAL_SPI_STATE_BUSY_TX_RX) == HAL_SPI_STATE_BUSY_TX_RX)

/**********************************************************************************************************************/
/* Private functions declaration                                                                                      */
/**********************************************************************************************************************/

static int32_t get_prescaler(spi_dma_resources_t *p_this);
static void    transfer_complete(spi_dma_resources_t *p_this);

/**********************************************************************************************************************/
/* Implementation                                                                                                     */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description : Initialize SPI interface
 * Input       : p_this - Pointer to interface resources
 *               cb     - Pointer to callback function
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_Initialize(spi_dma_resources_t *p_this, ARM_SPI_SignalEvent_t cb)
{
    if (p_this->state != SPI_NOT_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    int32_t prescaler = get_prescaler(p_this);

    if (prescaler == SPI_PRESCALER_ERROR)
    {
        return ARM_DRIVER_ERROR;
    }

    p_this->spi_handle.Instance               = p_this->user_conf->instance;
    p_this->spi_handle.Init                   = *(p_this->user_conf->init);
    p_this->spi_handle.Init.BaudRatePrescaler = prescaler;

    p_this->spi_handle.hdmatx = p_this->dma_tx_hal_st;
    p_this->spi_handle.hdmarx = p_this->dma_rx_hal_st;

    p_this->callback = cb;

    gpio_control(p_this->gpios->clk, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->clk_conf);
    gpio_control(
        p_this->gpios->clk, ARM_GPIO_SET_AF, ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));

    gpio_control(p_this->gpios->cs, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->cs_conf);

    /* Set CS as AF only if it isn't driven by software */
    if (p_this->spi_handle.Init.NSS != SPI_NSS_SOFT)
    {
        gpio_control(p_this->gpios->cs,
                     ARM_GPIO_SET_AF,
                     ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));
    }

    gpio_control(p_this->gpios->miso, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->miso_conf);
    gpio_control(
        p_this->gpios->miso, ARM_GPIO_SET_AF, ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));

    gpio_control(p_this->gpios->mosi, ARM_GPIO_CONFIGURE_STATE, p_this->gpios->mosi_conf);
    gpio_control(
        p_this->gpios->mosi, ARM_GPIO_SET_AF, ARM_GPIO_AF_MAKE(p_this->gpios->alternate_f, p_this->gpios->alternate_f));

    /* Check 'full transfers' configuration */
    if (p_this->user_conf->enforce_full_transfers)
    {
        proj_assert(p_this->spi_handle.Init.NSS == SPI_NSS_HARD_INPUT);
        p_this->cs_active = false;
    }

    /* Initialize DMAs and register SPI IRQ as a callback function to the DMAs lower layer */
    p_this->dma_tx->Initialize(NULL);
    p_this->dma_rx->Initialize(NULL);

    /* Enable SPI clock to initialize SPI */
    p_this->clock_enable();

    HAL_SPI_Init(&(p_this->spi_handle));

    p_this->clock_disable();

    gpio_control(p_this->gpios->clk, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->gpios->miso, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->gpios->mosi, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(p_this->gpios->cs, ARM_GPIO_SET_STATE, CLOSE);

    p_this->state = SPI_NOT_POWERED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : De-initialize SPI interface
 * Input       : p_this - Pointer to interface resources
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_Uninitialize(spi_dma_resources_t *p_this)
{
    proj_assert(p_this->state == SPI_NOT_POWERED);

    HAL_SPI_DeInit(&p_this->spi_handle);

    p_this->dma_tx->Uninitialize();
    p_this->dma_rx->Uninitialize();

    p_this->state = SPI_NOT_INITIALIZED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : Control SPI Interface Power
 * Input       : p_this - Pointer to interface resources
 *               state  - Power state
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_PowerControl(spi_dma_resources_t *p_this, ARM_POWER_STATE state)
{
    if (!IS_SPI_INITIALIZED(p_this))
    {
        return ARM_DRIVER_ERROR;
    }

    if (IS_SPI_BUSY(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            /* Double ARM_POWER_xx is allowed to happen in cmsis and must return OK */
            if (p_this->state == SPI_NOT_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            p_this->state = SPI_NOT_POWERED;

            p_this->clock_disable();

            p_this->dma_tx->PowerControl(ARM_POWER_OFF);
            p_this->dma_rx->PowerControl(ARM_POWER_OFF);

            gpio_control(p_this->gpios->clk, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(p_this->gpios->miso, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(p_this->gpios->mosi, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(p_this->gpios->cs, ARM_GPIO_SET_STATE, CLOSE);

            return ARM_DRIVER_OK;
        }

        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        case ARM_POWER_FULL:
        {
            /* Double ARM_POWER_xx is allowed to happen in cmsis and must return OK */
            if (p_this->state == SPI_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            gpio_control(p_this->gpios->mosi, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(p_this->gpios->miso, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(p_this->gpios->clk, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(p_this->gpios->cs, ARM_GPIO_SET_STATE, OPEN);

            /* Set CS interrupt in 'full transfers' configuration */
            if (p_this->user_conf->enforce_full_transfers)
            {
                gpio_control(p_this->gpios->cs, ARM_GPIO_SET_INT_HANDLER, (uint32_t)p_this->cs_interrupt_cb);
                gpio_control(p_this->gpios->cs, ARM_GPIO_SET_INT_TRIGGER, ARM_GPIO_INT_TRIGGER_FALLING);
            }

            /* Enable DMAs */
            p_this->dma_tx->PowerControl(ARM_POWER_FULL);
            p_this->dma_rx->PowerControl(ARM_POWER_FULL);

            p_this->state = SPI_POWERED;

            p_this->clock_enable();

            return ARM_DRIVER_OK;
        }
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}

/***********************************************************************************************************************
 * Description : Start sending data to SPI transmitter.
 * Input       : p_this - Pointer to interface resources
 *               data   - Pointer to buffer with data to send
 *               num    - Number of data items to send
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_Send(spi_dma_resources_t* p_this, const void* data, uint32_t num)
{
    if (num == 0)
    {
        proj_assert(false);

        return ARM_DRIVER_OK;
    }

    if (!IS_SPI_POWERED(p_this))
    {
        return ARM_DRIVER_ERROR;
    }

    if (IS_SPI_BUSY(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    proj_assert(data != NULL);
    /* HAL_SPI_Transmit_DMA support only uint16_t num, so we must check this to avoid overflow */
    proj_assert(num < (uint32_t)UINT16_MAX);

    /* Send using DMA with the given SPI HAL function */
    HAL_StatusTypeDef res = HAL_SPI_Transmit_DMA(&(p_this->spi_handle), (uint8_t *)data, (uint16_t)num);

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
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
 * Description : Start receive data with SPI
 * Input       : p_this - Pointer to interface resources
 *               data   - Pointer to buffer with data to receive
 *               num    - Number of data items to receive
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_Receive(spi_dma_resources_t* p_this, void* data, uint32_t num)
{
    if (num == 0)
    {
        proj_assert(false);

        return ARM_DRIVER_OK;
    }

    if (!IS_SPI_POWERED(p_this))
    {
        return ARM_DRIVER_ERROR;
    }

    if (IS_SPI_BUSY(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    proj_assert(data != NULL);
    /* HAL_SPI_Transmit_DMA support only uint16_t num, so we must check this to avoid overflow */
    proj_assert(num < (uint32_t)UINT16_MAX);

    /* Flush receive queue before starting transfer */
#if defined(STM32L4) || defined(STM32WL) || defined(STM32U5) //FIXME: is necessary?
    HAL_SPIEx_FlushRxFifo(&p_this->spi_handle);
#endif

    /* Receive using DMA with the given SPI HAL function */
    HAL_StatusTypeDef res = HAL_SPI_Receive_DMA(&(p_this->spi_handle), (uint8_t *)data, (uint16_t)num);

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
 * Description : Start sending/receiving data to/from SPI transmitter/receiver
 * Input       : p_this   - Pointer to interface resources
 *               data_out - Pointer to buffer with data to send
 *               data_in  - Pointer to buffer for data to receive
 *               num      - Number of data items to transfer
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_Transfer(spi_dma_resources_t* p_this, const void* data_out, void* data_in, uint32_t num)
{
    if (num == 0)
    {
        proj_assert(false);

        return ARM_DRIVER_OK;
    }

    if (!IS_SPI_INITIALIZED(p_this))
    {
        return ARM_DRIVER_ERROR;
    }

    if (IS_SPI_BUSY(p_this))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    proj_assert(data_out != NULL);
    proj_assert(data_in != NULL);

    /* HAL_SPI_TransmitReceive_DMA support only uint16_t num, so we must check this to avoid overflow */
    proj_assert(num < (uint32_t)UINT16_MAX);

    /* Flush receive queue before starting transfer */
#if defined(STM32L4) || defined(STM32WL)  || defined(STM32U5) //FIXME: is necessary?
    HAL_SPIEx_FlushRxFifo(&p_this->spi_handle);
#endif

    HAL_StatusTypeDef res =
        HAL_SPI_TransmitReceive_DMA(&(p_this->spi_handle), (uint8_t *)data_out, data_in, (uint16_t)num);

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
        /* In case of error not aforementioned */
        default:
        {
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
 * Description : Get transferred data count
 * Input       : p_this - Pointer to interface resources
 * Return      : number of data items transmitted
 **********************************************************************************************************************/
uint32_t spi_dma_GetDataCount(spi_dma_resources_t *p_this)
{
    ARGUMENT_UNUSED(p_this);

    /* This is unsupported but returns uint (ARM_DRIVER_UNSUPPORTED is int) so we return 0 */
    return 0;
}

/***********************************************************************************************************************
 * Description : Control SPI Interface
 * Input       : p_this  - Pointer to interface resources
 *               control - Operation
 *               arg     - Argument of operation (optional)
 * Return      : execution status
 **********************************************************************************************************************/
int32_t spi_dma_Control(spi_dma_resources_t *p_this, uint32_t control, uint32_t arg)
{
    switch (control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_CONTROL_SS:
        {
            /* We shouldn't toggle the CS by software if it is configured to be toggled by hardware */
            proj_assert(p_this->user_conf->init->NSS == SPI_NSS_SOFT);

            /* Control slave select when it is configured as software control */
            if (arg == ARM_SPI_SS_INACTIVE)
            {
                gpio_write(p_this->gpios->cs, HIGH);
            }

            if (arg == ARM_SPI_SS_ACTIVE)
            {
                gpio_write(p_this->gpios->cs, LOW);
            }

            break;
        }
        case ARM_SPI_ABORT_TRANSFER:
        {
            /* Ignore the operation if driver wasn't busy */
            if (!IS_SPI_BUSY(p_this))
            {
                break;
            }

            /* Call HAL to stop dma */
            /* NOTE: The return value of this function is not indicative as it returns
                     HAL_OK only if both RX and TX were busy, so we don't even check it. */
            HAL_SPI_DMAStop(&p_this->spi_handle);

            break;
        }
        default:
        {
            return ARM_DRIVER_ERROR;
        }
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : Transfer from HAL_DMA_StateTypeDef to ARM_SPI_STATUS
 * Input       : p_this - Pointer to interface resources
 * Return      : SPI status
 **********************************************************************************************************************/
ARM_SPI_STATUS spi_dma_GetStatus(spi_dma_resources_t *p_this)
{
    p_this->status.busy = IS_SPI_BUSY(p_this);

    /* In case there is an overrun error */
    if (p_this->spi_handle.ErrorCode & HAL_SPI_ERROR_OVR)
    {
        p_this->status.data_lost = true;
    }

    /* In case there is a mode fault error */
    if (p_this->spi_handle.ErrorCode & HAL_SPI_ERROR_MODF)
    {
        p_this->status.mode_fault = true;
    }

    return p_this->status;
}

/***********************************************************************************************************************
 * Description : Callback of CS pin, to handle the 'full transfers' configuration.
 * Input       : p_this - Pointer to interface resources
 ***********************************************************************************************************************/
void spi_dma_cs_interrupt_callback(spi_dma_resources_t *p_this)
{
    /* Ignore noises */
    if (!IS_SPI_BUSY(p_this))
    {
        return;
    }

    /* Standard case, where CS wasn't triggered */
    if (!p_this->cs_active)
    {
        p_this->cs_active = true;
        return;
    }

    /* Abort the DMA transfer */
    HAL_SPI_DMAStop(&p_this->spi_handle);

    /* Update status & state */
    p_this->status.data_lost = true;
    p_this->state            = SPI_POWERED;

    /* Update CS state */
    p_this->cs_active = false;

    if (p_this->callback != NULL)
    {
        p_this->callback(ARM_SPI_EVENT_DATA_LOST);
    }
}

/***********************************************************************************************************************
 * Private functions implementation
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Description : Common handler for Tx/Rx/TxRx complete callbacks.
 * Input       : p_this - Pointer to interface resources
 ***********************************************************************************************************************/
static void transfer_complete(spi_dma_resources_t *p_this)
{
    p_this->status.data_lost = false;
    p_this->state            = SPI_POWERED;

    /* Handle 'full transfers' configuration */
    if (p_this->user_conf->enforce_full_transfers)
    {
        p_this->cs_active = false;
    }

    if (p_this->callback != NULL)
    {
        p_this->callback(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }
}

/***********************************************************************************************************************
 * Description : Determine prescaler based on current CLK and Max CLK
 * Input       : p_this       - pointer to the driver's resources.
 * Return      : ARM's SPI baudrate or DRIVER_ERROR
 ***********************************************************************************************************************/
static int32_t get_prescaler(spi_dma_resources_t *p_this)
{
    uint32_t periph_clk = p_this->get_clk_freq_();

    double prescaler = (double)periph_clk / p_this->user_conf->max_allowed_baudrate_hz;

    if (prescaler <= 2)
    {
        return SPI_BAUDRATEPRESCALER_2;
    }
    else if (prescaler <= 4)
    {
        return SPI_BAUDRATEPRESCALER_4;
    }
    else if (prescaler <= 8)
    {
        return SPI_BAUDRATEPRESCALER_8;
    }
    else if (prescaler <= 16)
    {
        return SPI_BAUDRATEPRESCALER_16;
    }
    else if (prescaler <= 32)
    {
        return SPI_BAUDRATEPRESCALER_32;
    }
    else if (prescaler <= 64)
    {
        return SPI_BAUDRATEPRESCALER_64;
    }
    else if (prescaler <= 128)
    {
        return SPI_BAUDRATEPRESCALER_128;
    }
    else if (prescaler <= 256)
    {
        return SPI_BAUDRATEPRESCALER_256;
    }

    return SPI_PRESCALER_ERROR;
}

/**********************************************************************************************************************/
/* HAL weak functions implementation                                                                                  */
/**********************************************************************************************************************/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_dma_resources_t *p_this = (spi_dma_resources_t *)hspi;

    transfer_complete(p_this);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_dma_resources_t *p_this = (spi_dma_resources_t *)hspi;

    transfer_complete(p_this);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_dma_resources_t *p_this = (spi_dma_resources_t *)hspi;

    transfer_complete(p_this);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    spi_dma_resources_t *p_this = (spi_dma_resources_t *)hspi;

    if (p_this->callback != NULL)
    {
        p_this->callback(ARM_SPI_EVENT_DATA_LOST);
    }
}