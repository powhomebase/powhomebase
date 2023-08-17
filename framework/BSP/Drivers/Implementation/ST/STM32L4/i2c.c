/**********************************************************************************************************************/
/* Description        : STM32L4 i2c driver implementation.                                                            */
/**********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "i2c.h"

/* Related Project's headers */
#include <proj_assert.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* Default TIMINGR register values */
#define TIMINGR_EMPTY           (0x00000000)
#define PRESC_DEFAULT           (0)
#define SCLDEL_DEFAULT          (1)
#define SDADEL_DEFAULT          (1)
#define TSYNC_IN_CYCLES_DEFAULT (10)

/**********************************************************************************************************************/
/* Prototypes                                                                                                         */
/**********************************************************************************************************************/

/* Calculates timing register to match user's desired baudrate */
__STATIC_INLINE uint32_t i2c_calculate_timing_register(uint32_t bus_speed, uint32_t clk_freq);

/**********************************************************************************************************************/
/* CMSIS functions implementation                                                                                     */
/**********************************************************************************************************************/

/***********************************************************************************************************************
* Description    : Initialize I2C Interface
* Input          : ARM_I2C_SignalEvent  - pointer to callback event
                   i2c_resources_t*     - pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_Initialize(ARM_I2C_SignalEvent_t cb_event, i2c_resources_t* i2c_resources)
{
    /* Check if the driver has already been initialized */
    if (i2c_resources->state == I2C_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    /* Return error if trying to initialize a powered driver */
    if (i2c_resources->state == I2C_POWERED)
    {
        return ARM_DRIVER_OK;
    }

    /* Set callback function in driver resources */
    i2c_resources->callback = cb_event;

    /* Enable peripheral clock for configuration */
    i2c_resources->clock_enable();

    /* Configure SDA GPIO */
    gpio_control(i2c_resources->gpios->sda, ARM_GPIO_CONFIGURE_STATE, i2c_resources->gpios->sda_conf);
    gpio_control(i2c_resources->gpios->sda,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(i2c_resources->gpios->alternate_f, i2c_resources->gpios->alternate_f));

    /* Configure SCL GPIO */
    gpio_control(i2c_resources->gpios->scl, ARM_GPIO_CONFIGURE_STATE, i2c_resources->gpios->scl_conf);
    gpio_control(i2c_resources->gpios->scl,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(i2c_resources->gpios->alternate_f, i2c_resources->gpios->alternate_f));

    /* Close GPIOs */
    gpio_control(i2c_resources->gpios->scl, ARM_GPIO_SET_STATE, CLOSE);
    gpio_control(i2c_resources->gpios->sda, ARM_GPIO_SET_STATE, CLOSE);

    /* Set Master/Slave in I2C status*/
    i2c_resources->status.mode = i2c_resources->master;

    /* set init struct to handler*/
    i2c_resources->i2c_handler.Init = *(i2c_resources->user_confs->i2cInit);

    /*set instance */
    i2c_resources->i2c_handler.Instance = i2c_resources->instance;

    /* Calculate timing register */
    uint32_t timing_reg =
        i2c_calculate_timing_register(i2c_resources->user_confs->bus_speed, i2c_resources->rcc_periph_clk);

    if (timing_reg == TIMINGR_EMPTY)
    {
        return ARM_DRIVER_ERROR;
    }

    i2c_resources->i2c_handler.Init.Timing = timing_reg;

    /* Initialize I2C */
    if (HAL_I2C_Init(&i2c_resources->i2c_handler) != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Disable peripheral clock after configuration */
    i2c_resources->clock_disable();

    /* Set driver state to I2C_INITIALIZED */
    i2c_resources->state = I2C_INITIALIZED;

    /* Set status busy flag to NOT_BUSY */
    i2c_resources->status.busy = NOT_BUSY;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description    : De-initialize I2C Interface
 * Input          : i2c_resources_t*     - pointer to interface resources
 * Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_Uninitialize(i2c_resources_t* i2c_resources)
{
    i2c_resources->state = I2C_NOT_INITIALIZED;
    if (HAL_I2C_DeInit(&i2c_resources->i2c_handler) != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
* Description    : Control I2C Interface Power
* Input          : ARM_POWER_STATE      - power state
                   i2c_resources_t*     - pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_PowerControl(ARM_POWER_STATE state, i2c_resources_t* i2c_resources)
{
    switch (state)
    {
        /* Turn off I2C driver*/
        case ARM_POWER_OFF:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (i2c_resources->state == I2C_INITIALIZED)
            {
                return ARM_DRIVER_OK;
            }

            if (i2c_resources->state == I2C_NOT_INITIALIZED)
            {
                proj_assert(false);

                return ARM_DRIVER_ERROR;
            }

            /* Disable peripheral interrupts */
            NVIC_DisableIRQ(i2c_resources->irq);

            /* Disable peripheral clock */
            i2c_resources->clock_disable();

            /* Close GPIOs */
            gpio_control(i2c_resources->gpios->sda, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(i2c_resources->gpios->scl, ARM_GPIO_SET_STATE, CLOSE);

            /* Set driver state to I2C_INITIALIZED */
            i2c_resources->state = I2C_INITIALIZED;

            return ARM_DRIVER_OK;
        }

        /* Low power mode - NOT SUPPORTED */
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Turn on I2C driver */
        case ARM_POWER_FULL:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (i2c_resources->state == I2C_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            if (i2c_resources->state == I2C_NOT_INITIALIZED)
            {
                proj_assert(false);

                return ARM_DRIVER_ERROR;
            }

            /* Enable driver clock */
            i2c_resources->clock_enable();

            /* Open GPIOs */
            gpio_control(i2c_resources->gpios->sda, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(i2c_resources->gpios->scl, ARM_GPIO_SET_STATE, OPEN);

            while (gpio_read(i2c_resources->gpios->scl) == LOW
                   || gpio_read(i2c_resources->gpios->sda) == LOW)
                ;

            /* Enable interrupts in NVIC */
            NVIC_EnableIRQ(i2c_resources->irq);

            /* Set driver state to I2C_POWERED */
            i2c_resources->state = I2C_POWERED;

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
* Description    : Start transmitting data as I2C Master
* Input          : uint32_t adr         - Slave address (7-bit or 10-bit)
                   uint8_t *data        - Pointer to buffer with data to transmit to I2C Slave
                   uint32_t num         - Number of data bytes to transmit
                   bool xfer_pending    - Transfer operation is pending - Stop condition will not be generated
                   i2c_resources_t*     - Pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_MasterTransmit(uint32_t         adr,
                           const uint8_t*   data,
                           uint32_t         num,
                           bool             xfer_pending,
                           i2c_resources_t* i2c_resources)
{
    /* I2C HAL transmit execution status */
    HAL_StatusTypeDef hal_transmit_result;

    /* Check if the transmit does not send a stop bit and we are already sending one without a stop bit */
    if (i2c_resources->xfer_pending && xfer_pending)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Shift address left as HAL takes input of 7 bit address - with format AAAA AAAX. A=addr bit, X=ignoring bit */
    adr = adr << 1;

    /* Set I2C status */
    i2c_resources->status.busy      = BUSY;
    i2c_resources->status.direction = TRANSMITTING_DIR;

    /* Clear error status from previous operations */
    i2c_resources->status.arbitration_lost = false;
    i2c_resources->status.bus_error        = false;

    /* Check if we need to wait for the buffer in order to complete transfer */
    if (xfer_pending)
    {
        /* Saving slave address */
        i2c_resources->transfer_data.address = (uint16_t)adr;

        /* Saving first part of the data to transmit */
        i2c_resources->transfer_data.first_data_buf = (uint8_t*)data;

        /* Saving buffer length for first data buffer */
        i2c_resources->transfer_data.first_data_length = num;

        /* Set I2C resources xfer_pending flag true to signal for the next Transmit/Receive */
        i2c_resources->xfer_pending = true;

        return ARM_DRIVER_OK;
    }

    /* In case we get the second part of the data to send */
    if (i2c_resources->xfer_pending)
    {
        /* Saving first part of the data to transmit */
        i2c_resources->transfer_data.second_data_buf = (uint8_t*)data;

        /* Saving buffer length for first data buffer */
        i2c_resources->transfer_data.second_data_length = num;

        /* Set I2C resources xfer_pending flag true to signal for the next Transmit/Receive */
        i2c_resources->xfer_pending = false;

        /* Sending the data */
        hal_transmit_result = HAL_I2C_Mem_Write_IT(&i2c_resources->i2c_handler,
                                                   i2c_resources->transfer_data.address,
                                                   (*i2c_resources->transfer_data.first_data_buf),
                                                   I2C_MEMADD_SIZE_8BIT,
                                                   i2c_resources->transfer_data.second_data_buf,
                                                   i2c_resources->transfer_data.second_data_length);
    }

    /* In case a single data byte is to be sent */
    else
    {
        /* Saving slave address */
        i2c_resources->transfer_data.address = adr;

        /* Saving data to transmit */
        i2c_resources->transfer_data.first_data_buf = (uint8_t*)data;

        /* Saving buffer length */
        i2c_resources->transfer_data.first_data_length = num;

        /* Sending the data */
        hal_transmit_result = HAL_I2C_Master_Transmit_IT(&i2c_resources->i2c_handler,
                                                         i2c_resources->transfer_data.address,
                                                         i2c_resources->transfer_data.first_data_buf,
                                                         i2c_resources->transfer_data.first_data_length);
    }

    /* Return success code in case the operation began successfully */
    if (hal_transmit_result == HAL_OK)
    {
        return ARM_DRIVER_OK;
    }

    /* Return execution code according to the I2C HAL execution code */
    switch (hal_transmit_result)
    {
        /* In case the driver is busy */
        case HAL_BUSY:
        {
            return ARM_DRIVER_ERROR_BUSY;
        }
        /* In case there was a timeout */
        case HAL_TIMEOUT:
        {
            return ARM_DRIVER_ERROR_TIMEOUT;
        }
        /* In case of any error not aforementioned */
        default:
        {
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
* Description    : Start receiving data as I2C Master
* Input          : uint32_t adr         - Slave address (7-bit or 10-bit)
                   uint8_t *data        - Pointer to buffer with data to receive to I2C Slave
                   uint32_t num         - Number of data bytes to receive
                   bool xfer_pending    - Transfer operation is pending - Stop condition will not be generated
                   i2c_resources_t*     - Pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_MasterReceive(uint32_t adr, uint8_t* data, uint32_t num, bool xfer_pending, i2c_resources_t* i2c_resources)
{
    ARGUMENT_UNUSED(xfer_pending);

    /* I2C HAL transmit execution status */
    HAL_StatusTypeDef hal_receive_result;

    /* Shift address left as HAL takes input of 7 bit address - with format AAAA AAAX. A=addr bit, X=ignoring bit */
    adr = adr << 1;

    /* Set I2C status */
    i2c_resources->status.busy      = BUSY;
    i2c_resources->status.direction = RECEIVING_DIR;

    /* Clear error status from previous operations */
    i2c_resources->status.arbitration_lost = false;
    i2c_resources->status.bus_error        = false;

    /* Check if we are only reading */
    if (!i2c_resources->xfer_pending)
    {
        /* Saving slave address */
        i2c_resources->transfer_data.address = adr;

        /* Saving buffer data will be written to  */
        i2c_resources->transfer_data.first_data_buf = (uint8_t*)data;

        /* Saving buffer length */
        i2c_resources->transfer_data.first_data_length = num;

        /* Start I2C transfer */
        hal_receive_result = HAL_I2C_Master_Receive_IT(&i2c_resources->i2c_handler,
                                                       i2c_resources->transfer_data.address,
                                                       i2c_resources->transfer_data.first_data_buf,
                                                       i2c_resources->transfer_data.first_data_length);
    }

    /* Check if we are reading as a part of a WRITE_READ sequence */
    else
    {
        /* Saving buffer data will be written to  */
        i2c_resources->transfer_data.second_data_buf = (uint8_t*)data;

        /* Saving buffer length */
        i2c_resources->transfer_data.second_data_length = num;

        /* Start I2C transfer */
        hal_receive_result = HAL_I2C_Mem_Read_IT(&i2c_resources->i2c_handler,
                                                 i2c_resources->transfer_data.address,
                                                 (*i2c_resources->transfer_data.first_data_buf),
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 i2c_resources->transfer_data.second_data_buf,
                                                 i2c_resources->transfer_data.second_data_length);


        /* Set the resources xfer_pending flag to false */
        i2c_resources->xfer_pending = false;
    }

    /* Return success code in case the operation began successfully */
    if (hal_receive_result == HAL_OK)
    {
        return ARM_DRIVER_OK;
    }

    /* Return execution code according to the I2C HAL execution code */
    switch (hal_receive_result)
    {
        /* In case the driver is busy */
        case HAL_BUSY:
        {
            return ARM_DRIVER_ERROR_BUSY;
        }
        /* In case there was a timeout */
        case HAL_TIMEOUT:
        {
            return ARM_DRIVER_ERROR_TIMEOUT;
        }
        /* In case of any error not aforementioned */
        default:
        {
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
* Description    : Start transmitting data as I2C Slave
* Input          : uint8_t *data        - Pointer to buffer with data to transmit to I2C Master
                   uint32_t num         - Number of data bytes to transmit
                   i2c_resources_t*     - Pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_SlaveTransmit(const uint8_t* data, uint32_t num, i2c_resources_t* i2c_resources)
{
    ARGUMENT_UNUSED(data);
    ARGUMENT_UNUSED(num);
    ARGUMENT_UNUSED(i2c_resources);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
* Description    : Start receiving data as I2C Slave
* Input          : uint8_t *data        - Pointer to buffer with data to receive to I2C Master
                   uint32_t num         - Number of data bytes to receive
                   i2c_resources_t*     - Pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_SlaveReceive(uint8_t* data, uint32_t num, i2c_resources_t* i2c_resources)
{
    ARGUMENT_UNUSED(data);
    ARGUMENT_UNUSED(num);
    ARGUMENT_UNUSED(i2c_resources);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description    : Get transferred data count
 * Input          : i2c_resources_t*     - Pointer to interface resources
 * Return         : int32_t - number of data bytes transferred; -1 when Slave is not addressed by Master
 ***********************************************************************************************************************/
int32_t i2c_GetDataCount(i2c_resources_t* i2c_resources)
{
    ARGUMENT_UNUSED(i2c_resources);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
* Description    : Control I2C Interface
* Input          : uint32_t control     - Operation
                   uint32_t arg         - Argument of operation (optional)
                   i2c_resources_t*     - Pointer to interface resources
* Return         : int32_t - execution status
 ***********************************************************************************************************************/
int32_t i2c_Control(uint32_t control, uint32_t arg, i2c_resources_t* i2c_resources)
{
    ARGUMENT_UNUSED(control);
    ARGUMENT_UNUSED(arg);
    ARGUMENT_UNUSED(i2c_resources);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description    : Get I2C status
 * Input          : i2c_resources_t*     - Pointer to interface resources
 * Return         : ARM_I2C_STATUS       - I2C status
 ***********************************************************************************************************************/
ARM_I2C_STATUS i2c_GetStatus(i2c_resources_t* i2c_resources)
{
    /* Get UART handler state */
    HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(&i2c_resources->i2c_handler);

    /* Configure status according to I2C handler state */
    switch (i2c_state)
    {
        case HAL_I2C_STATE_READY:
        case HAL_I2C_STATE_RESET:
        {
            i2c_resources->status.busy = false;
            break;
        }
        case HAL_I2C_STATE_MASTER_BUSY_RX:
        {
            i2c_resources->status.busy      = true;
            i2c_resources->status.direction = RECEIVING_DIR;
            break;
        }
        case HAL_I2C_STATE_MASTER_BUSY_TX:
        {
            i2c_resources->status.busy      = true;
            i2c_resources->status.direction = TRANSMITTING_DIR;
            break;
        }
        default:
        {
            break;
        }
    }

    if (i2c_resources->i2c_handler.ErrorCode & HAL_I2C_ERROR_BERR)
    {
        i2c_resources->status.bus_error = true;
    }

    if (i2c_resources->i2c_handler.ErrorCode & HAL_I2C_ERROR_ARLO)
    {
        i2c_resources->status.arbitration_lost = true;
    }

    return i2c_resources->status;
}


/**********************************************************************************************************************/
/* HAL weak functions implementation                                                                                  */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Master Tx Transfer completed callback.
 * Input          : hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 * Return         : void
 ***********************************************************************************************************************/
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    i2c_resources_t* p_i2c_resources = (i2c_resources_t*)hi2c;

    /* Call I2C callback if it is not NULL*/
    if (p_i2c_resources->callback != NULL)
    {
        p_i2c_resources->callback(ARM_I2C_EVENT_TRANSFER_DONE);
    }
}

/***********************************************************************************************************************
 * Description    : Memory Transfer Tx complete callback.
 * Input          : hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 * Return         : void
 ***********************************************************************************************************************/
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    i2c_resources_t* p_i2c_resources = (i2c_resources_t*)hi2c;

    /* Call I2C callback if it is not NULL*/
    if (p_i2c_resources->callback != NULL)
    {
        p_i2c_resources->callback(ARM_I2C_EVENT_TRANSFER_DONE);
    }
}

/***********************************************************************************************************************
 * Description    : Master Rx Transfer completed callback.
 * Input          : hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 * Return         : void
 ***********************************************************************************************************************/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    i2c_resources_t* p_i2c_resources = (i2c_resources_t*)hi2c;

    /* Call I2C callback if it is not NULL*/
    if (p_i2c_resources->callback != NULL)
    {
        p_i2c_resources->callback(ARM_I2C_EVENT_TRANSFER_DONE);
    }
}

/***********************************************************************************************************************
 * Description    : Memory Transfer Rx complete callback.
 * Input          : hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 * Return         : void
 ***********************************************************************************************************************/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    i2c_resources_t* p_i2c_resources = (i2c_resources_t*)hi2c;

    /* Call I2C callback if it is not NULL*/
    if (p_i2c_resources->callback != NULL)
    {
        p_i2c_resources->callback(ARM_I2C_EVENT_TRANSFER_DONE);
    }
}

/***********************************************************************************************************************
 * Description    : I2C error callbacks.
 * Input          : hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 * Return         : void
 ***********************************************************************************************************************/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    i2c_resources_t* p_i2c_resources = (i2c_resources_t*)hi2c;

    /* Event */
    uint32_t event = 0;

    /* Error code */
    uint32_t error = 0;

    /* Get error state after it has been updated by the HAL handler */
    error = HAL_I2C_GetError(&p_i2c_resources->i2c_handler);

    /* In case a bus error occurred */
    if (error & HAL_I2C_ERROR_BERR)
    {
        event |= ARM_I2C_EVENT_BUS_ERROR;
    }

    /* In case an arbitration lost error occurred */
    if (error & HAL_I2C_ERROR_ARLO)
    {
        event |= ARM_I2C_EVENT_ARBITRATION_LOST;
    }

    /* Call I2C callback if it is not NULL*/
    if (p_i2c_resources->callback != NULL)
    {
        p_i2c_resources->callback(event);
    }
}

/**********************************************************************************************************************/
/* Internal functions implementation                                                                                  */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Calculates TIMINGR register value to match user's baudrate
 * Input          : bus_speed      - user's baudrate
 *                  rcc_periph_clk - RCC peripheral clock
 * Return         : uint32_t - the calculated register value
 ***********************************************************************************************************************/
__STATIC_INLINE uint32_t i2c_calculate_timing_register(uint32_t bus_speed, uint32_t rcc_periph_clk)
{
    /*  NOTE - YW - Explenation of calculation (refer to reference manual for more info):
     *  T_user = T_sync1 + T_sync2 + T_sclh + T_scll =
     *           T_sync + (SCLH + 1 + SCLL + 1)(PRESC + 1)T_clk =
     *           T_clk * (PRESC + 1) * (SCLH + SCLL + 2 + N_sync);
     *  where N_sync is approximate number of clock cycles for sync.
     *  Notice that T_sync depends on several parameters and this is just approximation for
     *  for the ease of calculation (refer to reference manual for more info about T_sync).
     *  Choosing duty cycle of 50% forces SCLH = SCLL = SCLX, and then:
     *  SCLX = ((F_clk/F_user) / (PRESC + 1) - 2 - N_sync) / 2;
     *  In our calculation we are going to use default values for PRESC, and N_sync.
     *  Notice that SCLH/SCLH are 8 bit and therefore the result must be less than UINT8_MAX
     */

    /* Get peripheral clock frequency */
    uint32_t peripheral_clk_freq = HAL_RCCEx_GetPeriphCLKFreq(rcc_periph_clk);

    /* Calculate SCLL/SCLH */
    double SCLX = ((((double)peripheral_clk_freq / bus_speed) / (PRESC_DEFAULT + 1)) - 2 - TSYNC_IN_CYCLES_DEFAULT) / 2;

    /* Round each variable to make sure SCLL+SCLH = 2SCLX */
    uint32_t SCLL = (uint32_t)(SCLX);
    uint32_t SCLH = (uint32_t)(SCLX + 0.5);

    /* Check if result exceeds max value */
    if (SCLH >= UINT8_MAX)
    {
        return TIMINGR_EMPTY;
    }

    /* Create register */
    uint32_t reg_value = TIMINGR_EMPTY;

    reg_value |= SCLL << I2C_TIMINGR_SCLL_Pos;
    reg_value |= SCLH << I2C_TIMINGR_SCLH_Pos;
    reg_value |= SCLDEL_DEFAULT << I2C_TIMINGR_SCLDEL_Pos;
    reg_value |= SDADEL_DEFAULT << I2C_TIMINGR_SDADEL_Pos;
    reg_value |= PRESC_DEFAULT << I2C_TIMINGR_PRESC_Pos;

    return reg_value;
}
