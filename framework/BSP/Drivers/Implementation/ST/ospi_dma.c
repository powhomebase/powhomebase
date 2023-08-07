/***********************************************************************************************************************
 * Description: STM32 OSPI DMA driver implementation.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "ospi_dma.h"

/* Related Project's headers */
#include <proj_assert.h>
#include <string.h>
#include <syscalls.h>

/**********************************************************************************************************************/
/* Functions Implementation                                                                                           */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description  : Initialize OSPI interface.
 * Input        : ospi_resources    - pointer to interface resources.
 *                cb_event          - pointer to callback function.
 * Return       : Execution status.
 **********************************************************************************************************************/
int32_t ospi_dma_Initialize(ospi_dma_resources_t* ospi_resources, ARM_QSPI_SignalEvent_t cb_event)
{
    proj_assert(ospi_resources->state == OSPI_NOT_INITIALIZED);

    /* Initialize the OSPI handle */
    ospi_resources->ospi_handle.Instance = ospi_resources->p_user_conf->instance;

    /* Copy st hal init struct */
    ospi_resources->ospi_handle.Init = *(ospi_resources->p_user_conf->init);

    /* Connect the dma */
    ospi_resources->ospi_handle.hdma = ospi_resources->dma_hal_st;

    /* Register the callback */
    ospi_resources->callback = cb_event;

    /* Configure CLK GPIO */
    gpio_control(ospi_resources->gpios->clk, ARM_GPIO_CONFIGURE_STATE, ospi_resources->gpios->clk_conf);
    gpio_control(ospi_resources->gpios->clk,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(ospi_resources->gpios->alternate_f, ospi_resources->gpios->alternate_f));

    /* Configure CS GPIO */
    gpio_control(ospi_resources->gpios->cs, ARM_GPIO_CONFIGURE_STATE, ospi_resources->gpios->cs_conf);
    gpio_control(ospi_resources->gpios->cs,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(ospi_resources->gpios->alternate_f, ospi_resources->gpios->alternate_f));

    /* Configure IO0 GPIO */
    gpio_control(ospi_resources->gpios->io0, ARM_GPIO_CONFIGURE_STATE, ospi_resources->gpios->io0_conf);
    gpio_control(ospi_resources->gpios->io0,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(ospi_resources->gpios->alternate_f, ospi_resources->gpios->alternate_f));

    /* Configure IO1 GPIO */
    gpio_control(ospi_resources->gpios->io1, ARM_GPIO_CONFIGURE_STATE, ospi_resources->gpios->io1_conf);
    gpio_control(ospi_resources->gpios->io1,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(ospi_resources->gpios->alternate_f, ospi_resources->gpios->alternate_f));

    /* Configure IO2 GPIO */
    gpio_control(ospi_resources->gpios->io2, ARM_GPIO_CONFIGURE_STATE, ospi_resources->gpios->io2_conf);
    gpio_control(ospi_resources->gpios->io2,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(ospi_resources->gpios->alternate_f, ospi_resources->gpios->alternate_f));

    /* Configure IO3 GPIO */
    gpio_control(ospi_resources->gpios->io3, ARM_GPIO_CONFIGURE_STATE, ospi_resources->gpios->io3_conf);
    gpio_control(ospi_resources->gpios->io3,
                 ARM_GPIO_SET_AF,
                 ARM_GPIO_AF_MAKE(ospi_resources->gpios->alternate_f, ospi_resources->gpios->alternate_f));

    /* Initialize DMA and register OSPI IRQ as a callback function to the DMAs lower layer */
    ospi_resources->dma->Initialize(NULL);

    /* Enable OSPI clock to initialize OSPI */
    ospi_resources->clock_enable();

    /* Call HAL initializiation function */
    HAL_OSPI_Init(&ospi_resources->ospi_handle);

    /* Disable OSPI clock */
    ospi_resources->clock_disable();

    /* Update driver state to initialized */
    ospi_resources->state = OSPI_NOT_POWERED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description  : Uninitialize OSPI interface.
 * Input        : ospi_resources    - pointer to interface resources.
 * Return       : Execution status.
 **********************************************************************************************************************/
int32_t ospi_dma_Uninitialize(ospi_dma_resources_t* ospi_resources)
{
    ARGUMENT_UNUSED(ospi_resources);
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description  : Control OSPI Interface Power.
 * Input        : ospi_resources    - pointer to interface resources.
 *                state             - power state.
 * Return       : Execution status.
 **********************************************************************************************************************/
int32_t ospi_dma_PowerControl(ospi_dma_resources_t* ospi_resources, ARM_POWER_STATE state)
{
    /* This is user error, not supposed to happen in runtime */
    proj_assert(ospi_resources->state != OSPI_NOT_INITIALIZED);

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            /* Double ARM_POWER_xx is allowed to happen in cmsis and must return OK */
            if (ospi_resources->state == OSPI_NOT_POWERED)
                return ARM_DRIVER_OK;

            ospi_resources->state = OSPI_NOT_POWERED;

            /* Enable SPI interrupts */
            nvic_enable_irq(ospi_resources->p_user_conf->irq, false);

            /* Disable DMAs */
            ospi_resources->dma->PowerControl(ARM_POWER_OFF);

            /* Uninitialize GPIOs */
            gpio_control(ospi_resources->gpios->cs, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(ospi_resources->gpios->clk, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(ospi_resources->gpios->io3, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(ospi_resources->gpios->io2, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(ospi_resources->gpios->io1, ARM_GPIO_SET_STATE, CLOSE);
            gpio_control(ospi_resources->gpios->io0, ARM_GPIO_SET_STATE, CLOSE);

            /* Disable peripheral clock */
            ospi_resources->clock_disable();

            return ARM_DRIVER_OK;
        }
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        case ARM_POWER_FULL:
        {
            /* Double ARM_POWER_xx is allowed to happen in cmsis and must return OK */
            if ((ospi_resources->state == OSPI_POWERED) || (ospi_resources->state == OSPI_AUTOPOLLING_SET)
                || (ospi_resources->state == OSPI_SENDCMD))
                return ARM_DRIVER_OK;

            /* Enable SPI clock */
            ospi_resources->clock_enable();

            gpio_control(ospi_resources->gpios->io0, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(ospi_resources->gpios->io1, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(ospi_resources->gpios->io2, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(ospi_resources->gpios->io3, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(ospi_resources->gpios->clk, ARM_GPIO_SET_STATE, OPEN);
            gpio_control(ospi_resources->gpios->cs, ARM_GPIO_SET_STATE, OPEN);

            /* Enable DMAs */
            ospi_resources->dma->PowerControl(ARM_POWER_FULL);

            /* Enable SPI interrupts */
            nvic_enable_irq(ospi_resources->p_user_conf->irq, true);

            ospi_resources->state = OSPI_POWERED;

            return ARM_DRIVER_OK;
        }
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}


/***********************************************************************************************************************
 * Description  : Start sending data to OSPI transmitter.
 * Input        : ospi_resources    - pointer to interface resources.
 *                cmd_modes         - command mode.
 *                instruction       - instruction to send.
 *                address           - address to send.
 *                alternate_bytes   - alternate bytes.
 *                data              - data to send.
 *                data_num          - data size.
 *                dummy_clocks      - dummy clocks.
 * Return       : Execution status.
 **********************************************************************************************************************/
int32_t ospi_dma_SendCommand(ospi_dma_resources_t* ospi_resources,
                             uint32_t              cmd_modes,
                             uint32_t              instruction,
                             uint32_t              address,
                             uint32_t              alternate_bytes,
                             void*                 data,
                             uint32_t              data_num,
                             uint32_t              dummy_clocks)
{
    /* SendCommand is logical correct only in this states */
    proj_assert((ospi_resources->state == OSPI_POWERED) || (ospi_resources->state == OSPI_AUTOPOLLING_SET));

    int32_t return_value = ARM_DRIVER_ERROR;

    ospi_resources->status.busy = true;

    OSPI_RegularCmdTypeDef current_command;

    /* Zero out the struct */
    memset(&current_command, 0, sizeof(current_command));

    /* From here on we build the struct for hal st. First the constants */
    current_command.SIOOMode         = HAL_OSPI_SIOO_INST_EVERY_CMD;
    current_command.DataDtrMode      = HAL_OSPI_DATA_DTR_DISABLE;
    current_command.DQSMode          = HAL_OSPI_DQS_DISABLE;

    /* Dummy cycles */
    current_command.DummyCycles = dummy_clocks;

    /* Instruction */
    switch (cmd_modes & ARM_QSPI_INSMODE_Msk)
    {
        case ARM_QSPI_INSMODE_NONE:
            current_command.InstructionMode = HAL_OSPI_INSTRUCTION_NONE;
            break;
        case ARM_QSPI_INSMODE_1LINE:
            current_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
            current_command.Instruction     = instruction;
            break;
        case ARM_QSPI_INSMODE_2LINES:
            current_command.InstructionMode = HAL_OSPI_INSTRUCTION_2_LINES;
            current_command.Instruction     = instruction;
            break;
        case ARM_QSPI_INSMODE_4LINES:
            current_command.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
            current_command.Instruction     = instruction;
            break;
        default:
            return_value = ARM_DRIVER_ERROR_PARAMETER;
            goto exit;
    }

    /* Address */
    switch (cmd_modes & ARM_QSPI_ADDRESS_Msk)
    {
        case ARM_QSPI_ADDRESS_NONE:
            current_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
            break;
        case ARM_QSPI_ADDRESS_1LINE:
            current_command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
            break;
        case ARM_QSPI_ADDRESS_2LINES:
            current_command.AddressMode = HAL_OSPI_ADDRESS_2_LINES;
            break;
        case ARM_QSPI_ADDRESS_4LINES:
            current_command.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
            break;
        default:
            return_value = ARM_DRIVER_ERROR_PARAMETER;
            goto exit;
    }

    /* If we intend to send an address we need to configure the size of it */
    if ((cmd_modes & ARM_QSPI_ADDRESS_Msk) != ARM_QSPI_ADDRESS_NONE)
    {
        switch (cmd_modes & ARM_QSPI_ADDRESS_SIZE_Msk)
        {
            case ARM_QSPI_ADDRESS_SIZE_8BITS:
                current_command.AddressSize = HAL_OSPI_ADDRESS_8_BITS;
                break;
            case ARM_QSPI_ADDRESS_SIZE_16BITS:
                current_command.AddressSize = HAL_OSPI_ADDRESS_16_BITS;
                break;
            case ARM_QSPI_ADDRESS_SIZE_24BITS:
                current_command.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
                break;
            case ARM_QSPI_ADDRESS_SIZE_32BITS:
                current_command.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
                break;
            default:
                return_value = ARM_DRIVER_ERROR_PARAMETER;
                goto exit;
        }

        current_command.Address = address;
    }


    /* Alternate Bytes */
    switch (cmd_modes & ARM_QSPI_ABMODE_Msk)
    {
        case ARM_QSPI_ABMODE_NONE:
            current_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
            break;
        case ARM_QSPI_ABMODE_1LINE:
            current_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_1_LINE;
            break;
        case ARM_QSPI_ABMODE_2LINES:
            current_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_2_LINES;
            break;
        case ARM_QSPI_ABMODE_4LINES:
            current_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
            break;
        default:
            return_value = ARM_DRIVER_ERROR_PARAMETER;
            goto exit;
    }

    /* If we intend to send altenate bytes we need to configure the size of it */
    if ((cmd_modes & ARM_QSPI_ABMODE_Msk) != ARM_QSPI_ABMODE_NONE)
    {
        switch (cmd_modes & ARM_QSPI_AB_SIZE_Msk)
        {
            case ARM_QSPI_AB_SIZE_8BITS:
                current_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
                break;
            case ARM_QSPI_AB_SIZE_16BITS:
                current_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_16_BITS;
                break;
            case ARM_QSPI_AB_SIZE_24BITS:
                current_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_24_BITS;
                break;
            case ARM_QSPI_AB_SIZE_32BITS:
                current_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_32_BITS;
                break;
            default:
                return_value = ARM_DRIVER_ERROR_PARAMETER;
                goto exit;
        }
        current_command.AlternateBytes = alternate_bytes;
    }


    /* Data */
    switch (cmd_modes & ARM_QSPI_DATAMODE_Msk)
    {
        case ARM_QSPI_DATAMODE_NONE:
            current_command.DataMode = HAL_OSPI_DATA_NONE;
            break;
        case ARM_QSPI_DATAMODE_1LINE:
            current_command.DataMode = HAL_OSPI_DATA_1_LINE;
            break;
        case ARM_QSPI_DATAMODE_2LINES:
            current_command.DataMode = HAL_OSPI_DATA_2_LINES;
            break;
        case ARM_QSPI_DATAMODE_4LINES:
            current_command.DataMode = HAL_OSPI_DATA_4_LINES;
            break;
        default:
            return_value = ARM_DRIVER_ERROR_PARAMETER;
            goto exit;
    }

    /* If we intend to send data set the number of bytes we want to send */
    if ((cmd_modes & ARM_QSPI_DATAMODE_Msk) != ARM_QSPI_DATAMODE_NONE)
    {
        current_command.NbData = data_num;
    }

    /* Now we need to send the command */

    HAL_StatusTypeDef result;

    /* If we need to autopolling send the command with autopolling */
    if (ospi_resources->state == OSPI_AUTOPOLLING_SET)
    {
        ospi_resources->state = OSPI_SENDCMD;

        result = HAL_OSPI_AutoPolling_IT(&ospi_resources->ospi_handle, &ospi_resources->autopolling_conf);

        if (result != HAL_OK)
            return_value = ARM_DRIVER_ERROR;
        else
            return_value = ARM_DRIVER_OK;

        goto exit;
    }

    /* Send the command */
    ospi_resources->state = OSPI_SENDCMD;

    result = HAL_OSPI_Command_IT(&ospi_resources->ospi_handle, &current_command);

    if (result != HAL_OK)
    {
        return_value = ARM_DRIVER_ERROR;
        goto exit;
    }

    /* If we want to send/receive data than we want to do it now, after we have sent the command header */
    switch (cmd_modes & ARM_QSPI_CMDTYPE_Msk)
    {
        case ARM_QSPI_CMDTYPE_NORMAL:
            proj_assert(data == NULL);
            break;
        case ARM_QSPI_CMDTYPE_TX:
            proj_assert(data != NULL);
            result = HAL_OSPI_Transmit_DMA(&ospi_resources->ospi_handle, (uint8_t*)data);
            break;
        case ARM_QSPI_CMDTYPE_RX:
            proj_assert(data != NULL);
            result = HAL_OSPI_Receive_DMA(&ospi_resources->ospi_handle, (uint8_t*)data);
            break;
        case ARM_QSPI_CMDTYPE_FUTUREUSE: /* Todo: Not sure if needed */
            proj_assert(data == NULL);
            ospi_resources->state = OSPI_POWERED;
            break;
        default:
            return_value = ARM_DRIVER_ERROR_PARAMETER;
            goto exit;
    }

    if (result != HAL_OK)
        return_value = ARM_DRIVER_ERROR;

exit:

    return return_value;
}

/***********************************************************************************************************************
 * Description  : Start receive data with SPI.
 * Input        : ospi_resources    - pointer to interface resources.
 *                match             - value to be compared with the masked status.
 *                mask              - mask to be applied to the status bytes.
 *                match_mode        - method used for determining a match.
 *                status_byte_size  - size of the status bytes received. (unused)
 *                interval          - number of clock cycles between two read during automatic polling phases.
 *                auto_stop         - specifies if automatic polling is stopped after a match.
 * Return       : Execution status.
 **********************************************************************************************************************/
int32_t ospi_dma_SetAutoPolling(ospi_dma_resources_t* ospi_resources,
                                uint32_t              match,
                                uint32_t              mask,
                                uint8_t               match_mode,
                                uint8_t               status_byte_size,
                                uint16_t              interval,
                                uint8_t               auto_stop)
{
    proj_assert(ospi_resources->state == OSPI_POWERED);
    ARGUMENT_UNUSED(status_byte_size);

    /* Configure the struct */

    ospi_resources->autopolling_conf.Match           = match;
    ospi_resources->autopolling_conf.Mask            = mask;
    ospi_resources->autopolling_conf.MatchMode       = match_mode;
    ospi_resources->autopolling_conf.Interval        = interval;

    switch (auto_stop)
    {
        case ARM_QSPI_DRIVER_AUTOPOOL_AUTOSTOP_DISABLED:
            ospi_resources->autopolling_conf.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_DISABLE;
            break;
        case ARM_QSPI_DRIVER_AUTOPOOL_AUTOSTOP_ENABLE:
            ospi_resources->autopolling_conf.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
            break;
        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Move state machine to a state that signals the next sendCommand to use autopolling */
    ospi_resources->state = OSPI_AUTOPOLLING_SET;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description  : Control OSPI Interface.
 * Input        : ospi_resources    - pointer to interface resources.
 *                control           - operation.
 *                arg               - argument of operation (optional).
 * Return       : Execution status.
 **********************************************************************************************************************/
int32_t ospi_dma_Control(ospi_dma_resources_t* ospi_resources, uint32_t control, uint32_t arg)
{
    ARGUMENTS_UNUSED(ospi_resources, control, arg);
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description  : Transfer from HAL_DMA_StateTypeDef to ARM_QSPI_STATUS.
 * Input        : ospi_resources    - pointer to interface resources.
 * Return       : OSPI status.
 **********************************************************************************************************************/
ARM_QSPI_STATUS ospi_dma_GetStatus(ospi_dma_resources_t* ospi_resources)
{
    return ospi_resources->status;
}

/***********************************************************************************************************************
 * Description  : OSPI callback from CMSIS dma layer.
 * Input        : ospi_resources    - pointer to interface resources.
 *                event             - raised event.
 **********************************************************************************************************************/
void ospi_dma_callback(ospi_dma_resources_t* ospi_resources, uint32_t event)
{
    proj_assert(ospi_resources->state == OSPI_SENDCMD || ospi_resources->state == OSPI_AUTOPOLLING_SET);

    /* Reset busy flag */
    ospi_resources->status.busy      = false;
    ospi_resources->status.data_lost = false;

    /* Move state machine to idle */
    ospi_resources->state = OSPI_POWERED;

    /* Call the callback if exist */
    if (ospi_resources->callback)
        ospi_resources->callback(event);
}


/**********************************************************************************************************************/
/* HAL weak functions implementation                                                                                  */
/**********************************************************************************************************************/

/**
 * @brief  Transfer Error callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef* hospi)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    ospi_dma_resources_t* ospi_resources = (ospi_dma_resources_t*)hospi;

    ospi_dma_callback(ospi_resources, ARM_QSPI_EVENT_DATA_LOST);
}

/**
 * @brief  Command completed callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef* hospi)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    ospi_dma_resources_t* ospi_resources = (ospi_dma_resources_t*)hospi;

    ospi_dma_callback(ospi_resources, ARM_QSPI_EVENT_TRANSFER_COMPLETE);
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef* hospi)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    ospi_dma_resources_t* ospi_resources = (ospi_dma_resources_t*)hospi;

    ospi_dma_callback(ospi_resources, ARM_QSPI_EVENT_TRANSFER_COMPLETE);
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef* hospi)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    ospi_dma_resources_t* ospi_resources = (ospi_dma_resources_t*)hospi;

    ospi_dma_callback(ospi_resources, ARM_QSPI_EVENT_TRANSFER_COMPLETE);
}

/**
 * @brief  FIFO Threshold callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_FifoThresholdCallback(OSPI_HandleTypeDef* hospi)
{
    ARGUMENT_UNUSED(hospi);
    /* We're not working with HAL OSPI's interrupt cmds so we're not expecting
       this. However, in release we don't care if that happens unless it's
       effecting other things. */
    proj_assert(false);
}

/**
 * @brief  Status Match callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef* hospi)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    ospi_dma_resources_t* ospi_resources = (ospi_dma_resources_t*)hospi;

    ospi_dma_callback(ospi_resources, ARM_QSPI_EVENT_AUTOPOL_TRIGGER);
}

/**
 * @brief  Timeout callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_OSPI_TimeOutCallback(OSPI_HandleTypeDef* hospi)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    ospi_dma_resources_t* ospi_resources = (ospi_dma_resources_t*)hospi;

    ospi_dma_callback(ospi_resources, ARM_QSPI_EVENT_DATA_LOST);
}
