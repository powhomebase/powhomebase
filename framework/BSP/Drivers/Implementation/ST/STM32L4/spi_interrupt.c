/*******************************************************************************
* Description        : SPI using interrupt interafce implementation
*******************************************************************************/

/*********************************************************/
/* Includes 											 */
/*********************************************************/
#include <spi_interrupt.h>
#include "syscalls.h"
#include "gpio.h"

/*********************************************************/
/* Variables											 */
/*********************************************************/

gpio_pin_obj_t cs_pin;

/*******************************************************************************
* Description    : Initialize SPI interface
* Input          : ARM_SPI_SignalEvent_t      			 - Pointer to callback function
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Initialize(ARM_SPI_SignalEvent_t cb_event, spi_interrupt_resources_t* spi_resources)
{
	spi_resources->spi_handler->Instance = spi_resources->user_conf->instance;

	spi_resources->spi_handler->Init.BaudRatePrescaler = spi_resources->user_conf->init->BaudRatePrescaler;
	spi_resources->spi_handler->Init.CLKPhase = spi_resources->user_conf->init->CLKPhase;
	spi_resources->spi_handler->Init.CLKPolarity = spi_resources->user_conf->init->CLKPolarity;
	spi_resources->spi_handler->Init.CRCCalculation = spi_resources->user_conf->init->CRCCalculation;
	spi_resources->spi_handler->Init.CRCLength = spi_resources->user_conf->init->CRCLength;
	spi_resources->spi_handler->Init.CRCPolynomial = spi_resources->user_conf->init->CRCPolynomial;
	spi_resources->spi_handler->Init.DataSize = spi_resources->user_conf->init->DataSize;
	spi_resources->spi_handler->Init.Direction = spi_resources->user_conf->init->Direction;
	spi_resources->spi_handler->Init.Mode = spi_resources->user_conf->init->Mode;
	spi_resources->spi_handler->Init.NSS = spi_resources->user_conf->init->NSS;
	spi_resources->spi_handler->Init.NSSPMode = spi_resources->user_conf->init->NSSPMode;
	spi_resources->spi_handler->Init.TIMode = spi_resources->user_conf->init->TIMode;

	spi_resources->callback = cb_event;

	 return ARM_DRIVER_OK;
}

/*******************************************************************************
* Description    : De-initialize SPI interface
* Input          : spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Uninitialize(spi_interrupt_resources_t* spi_resources)
{
    ARGUMENT_UNUSED(spi_resources);

	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/*******************************************************************************
* Description    : Control SPI Interface Power
* Input          : ARM_POWER_STATE              - Power state
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_PowerControl(ARM_POWER_STATE state,spi_interrupt_resources_t* spi_resources)
{
	/* GPIO init struct (used when opening peripheral) */
  GPIO_InitTypeDef  gpio_init;

	switch(state)
	{
		case ARM_POWER_OFF:
			/* Return ARM_DRIVER_OK if driver is already powered to the desired state */
      if(spi_resources->spi_handler->State == HAL_SPI_STATE_RESET)
				return ARM_DRIVER_OK;

			/* Disable peripheral interrupts */
			nvic_enable_irq(spi_resources->user_conf->irq, false);

			/* Disable peripheral clock */
			spi_resources->user_conf->clock_disable();

			/* Uninitialize GPIOs */
			HAL_GPIO_DeInit(spi_resources->user_conf->gpios->clk_gpio_port, spi_resources->user_conf->gpios->clk_gpio_pin);
			HAL_GPIO_DeInit(spi_resources->user_conf->gpios->cs_gpio_port, spi_resources->user_conf->gpios->cs_gpio_pin);
			HAL_GPIO_DeInit(spi_resources->user_conf->gpios->miso_gpio_port, spi_resources->user_conf->gpios->miso_gpio_pin);
			HAL_GPIO_DeInit(spi_resources->user_conf->gpios->mosi_gpio_port, spi_resources->user_conf->gpios->mosi_gpio_pin);

			/* Call SPI HAL uninitialize function */
      HAL_SPI_DeInit(spi_resources->spi_handler);

			return ARM_DRIVER_OK;

		/* Low Power Mode - NOT SUPPORTED */
		case ARM_POWER_LOW:
			return ARM_DRIVER_ERROR_UNSUPPORTED;

		/* Turn on driver */
		case ARM_POWER_FULL:
			/* Return ARM_DRIVER_OK if driver is already powered to the desired state */
      if(spi_resources->spi_handler->State == HAL_SPI_STATE_READY)
				return ARM_DRIVER_OK;

      /* Enable SPI clock */
			spi_resources->user_conf->clock_enable();

			/* Enable GPIOs' clocks - currently done in a different place */

			/* Configure CLK GPIO */
			gpio_init.Pin       = spi_resources->user_conf->gpios->clk_gpio_pin;
			gpio_init.Mode = GPIO_MODE_AF_PP;
			gpio_init.Alternate = GPIO_AF5_SPI1;
			gpio_init.Pull = GPIO_NOPULL;
			gpio_init.Speed = GPIO_SPEED_HIGH;
			HAL_GPIO_Init(spi_resources->user_conf->gpios->clk_gpio_port, &gpio_init);

			/* Configure CS GPIO */
            cs_pin.gpio_port = spi_resources->user_conf->gpios->cs_gpio_port;
            cs_pin.pin_num   = spi_resources->user_conf->gpios->cs_gpio_pin;

			//gpio_init.Pin       = spi_resources->user_conf->gpios->cs_gpio_pin;
			//gpio_init.Mode = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_AF_PP;
			//gpio_init.Alternate = GPIO_AF5_SPI1;
			//gpio_init.Pull = GPIO_PULLUP;//GPIO_NOPULL;
			//gpio_init.Speed = GPIO_SPEED_HIGH;
			//HAL_GPIO_Init(spi_resources->user_conf->gpios->cs_gpio_port, &gpio_init);

            gpio_control(&cs_pin,
                         ARM_GPIO_CONFIGURE_STATE,
                         ARM_GPIO_CONF_MAKE(ARM_GPIO_OUTPUT | ARM_GPIO_DOUT_HIGH, ARM_GPIO_DISABLED));

            gpio_control(&cs_pin, ARM_GPIO_SET_STATE, OPEN);

            /* Configure MISO GPIO */
			gpio_init.Pin       = spi_resources->user_conf->gpios->miso_gpio_pin;
			gpio_init.Mode = GPIO_MODE_AF_PP;
			gpio_init.Alternate = GPIO_AF5_SPI1;
			gpio_init.Pull = GPIO_NOPULL;
			gpio_init.Speed = GPIO_SPEED_HIGH;
			HAL_GPIO_Init(spi_resources->user_conf->gpios->miso_gpio_port, &gpio_init);

			/* Configure MOSI GPIO */
			gpio_init.Pin       = spi_resources->user_conf->gpios->mosi_gpio_pin;
			gpio_init.Mode = GPIO_MODE_AF_PP;
			gpio_init.Alternate = GPIO_AF5_SPI1;
			gpio_init.Pull = GPIO_NOPULL;
			gpio_init.Speed = GPIO_SPEED_HIGH;
			HAL_GPIO_Init(spi_resources->user_conf->gpios->mosi_gpio_port, &gpio_init);

			/* Enable SPI interrupts */
			nvic_enable_irq(spi_resources->user_conf->irq, true);

			/* Call SPI HAL initializiation function */
			HAL_SPI_Init(spi_resources->spi_handler);

			return ARM_DRIVER_OK;

		/* Parameter Error */
		default:
			return ARM_DRIVER_OK;
	}
}

/*******************************************************************************
* Description    : Start sending data to SPI transmitter.
* Input          : void *data                   - Pointer to buffer with data to send
                   uint32_t num                 - Number of data items to send
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Send(const void *data, uint32_t num,spi_interrupt_resources_t* spi_resources)
{
	  /* SPI HAL transmit execution status */
    HAL_StatusTypeDef hal_transmit_result;

    /* Send using interrupt with the given SPI HAL function */

    hal_transmit_result = HAL_SPI_Transmit_IT(spi_resources->spi_handler, (uint8_t *)data, (uint16_t) num);

    /* Return execution code according to the SPI HAL execution code */
    switch(hal_transmit_result){
        /* In case the execution was as expected */
        case HAL_OK:
            return ARM_DRIVER_OK;
        /* In case the driver is busy */
        case HAL_BUSY:
            return ARM_DRIVER_ERROR_BUSY;
        /* In case there was a timeout */
        case HAL_TIMEOUT:
            return ARM_DRIVER_ERROR_TIMEOUT;
        /* In case of any error not aforementioned */
        default:
            return ARM_DRIVER_ERROR;
		}
}


/*******************************************************************************
* Description    : Start receiving data from SPI receiver.
* Input          : void *data                   - Pointer to buffer for data to receive
                   uint32_t num                 - Number of data items to receive
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Receive(void *data, uint32_t num,spi_interrupt_resources_t* spi_resources)
{
    /* SPI HAL transmit execution status */
    HAL_StatusTypeDef hal_receive_result;

    /* Send using interrupt with the given SPI HAL function */
    hal_receive_result = HAL_SPI_Receive_IT(spi_resources->spi_handler, (uint8_t *)data, (uint16_t) num);

    /* Return execution code according to the SPI HAL execution code */
    switch(hal_receive_result){
        /* In case the execution was as expected */
        case HAL_OK:
            return ARM_DRIVER_OK;
        /* In case the driver is busy */
        case HAL_BUSY:
            return ARM_DRIVER_ERROR_BUSY;
        /* In case there was a timeout */
        case HAL_TIMEOUT:
            return ARM_DRIVER_ERROR_TIMEOUT;
        /* In case of any error not aforementioned */
        default:
            return ARM_DRIVER_ERROR;
    }
}

/*******************************************************************************
* Description    : Start sending/receiving data to/from SPI transmitter/receiver
* Input          : void *data_out               - Pointer to buffer with data to send
                   void *data_in                - Pointer to buffer for data to receive
                   uint32_t num                 - Number of data items to transfer
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Transfer(const void *data_out, void *data_in, uint32_t num, spi_interrupt_resources_t* spi_resources)
{
    ARGUMENT_UNUSED(data_out);
    ARGUMENT_UNUSED(data_in);
    ARGUMENT_UNUSED(num);
    ARGUMENT_UNUSED(spi_resources);

	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/*******************************************************************************
* Description    : Get transferred data count
* Input          : spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : uint32_t - number of data items transmitted
*******************************************************************************/
uint32_t spi_interrupt_GetDataCount(spi_interrupt_resources_t* spi_resources)
{
    ARGUMENT_UNUSED(spi_resources);

	return 0;
}

/*******************************************************************************
* Description    : Control SPI Interface
* Input          : uint32_t control             		- Operation
                   uint32_t arg                 		- Argument of operation (optional)
                   spi_interrupt_resources_t*       - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Control(uint32_t control, uint32_t arg,spi_interrupt_resources_t* spi_resources)
{
    ARGUMENT_UNUSED(spi_resources);

    switch(control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_CONTROL_SS:
            if (arg == ARM_SPI_SS_INACTIVE)
            {
                gpio_write(&cs_pin, HIGH);
            }
            if (arg == ARM_SPI_SS_ACTIVE)
            {
                gpio_write(&cs_pin, LOW);
            }
            return ARM_DRIVER_OK;
        default:
            return ARM_DRIVER_ERROR;
    }
}

/*******************************************************************************
* Description    : Get SPI status
* Input          : spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : ARM_SPI_STATUS             - USART status
*******************************************************************************/
ARM_SPI_STATUS spi_interrupt_GetStatus(spi_interrupt_resources_t* spi_resources)
{
	/* Get SPI handler state */
	HAL_SPI_StateTypeDef spi_state = HAL_SPI_GetState(spi_resources->spi_handler);

	/* Configure status according to UART handler state */
	switch(spi_state){
		case HAL_SPI_STATE_BUSY_RX:
        case HAL_SPI_STATE_BUSY_TX:
        case HAL_SPI_STATE_BUSY_TX_RX:
				spi_resources->status.busy = true;
				spi_resources->status.mode_fault = false;
            break;
        case HAL_SPI_STATE_READY:
        case HAL_SPI_STATE_RESET:
				spi_resources->status.busy = false;
				spi_resources->status.mode_fault = false;
                break;
		case HAL_SPI_STATE_ERROR:
				spi_resources->status.mode_fault = true;
        default:
                break;
    }

	return spi_resources->status;
}

/*******************************************************************************
* Description    : Tx interrupt handler
* Input          : uart_int_resources_t*         - Pointer to interface resources
* Return         : void
*******************************************************************************/
void SPI_INTERRUPT_IRQHandler(spi_interrupt_resources_t *spi_resources)
{
	bool busy_before, busy_after;
	uint32_t event = 0;

	spi_interrupt_GetStatus(spi_resources);

	busy_before = spi_resources->status.busy;

	HAL_SPI_IRQHandler(spi_resources->spi_handler);

	spi_interrupt_GetStatus(spi_resources);

	busy_after = spi_resources->status.busy;

	/* Transfer Complete */
	if (busy_before && !busy_after)
    {
		event |= ARM_SPI_EVENT_TRANSFER_COMPLETE;

        if (spi_resources->callback !=NULL)
            spi_resources->callback(event);
    }
}
