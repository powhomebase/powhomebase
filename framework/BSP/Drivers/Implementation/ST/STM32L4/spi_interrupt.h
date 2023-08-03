/*******************************************************************************
* Description        : SPI using interrupt interafce header
*******************************************************************************/
#pragma once

/*********************************************************/
/* Includes 											 */
/*********************************************************/

/* Related Project's headers */
#include <Driver_SPI.h>
#include <Driver_GPIO.h>
#include <global.h>
#include <stm32l4xx.h>

/*********************************************************/
/* Macros	 											 */
/*********************************************************/
//#define SPI_NOT_INITIALIZED               0
//#define SPI_INITIALIZED                   1
//#define SPI_POWERED                       2

#define SPI_INTERRUPT_GENERATE_OBJECT(spi_num)                                                                                                                                                            			\
extern spi_interrupt_resources_t   		SPI##spi_num##_RESOURCES;                                                                                                                                           			\
static int32_t                  			SPI##spi_num##_Initialize      (ARM_SPI_SignalEvent_t cb_event)                    { return spi_interrupt_Initialize (cb_event, &SPI##spi_num##_RESOURCES); }             \
static int32_t                  			SPI##spi_num##_Uninitialize    (void)                                              { return spi_interrupt_Uninitialize(&SPI##spi_num##_RESOURCES); }                      \
static int32_t                  			SPI##spi_num##_PowerControl    (ARM_POWER_STATE state)                             { return spi_interrupt_PowerControl (state, &SPI##spi_num##_RESOURCES); }              \
static int32_t                  			SPI##spi_num##_Send            (const void *data, uint32_t num)                    { return spi_interrupt_Send (data, num, &SPI##spi_num##_RESOURCES); }                  \
static int32_t                  			SPI##spi_num##_Receive         (void *data, uint32_t num)                          { return spi_interrupt_Receive (data, num, &SPI##spi_num##_RESOURCES); }               \
static int32_t                  			SPI##spi_num##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return spi_interrupt_Transfer (data_out, data_in, num, &SPI##spi_num##_RESOURCES); } \
static uint32_t                 			SPI##spi_num##_GetDataCount    (void)                                              { return spi_interrupt_GetDataCount (&SPI##spi_num##_RESOURCES); }                     \
static int32_t                  			SPI##spi_num##_Control         (uint32_t control, uint32_t arg)                    { return spi_interrupt_Control (control, arg, &SPI##spi_num##_RESOURCES); }            \
static ARM_SPI_STATUS           			SPI##spi_num##_GetStatus       (void)                                              { return spi_interrupt_GetStatus (&SPI##spi_num##_RESOURCES); }                        \
																																																																																																								\
       void                     			SPI##spi_num##_IRQHandler (void)                                    							 { SPI_INTERRUPT_IRQHandler(&SPI##spi_num##_RESOURCES); }                       				\
																																																																																																								\
ARM_DRIVER_SPI Driver_SPI##spi_num = {                                                                                                                                                              						\
    SPI##spi_num##_Initialize,                                                                                                                                                                      						\
    SPI##spi_num##_Uninitialize,                                                                                                                                                                    						\
    SPI##spi_num##_PowerControl,                                                                                                                                                                    						\
    SPI##spi_num##_Send,                                                                                                                                                                            						\
    SPI##spi_num##_Receive,                                                                                                                                                                         						\
    SPI##spi_num##_Transfer,                                                                                                                                                                        						\
    SPI##spi_num##_GetDataCount,                                                                                                                                                                    						\
    SPI##spi_num##_Control,                                                                                                                                                                        	 						\
    SPI##spi_num##_GetStatus                                                                                                                                                                       							\
};                                                                                                                                                                                                  						\
																																																																																																								\
spi_interrupt_resources_t SPI##spi_num##_RESOURCES = {                                                                                                                                               						\
    &spi##spi_num##_handler,                                                                                                                                                                               			\
    &SPI##spi_num##_USER_CONF                                                                                                                                                                   								\
};																																																																																																							\

/*********************************************************/
/* Typedefs			 									 */
/*********************************************************/

/* Clock enable function */
typedef void (*clock_control)(void);

/* SSP Pins Configuration */
typedef struct {
  uint16_t									clk_gpio_pin;		  // SCK pin
	GPIO_TypeDef              *clk_gpio_port;   // SCK port
	uint16_t									miso_gpio_pin;		// MISO pin
  GPIO_TypeDef              *miso_gpio_port;  // MISO port
	uint16_t									mosi_gpio_pin;		// MOSI pin
  GPIO_TypeDef              *mosi_gpio_port; 	// MOSI port
	uint16_t									cs_gpio_pin;			// CS port
  GPIO_TypeDef              *cs_gpio_port;   	// CS pin
	uint8_t								    alternate_g; 			// Alternate Function
} spi_int_gpios_t;

typedef struct{
		/* SPI registers base address */
		SPI_TypeDef											*instance;
		/* SPI communication parameters */
		SPI_InitTypeDef         				*init;
		/* SPI pins configuration */
		spi_int_gpios_t									*gpios;
		/* Clock enable function */
    clock_control               		clock_enable;
    /* Clock disable function */
    clock_control              		  clock_disable;
		/* SPI IRQ number */
	  IRQn_Type												irq;
} spi_interrupt_user_conf_t;

typedef struct{
    /* SPI register interface */
    SPI_HandleTypeDef 		        	*spi_handler;
	  /* SPI user configurations */
    spi_interrupt_user_conf_t       *user_conf;
    /* SPI status */
    ARM_SPI_STATUS        		      status;
    /* Callback function */
    ARM_SPI_SignalEvent_t     			callback;
} spi_interrupt_resources_t;


/*********************************************************/
/* Function Declarations								 */
/*********************************************************/

/*******************************************************************************
* Description    : Initialize SPI interface
* Input          : ARM_SPI_SignalEvent_t      			 - Pointer to callback function
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Initialize(ARM_SPI_SignalEvent_t cb_event, spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : De-initialize SPI interface
* Input          : spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Uninitialize(spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Control SPI Interface Power
* Input          : ARM_POWER_STATE              - Power state
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_PowerControl(ARM_POWER_STATE state,spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Start sending data to SPI transmitter.
* Input          : void *data                   - Pointer to buffer with data to send
                   uint32_t num                 - Number of data items to send
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Send(const void *data, uint32_t num,spi_interrupt_resources_t* spi_resources);


/*******************************************************************************
* Description    : Start receiving data from SPI receiver.
* Input          : void *data                   - Pointer to buffer for data to receive
                   uint32_t num                 - Number of data items to receive
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Receive(void *data, uint32_t num,spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Start sending/receiving data to/from SPI transmitter/receiver
* Input          : void *data_out               - Pointer to buffer with data to send
                   void *data_in                - Pointer to buffer for data to receive
                   uint32_t num                 - Number of data items to transfer
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Transfer(const void *data_out, void *data_in, uint32_t num,spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Get transferred data count
* Input          : spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : uint32_t - number of data items transmitted
*******************************************************************************/
uint32_t spi_interrupt_GetDataCount(spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Control SPI Interface
* Input          : uint32_t control             - Operation
                   uint32_t arg                 - Argument of operation (optional)
                   spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : int32_t - execution status
*******************************************************************************/
int32_t spi_interrupt_Control(uint32_t control, uint32_t arg,spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Get SPI status
* Input          : spi_interrupt_resources_t*        - Pointer to interface resources
* Return         : ARM_SPI_STATUS             - USART status
*******************************************************************************/
ARM_SPI_STATUS spi_interrupt_GetStatus(spi_interrupt_resources_t* spi_resources);

/*******************************************************************************
* Description    : Tx interrupt handler 
* Input          : uart_int_resources_t*         - Pointer to interface resources
* Return         : void
*******************************************************************************/
void SPI_INTERRUPT_IRQHandler(spi_interrupt_resources_t *spi_resources);
