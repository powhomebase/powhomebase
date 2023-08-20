/**********************************************************************************************************************/
/* Description        : STM32L4 i2c driver header file.                                                               */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Related Project's headers */
#include <Driver_GPIO.h>
#include <Driver_I2C.h>
#include <global.h>
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* Current driver status flag definition */
#define I2C_NOT_INITIALIZED 0
#define I2C_INITIALIZED 1
#define I2C_POWERED 2

#define TRANSMITTING_DIR 0
#define RECEIVING_DIR 1
#define NOT_BUSY 0
#define BUSY 1

#define MASTER true
#define SLAVE false

#define I2C_GENERATE_OBJECT(i2c_num, gpios_struct)                                                                                                                                                  \
extern i2c_resources_t        I2C##i2c_num##_RESOURCES;                                                                                                                                             \
static int32_t                I2C##i2c_num##_Initialize      (ARM_I2C_SignalEvent_t cb_event)           { return i2c_Initialize (cb_event, &I2C##i2c_num##_RESOURCES); }                            \
static int32_t                I2C##i2c_num##_Uninitialize    (void)                                     { return i2c_Uninitialize(&I2C##i2c_num##_RESOURCES); }                                     \
static int32_t                I2C##i2c_num##_PowerControl    (ARM_POWER_STATE state)                    { return i2c_PowerControl (state, &I2C##i2c_num##_RESOURCES); }                             \
static int32_t                I2C##i2c_num##_MasterTransmit  (uint32_t addr, const uint8_t *data,                                                                                                   \
                                                              uint32_t num, bool xfer_pending)          { return (i2c_MasterTransmit (addr, data, num, xfer_pending, &I2C##i2c_num##_RESOURCES));}  \
static int32_t                I2C##i2c_num##_MasterReceive   (uint32_t addr, uint8_t *data,                                                                                                         \
                                                              uint32_t num, bool xfer_pending)          { return (i2c_MasterReceive (addr, data, num, xfer_pending, &I2C##i2c_num##_RESOURCES)); }  \
static int32_t                I2C##i2c_num##_SlaveTransmit   (const uint8_t *data, uint32_t num)        { return (i2c_SlaveTransmit (data, num, &I2C##i2c_num##_RESOURCES)); }                      \
static int32_t                I2C##i2c_num##_SlaveReceive    (uint8_t *data, uint32_t num)              { return (i2c_SlaveReceive (data, num, &I2C##i2c_num##_RESOURCES)); }                       \
static int32_t                I2C##i2c_num##_GetDataCount    (void)                                     { return (i2c_GetDataCount (&I2C##i2c_num##_RESOURCES)); }                                  \
static int32_t                I2C##i2c_num##_Control         (uint32_t control, uint32_t arg)           { return (i2c_Control (control, arg, &I2C##i2c_num##_RESOURCES)); }                         \
static ARM_I2C_STATUS         I2C##i2c_num##_GetStatus       (void)                                     { return i2c_GetStatus (&I2C##i2c_num##_RESOURCES); }                                       \
       void                   I2C##i2c_num##_EV_IRQHandler   (void)                                     {HAL_I2C_EV_IRQHandler (&I2C##i2c_num##_RESOURCES.i2c_handler); }                           \
       void                   I2C##i2c_num##_ER_IRQHandler   (void)                                     {HAL_I2C_ER_IRQHandler(&I2C##i2c_num##_RESOURCES.i2c_handler); }                            \
                                                                                                                                                                                                    \
ARM_DRIVER_I2C Driver_I2C##i2c_num = {                                                                                                                                                              \
    .Initialize      = I2C##i2c_num##_Initialize,                                                                                                                                                   \
    .Uninitialize    = I2C##i2c_num##_Uninitialize,                                                                                                                                                 \
    .PowerControl    = I2C##i2c_num##_PowerControl,                                                                                                                                                 \
    .MasterTransmit  = I2C##i2c_num##_MasterTransmit,                                                                                                                                               \
    .MasterReceive   = I2C##i2c_num##_MasterReceive,                                                                                                                                                \
    .SlaveTransmit   = I2C##i2c_num##_SlaveTransmit,                                                                                                                                                \
    .SlaveReceive    = I2C##i2c_num##_SlaveReceive,                                                                                                                                                 \
    .GetDataCount    = I2C##i2c_num##_GetDataCount,                                                                                                                                                 \
    .Control         = I2C##i2c_num##_Control,                                                                                                                                                      \
    .GetStatus       = I2C##i2c_num##_GetStatus                                                                                                                                                     \
};                                                                                                                                                                                                  \
                                                                                                                                                                                                    \
void enable_i2c##i2c_num##_clock(void)                                                                                                                                                              \
{                                                                                                                                                                                                   \
    __HAL_RCC_I2C##i2c_num##_CLK_ENABLE();                                                                                                                                                          \
    __HAL_RCC_I2C##i2c_num##_CLK_SLEEP_ENABLE();                                                                                                                                                    \
                                                                                                                                                                                                    \
}                                                                                                                                                                                                   \
void disable_i2c##i2c_num##_clock(void)                                                                                                                                                             \
{                                                                                                                                                                                                   \
    __HAL_RCC_I2C##i2c_num##_CLK_SLEEP_DISABLE();                                                                                                                                                   \
    __HAL_RCC_I2C##i2c_num##_CLK_DISABLE();                                                                                                                                                         \
}                                                                                                                                                                                                   \
                                                                                                                                                                                                    \
i2c_resources_t I2C##i2c_num##_RESOURCES = {                                                                                                                                                        \
    .instance         =    I2C##i2c_num,                                                                                                                                                            \
    .gpios            =    &gpios_struct,                                                                                                                                                           \
    .clock_enable     =    enable_i2c##i2c_num##_clock,                                                                                                                                             \
    .clock_disable    =    disable_i2c##i2c_num##_clock,                                                                                                                                            \
    .state            =    I2C_NOT_INITIALIZED,                                                                                                                                                     \
    .master           =    MASTER,                                                                                                                                                                  \
    .user_confs       =    &I2C##i2c_num##_USER_CONF,                                                                                                                                               \
    .irq              =    I2C##i2c_num##_EV_IRQn,                                                                                                                                                  \
    .rcc_periph_clk   =    RCC_PERIPHCLK_I2C##i2c_num,                                                                                                                                              \
};

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*i2c_clock_control_t)(void);

/* SPI Pins Configuration */
typedef struct {
    /* I2C SDA */
    ARM_GPIO_PIN *sda;
    uint32_t      sda_conf;
    /* I2C SCL */
    ARM_GPIO_PIN *scl;
    uint32_t      scl_conf;
    /* Alternate function */
    uint8_t alternate_f;
} i2c_int_gpios_t;

typedef struct {
    /* I2C initialization struct */
    I2C_InitTypeDef *i2cInit;
    /* bus speed */
    uint32_t bus_speed;
} i2c_user_conf_t;

typedef struct {
    /* Address of slave to transmit to */
    uint16_t address;
    /* First data buffer */
    uint8_t *first_data_buf;
    /* First data length */
    uint32_t first_data_length;
    /* Second data buffer */
    uint8_t *second_data_buf;
    /* Second data length */
    uint32_t second_data_length;
} i2c_transfer_t;

typedef struct {
    /* ST I2C handler struct */
    /*this must be the struct itself (and not a pointer) so we could upcast the handle pointer to the resource pointer
     * in the implementation of the weak function*/
    I2C_HandleTypeDef i2c_handler;
    /* I2C typedef */
    I2C_TypeDef *instance;
    /* Data line GPIO */
    i2c_int_gpios_t *gpios;
    /*enable clk function*/
    i2c_clock_control_t clock_enable;
    /*disable clk function*/
    i2c_clock_control_t clock_disable;
    /* Driver state */
    uint8_t state;
    /* Master/Slave flag */
    bool master;
    /* User configurations */
    i2c_user_conf_t *user_confs;
    /* IRQ handler */
    IRQn_Type irq;
    /* Peripheral clock */
    uint32_t rcc_periph_clk;
    /* I2C status */
    ARM_I2C_STATUS status;
    /* Callback function */
    ARM_I2C_SignalEvent_t callback;
    /* Xfer pending flag */
    bool xfer_pending;
    /* Data needed for I2C transfer */
    i2c_transfer_t transfer_data;
} i2c_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

/* Initialize I2C Interface */
int32_t i2c_Initialize(ARM_I2C_SignalEvent_t cb_event, i2c_resources_t *i2c_resources);

/* De-initialize I2C Interface */
int32_t i2c_Uninitialize(i2c_resources_t *i2c_resources);

/* Control I2C Interface Power */
int32_t i2c_PowerControl(ARM_POWER_STATE state, i2c_resources_t *i2c_resources);

/* Start transmitting data as I2C Master */
int32_t i2c_MasterTransmit(uint32_t         adr,
                           const uint8_t *  data,
                           uint32_t         num,
                           bool             xfer_pending,
                           i2c_resources_t *i2c_resources);

/* Start receiving data as I2C Master */
int32_t i2c_MasterReceive(uint32_t adr, uint8_t *data, uint32_t num, bool xfer_pending, i2c_resources_t *i2c_resources);

/* Start transmitting data as I2C Slave */
int32_t i2c_SlaveTransmit(const uint8_t *data, uint32_t num, i2c_resources_t *i2c_resources);

/* Start receiving data as I2C Slave */
int32_t i2c_SlaveReceive(uint8_t *data, uint32_t num, i2c_resources_t *i2c_resources);

/* Get transferred data count */
int32_t i2c_GetDataCount(i2c_resources_t *i2c_resources);

/* Control I2C Interface */
int32_t i2c_Control(uint32_t control, uint32_t arg, i2c_resources_t *i2c_resources);

/* Get I2C status */
ARM_I2C_STATUS i2c_GetStatus(i2c_resources_t *i2c_resources);
