/***********************************************************************************************************************
 * Description: STM32 QSPI DMA driver header file.
 **********************************************************************************************************************/

#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Related Project's headers */
#include <Driver_DMA.h>
#include <Driver_GPIO.h>
#include <Driver_QSPI.h>
#include <global.h>

/* Libraries */
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Generate a QSPI DMA driver object.
 * Input          : num_           - peripheral number.
 *                  user_conf_     - user configuration.
 *                  gpio_conf_     - gpios configuration.
 *                  dma_driver_ - dma transmission driver.
 *                  dma_handle_ - dma transmission handle.
 ***********************************************************************************************************************/
#define QSPI_DMA_GENERATE_OBJECT(num_, user_conf_, gpios_conf_, dma_driver_, dma_handle_)                              \
    extern qspi_dma_resources_t QSPI##num_##_RESOURCES;                                                                \
    static int32_t              QSPI##num_##_Initialize(ARM_SPI_SignalEvent_t cb_event)                                \
    {                                                                                                                  \
        return qspi_dma_Initialize(&QSPI##num_##_RESOURCES, cb_event);                                                 \
    }                                                                                                                  \
    static int32_t QSPI##num_##_Uninitialize(void)                                                                     \
    {                                                                                                                  \
        return qspi_dma_Uninitialize(&QSPI##num_##_RESOURCES);                                                         \
    }                                                                                                                  \
    static int32_t QSPI##num_##_PowerControl(ARM_POWER_STATE state)                                                    \
    {                                                                                                                  \
        return qspi_dma_PowerControl(&QSPI##num_##_RESOURCES, state);                                                  \
    }                                                                                                                  \
    static int32_t QSPI##num_##_SendCommand(uint32_t cmd_modes,                                                        \
                                            uint32_t instruction,                                                      \
                                            uint32_t address,                                                          \
                                            uint32_t alternate_bytes,                                                  \
                                            void*    data,                                                             \
                                            uint32_t data_num,                                                         \
                                            uint32_t dummy_clocks)                                                     \
    {                                                                                                                  \
        return qspi_dma_SendCommand(                                                                                   \
            &QSPI##num_##_RESOURCES, cmd_modes, instruction, address, alternate_bytes, data, data_num, dummy_clocks);  \
    }                                                                                                                  \
    static int32_t QSPI##num_##_SetAutoPolling(uint32_t match,                                                         \
                                               uint32_t mask,                                                          \
                                               uint8_t  match_mode,                                                    \
                                               uint8_t  status_byte_size,                                              \
                                               uint16_t interval,                                                      \
                                               uint8_t  auto_stop)                                                     \
    {                                                                                                                  \
        return qspi_dma_SetAutoPolling(                                                                                \
            &QSPI##num_##_RESOURCES, match, mask, match_mode, status_byte_size, interval, auto_stop);                  \
    }                                                                                                                  \
    static int32_t QSPI##num_##_Control(uint32_t control, uint32_t arg)                                                \
    {                                                                                                                  \
        return qspi_dma_Control(&QSPI##num_##_RESOURCES, control, arg);                                                \
    }                                                                                                                  \
    static ARM_QSPI_STATUS QSPI##num_##_GetStatus(void)                                                                \
    {                                                                                                                  \
        return qspi_dma_GetStatus(&QSPI##num_##_RESOURCES);                                                            \
    }                                                                                                                  \
                                                                                                                       \
    void QUADSPI_IRQHandler(void)                                                                                      \
    {                                                                                                                  \
        HAL_QSPI_IRQHandler(&QSPI##num_##_RESOURCES.qspi_handle);                                                      \
    }                                                                                                                  \
                                                                                                                       \
                                                                                                                       \
    ARM_DRIVER_QSPI Driver_QSPI##num_ = {                                                                              \
        .GetVersion      = FUNCTION_NOT_IMPLEMENTED,                                                                   \
        .GetCapabilities = FUNCTION_NOT_IMPLEMENTED,                                                                   \
        .Initialize      = QSPI##num_##_Initialize,                                                                    \
        .Uninitialize    = QSPI##num_##_Uninitialize,                                                                  \
        .PowerControl    = QSPI##num_##_PowerControl,                                                                  \
        .SendCommand     = QSPI##num_##_SendCommand,                                                                   \
        .SetAutoPolling  = QSPI##num_##_SetAutoPolling,                                                                \
        .Control         = QSPI##num_##_Control,                                                                       \
        .GetStatus       = QSPI##num_##_GetStatus,                                                                     \
    };                                                                                                                 \
                                                                                                                       \
    void enable_qspi_clock(void)                                                                                       \
    {                                                                                                                  \
        __QSPI_CLK_ENABLE();                                                                                           \
        __QSPI_CLK_SLEEP_ENABLE();                                                                                     \
    }                                                                                                                  \
    void disable_qspi_clock(void)                                                                                      \
    {                                                                                                                  \
        __QSPI_CLK_SLEEP_DISABLE();                                                                                    \
        __QSPI_CLK_DISABLE();                                                                                          \
    }                                                                                                                  \
                                                                                                                       \
    qspi_dma_resources_t QSPI##num_##_RESOURCES = {                                                                    \
        .p_user_conf   = &user_conf_,                                                                                  \
        .gpios         = &gpios_conf_,                                                                                 \
        .clock_enable  = enable_qspi_clock,                                                                            \
        .clock_disable = disable_qspi_clock,                                                                           \
        .state         = QSPI_NOT_INITIALIZED,                                                                         \
        .dma           = &dma_driver_,                                                                                 \
        .dma_hal_st    = &dma_handle_,                                                                                 \
    };


/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*clock_control)(void);

typedef enum {
    QSPI_NOT_INITIALIZED,
    QSPI_NOT_POWERED,
    QSPI_POWERED,
    QSPI_SENDCMD,
    QSPI_AUTOPOLLING_SET,
} qspi_dma_state_t;

/* QSPI Pins Configuration */
typedef struct {
    ARM_GPIO_PIN* io0;
    uint32_t      io0_conf;
    ARM_GPIO_PIN* io1;
    uint32_t      io1_conf;
    ARM_GPIO_PIN* io2;
    uint32_t      io2_conf;
    ARM_GPIO_PIN* io3;
    uint32_t      io3_conf;
    ARM_GPIO_PIN* cs;
    uint32_t      cs_conf;
    ARM_GPIO_PIN* clk;
    uint32_t      clk_conf;
    uint8_t       alternate_f;
} qspi_dma_gpios_t;

/* User configuration struct (to be declared and altered in Driver_objs) */
typedef struct {
    /* SPI registers base address */
    QUADSPI_TypeDef* instance;
    /* SPI communication parameters */
    QSPI_InitTypeDef const* const init;
    /* QSPI IRQ number */
    IRQn_Type irq;
} const qspi_dma_user_conf_t;

/* Resources struct (to be declared in the macro) */
typedef struct {
    /* SPI register interface */
    /*this must be the struct itself (and not a pointer) so we could upcast the handle pointer to the resource pointer
     * in the implementation of the weak function*/
    QSPI_HandleTypeDef qspi_handle;
    /* SPI user configurations */
    qspi_dma_user_conf_t* p_user_conf;
    /* SPI pins configuration */
    qspi_dma_gpios_t* gpios;
    /* SPI Clock enable function */
    clock_control clock_enable;
    /* SPI Clock disable function */
    clock_control clock_disable;
    /* This layer state machine */
    qspi_dma_state_t state;
    /* SPI status */
    ARM_QSPI_STATUS status;
    /* Callback function */
    ARM_QSPI_SignalEvent_t callback;
    /* DMA cmsis object */
    ARM_DRIVER_DMA* dma;
    /* DMA hal st object */
    DMA_HandleTypeDef* dma_hal_st;
    /* Current Autopolling config struct */
    QSPI_AutoPollingTypeDef autopolling_conf;
} qspi_dma_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

/* Initialize QSPI interface */
int32_t qspi_dma_Initialize(qspi_dma_resources_t* qspi_resources, ARM_QSPI_SignalEvent_t cb_event);

/* De-initialize SPI interface */
int32_t qspi_dma_Uninitialize(qspi_dma_resources_t* qspi_resources);

/* Control SPI Interface Power */
int32_t qspi_dma_PowerControl(qspi_dma_resources_t* qspi_resources, ARM_POWER_STATE state);

/* Send a command using QSPI */
int32_t qspi_dma_SendCommand(qspi_dma_resources_t* qspi_resources,
                             uint32_t              cmd_modes,
                             uint32_t              instruction,
                             uint32_t              address,
                             uint32_t              alternate_bytes,
                             void*                 data,
                             uint32_t              data_num,
                             uint32_t              dummy_clocks);

/* Set autopolling mode */
int32_t qspi_dma_SetAutoPolling(qspi_dma_resources_t* qspi_resources,
                                uint32_t              match,
                                uint32_t              mask,
                                uint8_t               match_mode,
                                uint8_t               status_byte_size,
                                uint16_t              interval,
                                uint8_t               auto_stop);

/* Control QSPI Interface */
int32_t qspi_dma_Control(qspi_dma_resources_t* qspi_resources, uint32_t control, uint32_t arg);

/* Get SPI status */
ARM_QSPI_STATUS qspi_dma_GetStatus(qspi_dma_resources_t* qspi_resources);

/* SPI callback from cmsis dma layer */
void qspi_dma_callback(qspi_dma_resources_t* qspi_resources, uint32_t event);
