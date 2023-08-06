/***********************************************************************************************************************
 * Description: STM32 OSPI DMA driver header file. Implementation assumes OCTOSPI isn't being used by STM32L4 boards.
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
 * Description    : Generate a OSPI DMA driver object.
 * Input          : num_        - peripheral number.
 *                  user_conf_  - user configuration.
 *                  gpio_conf_  - gpios configuration.
 *                  dma_driver_ - dma transmission driver.
 *                  dma_handle_ - dma transmission handle.
 ***********************************************************************************************************************/
#define OSPI_DMA_GENERATE_OBJECT(num_, user_conf_, gpios_conf_, dma_driver_, dma_handle_)                              \
    extern ospi_dma_resources_t OSPI##num_##_RESOURCES;                                                                \
    static int32_t              OSPI##num_##_Initialize(ARM_SPI_SignalEvent_t cb_event)                                \
    {                                                                                                                  \
        return ospi_dma_Initialize(&OSPI##num_##_RESOURCES, cb_event);                                                 \
    }                                                                                                                  \
    static int32_t OSPI##num_##_Uninitialize(void)                                                                     \
    {                                                                                                                  \
        return ospi_dma_Uninitialize(&OSPI##num_##_RESOURCES);                                                         \
    }                                                                                                                  \
    static int32_t OSPI##num_##_PowerControl(ARM_POWER_STATE state)                                                    \
    {                                                                                                                  \
        return ospi_dma_PowerControl(&OSPI##num_##_RESOURCES, state);                                                  \
    }                                                                                                                  \
    static int32_t OSPI##num_##_SendCommand(uint32_t cmd_modes,                                                        \
                                            uint32_t instruction,                                                      \
                                            uint32_t address,                                                          \
                                            uint32_t alternate_bytes,                                                  \
                                            void*    data,                                                             \
                                            uint32_t data_num,                                                         \
                                            uint32_t dummy_clocks)                                                     \
    {                                                                                                                  \
        return ospi_dma_SendCommand(                                                                                   \
            &OSPI##num_##_RESOURCES, cmd_modes, instruction, address, alternate_bytes, data, data_num, dummy_clocks);  \
    }                                                                                                                  \
    static int32_t OSPI##num_##_SetAutoPolling(uint32_t match,                                                         \
                                               uint32_t mask,                                                          \
                                               uint8_t  match_mode,                                                    \
                                               uint8_t  status_byte_size,                                              \
                                               uint16_t interval,                                                      \
                                               uint8_t  auto_stop)                                                     \
    {                                                                                                                  \
        return ospi_dma_SetAutoPolling(                                                                                \
            &OSPI##num_##_RESOURCES, match, mask, match_mode, status_byte_size, interval, auto_stop);                  \
    }                                                                                                                  \
    static int32_t OSPI##num_##_Control(uint32_t control, uint32_t arg)                                                \
    {                                                                                                                  \
        return ospi_dma_Control(&OSPI##num_##_RESOURCES, control, arg);                                                \
    }                                                                                                                  \
    static ARM_QSPI_STATUS OSPI##num_##_GetStatus(void)                                                                \
    {                                                                                                                  \
        return ospi_dma_GetStatus(&OSPI##num_##_RESOURCES);                                                            \
    }                                                                                                                  \
                                                                                                                       \
    void OCTO##num_##SPI_IRQHandler(void)                                                                              \
    {                                                                                                                  \
        HAL_OSPI_IRQHandler(&OSPI##num_##_RESOURCES.ospi_handle);                                                      \
    }                                                                                                                  \
                                                                                                                       \
    ARM_DRIVER_QSPI Driver_OSPI##num_ = {                                                                              \
        .GetVersion      = FUNCTION_NOT_IMPLEMENTED,                                                                   \
        .GetCapabilities = FUNCTION_NOT_IMPLEMENTED,                                                                   \
        .Initialize      = OSPI##num_##_Initialize,                                                                    \
        .Uninitialize    = OSPI##num_##_Uninitialize,                                                                  \
        .PowerControl    = OSPI##num_##_PowerControl,                                                                  \
        .SendCommand     = OSPI##num_##_SendCommand,                                                                   \
        .SetAutoPolling  = OSPI##num_##_SetAutoPolling,                                                                \
        .Control         = OSPI##num_##_Control,                                                                       \
        .GetStatus       = OSPI##num_##_GetStatus,                                                                     \
    };                                                                                                                 \
                                                                                                                       \
    static uint32_t HAL_RCC_OSPIM_CLK_ENABLED = 0;\
    void enable_ospi_clock(void)                                                                                       \
    {                                                                                                                  \
        __HAL_RCC_OSPI##num_##_CLK_ENABLE();                                                                                \
        __HAL_RCC_OSPI##num_##_CLK_SLEEP_ENABLE();                                                                                \
        OSPIM_CLK_ENABLE(); \
    }                                                                                                                  \
    void disable_ospi_clock(void)                                                                                      \
    {                                                                                                                  \
        __HAL_RCC_OSPI##num_##_CLK_SLEEP_DISABLE();                                                                         \
        __HAL_RCC_OSPI##num_##_CLK_DISABLE();                                                                               \
        OSPIM_CLK_DISABLE(); \
    }\
    \
        ospi_dma_resources_t OSPI##num_##_RESOURCES = {                                                                    \
        .p_user_conf   = &user_conf_,                                                                                  \
        .gpios         = &gpios_conf_,                                                                                 \
        .clock_enable  = enable_ospi_clock,                                                                            \
        .clock_disable = disable_ospi_clock,                                                                           \
        .state         = OSPI_NOT_INITIALIZED,                                                                         \
        .dma           = &dma_driver_,                                                                                 \
        .dma_hal_st    = &dma_handle_,                                                                                 \
    };

#if defined(__HAL_RCC_OSPIM_CLK_SLEEP_ENABLE)
#define OSPIM_CLK_ENABLE()
    HAL_RCC_OSPIM_CLK_ENABLED++; \
    if(HAL_RCC_OSPIM_CLK_ENABLED == 1) \
    { \
        __HAL_RCC_OSPIM_CLK_ENABLE(); \
        __HAL_RCC_OSPIM_CLK_SLEEP_ENABLE(); \
    }      

#define OSPIM_CLK_DISABLE()
    HAL_RCC_OSPIM_CLK_ENABLED--; \
    if(HAL_RCC_OSPIM_CLK_ENABLED == 0) \
    { \
        __HAL_RCC_OSPIM_CLK_DISABLE(); \
        __HAL_RCC_OSPIM_CLK_SLEEP_DISABLE(); \
    }     
#else                  
#define OSPIM_CLK_ENABLE()\
    HAL_RCC_OSPIM_CLK_ENABLED++; \
    if(HAL_RCC_OSPIM_CLK_ENABLED == 1) \
    { \
        __HAL_RCC_OSPIM_CLK_ENABLE(); \
    }      

#define OSPIM_CLK_DISABLE()\
    HAL_RCC_OSPIM_CLK_ENABLED--; \
    if(HAL_RCC_OSPIM_CLK_ENABLED == 0) \
    { \
        __HAL_RCC_OSPIM_CLK_DISABLE(); \
    }  
#endif


/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*clock_control)(void);

typedef enum {
    OSPI_NOT_INITIALIZED,
    OSPI_NOT_POWERED,
    OSPI_POWERED,
    OSPI_SENDCMD,
    OSPI_AUTOPOLLING_SET,
} ospi_dma_state_t;

/* OSPI Pins Configuration */
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
} const ospi_dma_gpios_t;

/* User configuration struct (to be declared and altered in Driver_objs) */
typedef struct {
    /* SPI registers base address */
    OCTOSPI_TypeDef* instance;
    /* SPI communication parameters */
    OSPI_InitTypeDef const* const init;
    /* OSPI IRQ number */
    IRQn_Type irq;
} const ospi_dma_user_conf_t;

/* Resources struct (to be declared in the macro) */
typedef struct {
    /* SPI register interface */
    /*this must be the struct itself (and not a pointer) so we could upcast the handle pointer to the resource pointer
     * in the implementation of the weak function*/
    OSPI_HandleTypeDef ospi_handle;
    /* SPI user configurations */
    ospi_dma_user_conf_t* p_user_conf;
    /* SPI pins configuration */
    ospi_dma_gpios_t* gpios;
    /* SPI Clock enable function */
    clock_control clock_enable;
    /* SPI Clock disable function */
    clock_control clock_disable;
    /* This layer state machine */
    ospi_dma_state_t state;
    /* SPI status */
    ARM_QSPI_STATUS status;
    /* Callback function */
    ARM_QSPI_SignalEvent_t callback;
    /* DMA cmsis object */
    ARM_DRIVER_DMA* dma;
    /* DMA hal st object */
    DMA_HandleTypeDef* dma_hal_st;
    /* Current Autopolling config struct */
    OSPI_AutoPollingTypeDef autopolling_conf;
} ospi_dma_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

/* Initialize OSPI interface */
int32_t ospi_dma_Initialize(ospi_dma_resources_t* ospi_resources, ARM_QSPI_SignalEvent_t cb_event);

/* De-initialize SPI interface */
int32_t ospi_dma_Uninitialize(ospi_dma_resources_t* ospi_resources);

/* Control SPI Interface Power */
int32_t ospi_dma_PowerControl(ospi_dma_resources_t* ospi_resources, ARM_POWER_STATE state);

/* Send a command using OSPI */
int32_t ospi_dma_SendCommand(ospi_dma_resources_t* ospi_resources,
                             uint32_t              cmd_modes,
                             uint32_t              instruction,
                             uint32_t              address,
                             uint32_t              alternate_bytes,
                             void*                 data,
                             uint32_t              data_num,
                             uint32_t              dummy_clocks);

/* Set autopolling mode */
int32_t ospi_dma_SetAutoPolling(ospi_dma_resources_t* ospi_resources,
                                uint32_t              match,
                                uint32_t              mask,
                                uint8_t               match_mode,
                                uint8_t               status_byte_size,
                                uint16_t              interval,
                                uint8_t               auto_stop);

/* Control OSPI Interface */
int32_t ospi_dma_Control(ospi_dma_resources_t* ospi_resources, uint32_t control, uint32_t arg);

/* Get SPI status */
ARM_QSPI_STATUS ospi_dma_GetStatus(ospi_dma_resources_t* ospi_resources);

/* SPI callback from cmsis dma layer */
void ospi_dma_callback(ospi_dma_resources_t* ospi_resources, uint32_t event);
