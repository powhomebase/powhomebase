/***********************************************************************************************************************
 * Description: STM32 SPI DMA driver header file.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Related Project's headers */
#include <Driver_DMA.h>
#include <Driver_GPIO.h>
#include <Driver_SPI.h>
#include <global.h>

/* Libraries */
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macro                                                                                                              */
/**********************************************************************************************************************/

#if defined(__HAL_RCC_SPI1_CLK_SLEEP_ENABLE)
#define _HAL_RCC_SPI_CLK_SLEEP_DISABLE(num_) __HAL_RCC_SPI##num_##_CLK_SLEEP_DISABLE()
#define _HAL_RCC_SPI_CLK_SLEEP_ENABLE(num_)  __HAL_RCC_SPI##num_##_CLK_SLEEP_ENABLE()
#else
#define _HAL_RCC_SPI_CLK_SLEEP_DISABLE(num_) {}
#define _HAL_RCC_SPI_CLK_SLEEP_ENABLE(num_) {}
#endif /* __HAL_RCC_SPI1_CLK_SLEEP_ENABLE */

/***********************************************************************************************************************
 * Description    : Generate a SPI DMA driver object.
 * Input          : num_           - peripheral number.
 *                  user_conf_     - user configuration.
 *                  gpio_conf_     - gpios configuration.
 *                  dma_tx_handle_ - dma transmission handle.
 *                  dma_tx_driver_ - dma transmission driver.
 *                  dma_rx_handle_ - dma reception handle.
 *                  dma_rx_driver_ - dma reception driver.
 ***********************************************************************************************************************/
#define SPI_DMA_GENERATE_OBJECT(                                                                                       \
    num_, user_conf_, gpios_conf_, dma_tx_handle_, dma_tx_driver_, dma_rx_handle_, dma_rx_driver_)                     \
    _SPI_DMA_GENERATE_CLKS(num_)                                                                                       \
    _SPI_DMA_GENERATE_OBJECT(num_,                                                                                     \
                             user_conf_,                                                                               \
                             gpios_conf_,                                                                              \
                             dma_tx_handle_,                                                                           \
                             dma_tx_driver_,                                                                           \
                             dma_rx_handle_,                                                                           \
                             dma_rx_driver_,                                                                           \
                             HAL_RCC_GetPCLK1Freq)

#define _SPI_DMA_GENERATE_OBJECT(                                                                                      \
    num_, user_conf_, gpios_conf_, dma_tx_handle_, dma_tx_driver_, dma_rx_handle_, dma_rx_driver_, get_freq_)          \
    extern spi_dma_resources_t SPI##num_##_RESOURCES;                                                                  \
    static int32_t             SPI##num_##_Initialize(ARM_SPI_SignalEvent_t cb_event)                                  \
    {                                                                                                                  \
        return spi_dma_Initialize(&SPI##num_##_RESOURCES, cb_event);                                                   \
    }                                                                                                                  \
    static int32_t SPI##num_##_Uninitialize(void)                                                                      \
    {                                                                                                                  \
        return spi_dma_Uninitialize(&SPI##num_##_RESOURCES);                                                           \
    }                                                                                                                  \
    static int32_t SPI##num_##_PowerControl(ARM_POWER_STATE state)                                                     \
    {                                                                                                                  \
        return spi_dma_PowerControl(&SPI##num_##_RESOURCES, state);                                                    \
    }                                                                                                                  \
    static int32_t SPI##num_##_Send(const void *data, uint32_t num)                                                    \
    {                                                                                                                  \
        return spi_dma_Send(&SPI##num_##_RESOURCES, data, num);                                                        \
    }                                                                                                                  \
    static int32_t SPI##num_##_Receive(void *data, uint32_t num)                                                       \
    {                                                                                                                  \
        return spi_dma_Receive(&SPI##num_##_RESOURCES, data, num);                                                     \
    }                                                                                                                  \
    static int32_t SPI##num_##_Transfer(const void *data_out, void *data_in, uint32_t num)                             \
    {                                                                                                                  \
        return spi_dma_Transfer(&SPI##num_##_RESOURCES, data_out, data_in, num);                                       \
    }                                                                                                                  \
    static uint32_t SPI##num_##_GetDataCount(void)                                                                     \
    {                                                                                                                  \
        return spi_dma_GetDataCount(&SPI##num_##_RESOURCES);                                                           \
    }                                                                                                                  \
    static int32_t SPI##num_##_Control(uint32_t control, uint32_t arg)                                                 \
    {                                                                                                                  \
        return spi_dma_Control(&SPI##num_##_RESOURCES, control, arg);                                                  \
    }                                                                                                                  \
    static ARM_SPI_STATUS SPI##num_##_GetStatus(void)                                                                  \
    {                                                                                                                  \
        return spi_dma_GetStatus(&SPI##num_##_RESOURCES);                                                              \
    }                                                                                                                  \
    static void SPI##num_##_cs_interrupt_cb(void)                                                                      \
    {                                                                                                                  \
        return spi_dma_cs_interrupt_callback(&SPI##num_##_RESOURCES);                                                  \
    }                                                                                                                  \
                                                                                                                       \
    const ARM_DRIVER_SPI Driver_SPI##num_ = {                                                                          \
        .Initialize   = SPI##num_##_Initialize,                                                                        \
        .Uninitialize = SPI##num_##_Uninitialize,                                                                      \
        .PowerControl = SPI##num_##_PowerControl,                                                                      \
        .Send         = SPI##num_##_Send,                                                                              \
        .Receive      = SPI##num_##_Receive,                                                                           \
        .Transfer     = SPI##num_##_Transfer,                                                                          \
        .GetDataCount = SPI##num_##_GetDataCount,                                                                      \
        .Control      = SPI##num_##_Control,                                                                           \
        .GetStatus    = SPI##num_##_GetStatus,                                                                         \
    };                                                                                                                 \
                                                                                                                       \
    spi_dma_resources_t SPI##num_##_RESOURCES = {                                                                      \
        .user_conf       = &user_conf_,                                                                                \
        .gpios           = &gpios_conf_,                                                                               \
        .clock_enable    = enable_spi##num_##_clock,                                                                   \
        .clock_disable   = disable_spi##num_##_clock,                                                                  \
        .state           = SPI_NOT_INITIALIZED,                                                                        \
        .dma_tx          = &dma_tx_driver_,                                                                            \
        .dma_rx          = &dma_rx_driver_,                                                                            \
        .dma_tx_hal_st   = &dma_tx_handle_,                                                                            \
        .dma_rx_hal_st   = &dma_rx_handle_,                                                                            \
        .cs_interrupt_cb = SPI##num_##_cs_interrupt_cb,                                                                \
        .get_clk_freq_   = get_freq_,                                                                                  \
    }

#define _SPI_DMA_GENERATE_CLKS(num_)                                                                                   \
    void enable_spi##num_##_clock(void)                                                                                \
    {                                                                                                                  \
        __HAL_RCC_SPI##num_##_CLK_ENABLE();                                                                            \
        _HAL_RCC_SPI_CLK_SLEEP_ENABLE(num_);                                                                           \
    }                                                                                                                  \
    void disable_spi##num_##_clock(void)                                                                               \
    {                                                                                                                  \
        __HAL_RCC_SPI##num_##_CLK_DISABLE();                                                                           \
        _HAL_RCC_SPI_CLK_SLEEP_DISABLE(num_);                                                                          \
    }

#if defined(STM32WL)
/***********************************************************************************************************************
 * Description    : Generate a SPI DMA driver object specifically for the SUBGHZ SPI.
 * Input          : num_           - peripheral number.
 *                  user_conf_     - user configuration.
 *                  gpio_conf_     - gpios configuration.
 *                  dma_tx_handle_ - dma transmission handle.
 *                  dma_tx_driver_ - dma transmission driver.
 *                  dma_rx_handle_ - dma reception handle.
 *                  dma_rx_driver_ - dma reception driver.
 ***********************************************************************************************************************/
#define SUBGHZSPI_DMA_GENERATE_OBJECT(                                                                                 \
    user_conf_, gpios_conf_, dma_tx_handle_, dma_tx_driver_, dma_rx_handle_, dma_rx_driver_)                           \
    _SUBGHZSPI_DMA_GENERATE_CLKS(SUBGHZ)                                                                               \
    _SPI_DMA_GENERATE_OBJECT(SUBGHZ,                                                                                   \
                             user_conf_,                                                                               \
                             gpios_conf_,                                                                              \
                             dma_tx_handle_,                                                                           \
                             dma_tx_driver_,                                                                           \
                             dma_rx_handle_,                                                                           \
                             dma_rx_driver_,                                                                           \
                             HAL_RCC_GetHCLK3Freq)

/* Note: the `name_` here is only used so that this will be compatible with th `_SPI_DMA_GENERATE_OBJECT` macro */
#define _SUBGHZSPI_DMA_GENERATE_CLKS(name_)                                                                            \
    void enable_spi##name_##_clock(void)                                                                               \
    {                                                                                                                  \
        __HAL_RCC_SUBGHZSPI_CLK_ENABLE();                                                                              \
        __HAL_RCC_SUBGHZSPI_CLK_SLEEP_ENABLE();                                                                        \
    }                                                                                                                  \
    void disable_spi##name_##_clock(void)                                                                              \
    {                                                                                                                  \
        __HAL_RCC_SUBGHZSPI_CLK_DISABLE();                                                                             \
        __HAL_RCC_SUBGHZSPI_CLK_SLEEP_DISABLE();                                                                       \
    }
#endif

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*clock_control)(void);

/* Get clock frequency function */
typedef uint32_t (*get_clk_freq_t)(void);

typedef enum {
    SPI_NOT_INITIALIZED = 0b00,
    SPI_NOT_POWERED     = 0b01,
    SPI_POWERED         = 0b11,
} spi_dma_state_t;

/* SPI Pins Configuration */
typedef struct {
    ARM_GPIO_PIN *mosi;
    uint32_t      mosi_conf;
    ARM_GPIO_PIN *miso;
    uint32_t      miso_conf;
    ARM_GPIO_PIN *cs;
    uint32_t      cs_conf;
    ARM_GPIO_PIN *clk;
    uint32_t      clk_conf;
    uint8_t       alternate_f;
} spi_dma_gpios_t;

/* User configuration struct (to be declared and altered in Driver_objs) */
typedef struct {
    SPI_TypeDef *                instance;                /* SPI registers base address */
    SPI_InitTypeDef const *const init;                    /* SPI communication parameters */
    uint32_t                     max_allowed_baudrate_hz; /* Maximun allowed SPI baud rate */
    bool                         enforce_full_transfers;  /* Allow only complete transfers */
} spi_dma_user_conf_t;

/* Resources struct (to be declared in the macro) */
typedef struct {
    /* SPI register interface */
    /*this must be the struct itself (and not a pointer) so we could upcast the handle pointer to the resource pointer
     * in the implementation of the weak function*/
    SPI_HandleTypeDef     spi_handle;
    spi_dma_user_conf_t * user_conf;       /* SPI user configurations */
    spi_dma_gpios_t *     gpios;           /* SPI pins configuration */
    clock_control         clock_enable;    /* SPI Clock enable function */
    clock_control         clock_disable;   /* SPI Clock disable function */
    spi_dma_state_t       state;           /* This layer state machine */
    ARM_DMA_SignalEvent_t dma_tx_cb;       /* This layer irq */
    ARM_DMA_SignalEvent_t dma_rx_cb;       /* This layer irq */
    ARM_SPI_STATUS        status;          /* SPI status */
    ARM_SPI_SignalEvent_t callback;        /* Callback function */
    ARM_DRIVER_DMA *      dma_tx;          /* DMA tx cmsis object */
    ARM_DRIVER_DMA *      dma_rx;          /* DMA rx cmsis object */
    DMA_HandleTypeDef *   dma_tx_hal_st;   /* DMA tx hal st object */
    DMA_HandleTypeDef *   dma_rx_hal_st;   /* DMA rx hal st object */
    ARM_GPIO_Interrupt_t  cs_interrupt_cb; /* Helpers for 'full transfers' configuration */
    bool                  cs_active;
    get_clk_freq_t        get_clk_freq_;
} spi_dma_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

/* Initialize SPI interface */
int32_t spi_dma_Initialize(spi_dma_resources_t *p_this, ARM_SPI_SignalEvent_t cb);

/* De-initialize SPI interface */
int32_t spi_dma_Uninitialize(spi_dma_resources_t *p_this);

/* Control SPI Interface Power */
int32_t spi_dma_PowerControl(spi_dma_resources_t *p_this, ARM_POWER_STATE state);

/* Start sending data to SPI transmitter */
int32_t spi_dma_Send(spi_dma_resources_t *p_this, const void *data, uint32_t num);

/* Start receive data with SPI */
int32_t spi_dma_Receive(spi_dma_resources_t *p_this, void *data, uint32_t num);

/* Start sending / receiving data to / from SPI transmitter receiver */
int32_t spi_dma_Transfer(spi_dma_resources_t *p_this, const void *data_out, void *data_in, uint32_t num);

/* Get transferred data count */
uint32_t spi_dma_GetDataCount(spi_dma_resources_t *p_this);

/* Control SPI Interface */
int32_t spi_dma_Control(spi_dma_resources_t *p_this, uint32_t control, uint32_t arg);

/* Get SPI status */
ARM_SPI_STATUS spi_dma_GetStatus(spi_dma_resources_t *p_this);

/* CS pin callback */
void spi_dma_cs_interrupt_callback(spi_dma_resources_t *p_this);