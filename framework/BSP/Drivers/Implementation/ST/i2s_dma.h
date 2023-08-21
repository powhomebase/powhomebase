/***********************************************************************************************************************
 * Description: STM32 I2S (SAI).
 **********************************************************************************************************************/

#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project related headers */
#include <Driver_DMA.h>
#include <Driver_GPIO.h>
#include <Driver_SAI.h>
#include <global.h>
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#if defined(STM32L4)
#define SAI_I2S_GENERATE_RCC_PERIPH_INIT_STRUCT(i2s_num, PLL_N, PLL_P, pll_num, sai_periph_num) \
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct##i2s_num = {                                                      \
        .PeriphClockSelection = RCC_PERIPHCLK_SAI##sai_periph_num,                                                     \
        .PLLSAI##pll_num =                                                                                             \
            {                                                                                                          \
                .PLLSAI##pll_num##Source   = RCC_PLLCFGR_PLLSRC_MSI,                                                   \
                .PLLSAI##pll_num##M        = 1,                                                                        \
                .PLLSAI##pll_num##N        = PLL_N,                                                                    \
                .PLLSAI##pll_num##P        = PLL_P,                                                                    \
                .PLLSAI##pll_num##ClockOut = RCC_PLLSAI##pll_num##_SAI##pll_num##CLK,                                  \
            },                                                                                                         \
        .Sai##sai_periph_num##ClockSelection = RCC_SAI##sai_periph_num##CLKSOURCE_PLLSAI##pll_num,                     \
    }
#elif defined(STM32U5)
#define SAI_I2S_GENERATE_RCC_PERIPH_INIT_STRUCT(i2s_num, PLL_N, PLL_P, pll_num, sai_periph_num) \
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct##i2s_num = {                                                      \
        .PeriphClockSelection = RCC_PERIPHCLK_SAI##sai_periph_num,                                                     \
        .PLL##pll_num =                                                                                                \
            {                                                                                                          \
                .PLL##pll_num##Source   = RCC_PLLSOURCE_MSI     ,                                                      \
                .PLL##pll_num##M        = 1,                                                                           \
                .PLL##pll_num##N        = PLL_N,                                                                       \
                .PLL##pll_num##P        = PLL_P,                                                                       \
                .PLL##pll_num##ClockOut = RCC_PLL##pll_num##_DIVP,                                                     \
            },                                                                                                         \
        .Sai##sai_periph_num##ClockSelection = RCC_SAI##sai_periph_num##CLKSOURCE_PLL##pll_num,                        \
    }
#endif

#define SAI_I2S_DMA_GENERATE_OBJECT(i2s_num,                                                                           \
                                    _gpios_conf,                                                                       \
                                    _user_conf,                                                                        \
                                    dma_driver,                                                                        \
                                    dma_handle,                                                                        \
                                    PLL_N,                                                                             \
                                    PLL_P,                                                                             \
                                    pll_num,                                                                           \
                                    sai_periph_num)                                                                    \
                                                                                                                       \
    extern i2s_dma_resources_t SAI_I2S##i2s_num##_RESOURCES;                                                           \
                                                                                                                       \
    static int32_t SAI_I2S##i2s_num##_Initialize(ARM_SAI_SignalEvent_t cb_event)                                       \
    {                                                                                                                  \
        return i2s_dma_Initialize(cb_event, &SAI_I2S##i2s_num##_RESOURCES);                                            \
    }                                                                                                                  \
    static int32_t SAI_I2S##i2s_num##_Uninitialize(void)                                                               \
    {                                                                                                                  \
        return i2s_dma_Uninitialize(&SAI_I2S##i2s_num##_RESOURCES);                                                    \
    }                                                                                                                  \
    static int32_t SAI_I2S##i2s_num##_PowerControl(ARM_POWER_STATE state)                                              \
    {                                                                                                                  \
        return i2s_dma_PowerControl(state, &SAI_I2S##i2s_num##_RESOURCES);                                             \
    }                                                                                                                  \
    static int32_t SAI_I2S##i2s_num##_Send(const void* data, uint32_t num)                                             \
    {                                                                                                                  \
        return i2s_dma_Send(data, num, &SAI_I2S##i2s_num##_RESOURCES);                                                 \
    }                                                                                                                  \
    static int32_t SAI_I2S##i2s_num##_Receive(void* data, uint32_t num)                                                \
    {                                                                                                                  \
        return i2s_dma_Receive(data, num, &SAI_I2S##i2s_num##_RESOURCES);                                              \
    }                                                                                                                  \
    static uint32_t SAI_I2S##i2s_num##_GetTxCount(void)                                                                \
    {                                                                                                                  \
        return i2s_dma_GetTxCount(&SAI_I2S##i2s_num##_RESOURCES);                                                      \
    }                                                                                                                  \
    static uint32_t SAI_I2S##i2s_num##_GetRxCount(void)                                                                \
    {                                                                                                                  \
        return i2s_dma_GetRxCount(&SAI_I2S##i2s_num##_RESOURCES);                                                      \
    }                                                                                                                  \
    static int32_t SAI_I2S##i2s_num##_Control(uint32_t control, uint32_t arg1, uint32_t arg2)                          \
    {                                                                                                                  \
        return i2s_dma_Control(control, arg1, arg2, &SAI_I2S##i2s_num##_RESOURCES);                                    \
    }                                                                                                                  \
    static ARM_SAI_STATUS SAI_I2S##i2s_num##_GetStatus(void)                                                           \
    {                                                                                                                  \
        return i2s_dma_GetStatus(&SAI_I2S##i2s_num##_RESOURCES);                                                       \
    }                                                                                                                  \
                                                                                                                       \
    ARM_DRIVER_SAI Driver_I2S##i2s_num = {                                                                             \
        .GetVersion      = FUNCTION_NOT_IMPLEMENTED,                                                                   \
        .GetCapabilities = FUNCTION_NOT_IMPLEMENTED,                                                                   \
        .Initialize      = SAI_I2S##i2s_num##_Initialize,                                                              \
        .Uninitialize    = SAI_I2S##i2s_num##_Uninitialize,                                                            \
        .PowerControl    = SAI_I2S##i2s_num##_PowerControl,                                                            \
        .Send            = SAI_I2S##i2s_num##_Send,                                                                    \
        .Receive         = SAI_I2S##i2s_num##_Receive,                                                                 \
        .GetTxCount      = SAI_I2S##i2s_num##_GetTxCount,                                                              \
        .GetRxCount      = SAI_I2S##i2s_num##_GetRxCount,                                                              \
        .Control         = SAI_I2S##i2s_num##_Control,                                                                 \
        .GetStatus       = SAI_I2S##i2s_num##_GetStatus,                                                               \
    };                                                                                                                 \
                                                                                                                       \
    SAI_I2S_GENERATE_RCC_PERIPH_INIT_STRUCT(i2s_num, PLL_N, PLL_P, pll_num, sai_periph_num);                           \
                                                                                                                       \
    i2s_dma_resources_t SAI_I2S##i2s_num##_RESOURCES = {                                                               \
        .state                       = SAI_I2S_NOT_INITIALIZED,                                                        \
        .gpios                       = _gpios_conf,                                                                    \
        .user_conf                   = _user_conf,                                                                     \
        .dma                         = dma_driver,                                                                     \
        .hdma                        = dma_handle,                                                                     \
        .RCC_PeriphCLKInitStruct_sai = &RCC_PeriphCLKInitStruct##i2s_num,                                              \
        .sai_num                     = SAI##sai_periph_num##_e,                                                        \
        .saipll_num                  = PLL##pll_num##_e,                                                               \
    };


/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*clock_control)(void);

typedef enum {
    SAI_I2S_NOT_INITIALIZED = 0,
    SAI_I2S_INITIALIZED     = 1,
    SAI_I2S_POWERED         = 2,
} sai_i2s_state_e;

typedef enum _SAI_MODE {
    RX = 0,
    TX,
} SAI_MODE;

typedef enum {
    SAI1_e = 1,
    SAI2_e = 2,
} sai_num_t;

typedef enum {
    PLL1_e = 1,
    PLL2_e = 2,
} saipll_num_t;

/* I2S Pins Configuration */
typedef struct {
    /* I2S SCK*/
    ARM_GPIO_PIN* sck;
    /* I2S SD */
    ARM_GPIO_PIN* sd;
    /* I2S FS */
    ARM_GPIO_PIN* fs;
    /* I2S MCLK */
    ARM_GPIO_PIN* mclk;
    /* Alternate function */
    uint8_t alternate_f;
} i2s_int_gpios_t;

typedef struct {
    /*initialization struct */
    SAI_InitTypeDef* init;
    /* Number of I2S data bits */
    uint32_t dataSize;
    /* Justification of I2S data within the frame */
    uint32_t protocol;
    /* instance */
    SAI_Block_TypeDef* instance;
    /*number of slots*/
    uint8_t num_of_slots;
    /*with or without Master CLK*/
    bool enable_master_clk;
} i2s_dma_user_conf_t;

typedef struct {
    /* ST I2S handle */
    /* this must be the struct itself (and not a pointer) so we could upcast the handle pointer to the resource pointer
       in the implementation of the weak function */
    SAI_HandleTypeDef i2s_handle;
    /* Driver state */
    sai_i2s_state_e state;
    /*  GPIOs */
    i2s_int_gpios_t* gpios;
    /* User configurations */
    i2s_dma_user_conf_t* user_conf;
    /*  DMA driver */
    ARM_DRIVER_DMA* dma;
    /* SAI DMA Handle parameters */
    DMA_HandleTypeDef* hdma;
    /*peripheral clock settings*/
    RCC_PeriphCLKInitTypeDef* RCC_PeriphCLKInitStruct_sai;
    /*sai peripheral number*/
    sai_num_t sai_num;
    /*sai pll number*/
    saipll_num_t saipll_num;
    /* USART status */
    ARM_SAI_STATUS status;
    /* Callback function */
    ARM_SAI_SignalEvent_t callback;
    /* SAI is RX or TX*/
    SAI_MODE transmitDirection;
} i2s_dma_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

int32_t        i2s_dma_Initialize(ARM_SAI_SignalEvent_t cb_event, i2s_dma_resources_t* i2s_resources);
int32_t        i2s_dma_Uninitialize(i2s_dma_resources_t* i2s_resources);
int32_t        i2s_dma_PowerControl(ARM_POWER_STATE state, i2s_dma_resources_t* i2s_resources);
int32_t        i2s_dma_Send(const void* data, uint32_t num, i2s_dma_resources_t* i2s_resources);
int32_t        i2s_dma_Receive(void* data, uint32_t num, i2s_dma_resources_t* i2s_resources);
uint32_t       i2s_dma_GetTxCount(i2s_dma_resources_t* i2s_resources);
uint32_t       i2s_dma_GetRxCount(i2s_dma_resources_t* i2s_resources);
int32_t        i2s_dma_Control(uint32_t control, uint32_t arg1, uint32_t arg2, i2s_dma_resources_t* i2s_resources);
ARM_SAI_STATUS i2s_dma_GetStatus(i2s_dma_resources_t* i2s_resources);