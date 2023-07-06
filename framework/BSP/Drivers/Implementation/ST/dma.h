/***********************************************************************************************************************
 * Description : STM32 dma driver header file.
 * Notes       :
 *               IMPORTANT: To simplify the driver, we decided to leave implementing the IRQ to the
 *               end-user, simply implement the IRQ handler of each DMA you use:
 *               For example, if instances 4 and 5 in block 2 share an IRQ -
 *
 *                 void DMA2_Instance4_5_IRQHandler(void)
 *                 {
 *                     // Leave the appropriate ones, depending on which instance is in use.
 *                     HAL_DMA_IRQHandler(&DMA2_4_RESOURCES->dma_handle);
 *                     HAL_DMA_IRQHandler(&DMA2_5_RESOURCES->dma_handle);
 *                 }
 *
 *               In other DMA blocks and instances -
 *
 *                 void DMA##dma_block##_Instance##dma_instance##_IRQHandler(void)
 *                 {
 *                      HAL_DMA_IRQHandler((&DMA##dma_num##_##dma_instance##_RESOURCES)->dma_handle);
 *                 }
 *
 * Simply copy the relevant template, to your objs file.
 *
 * Note that in the stm32F4 there are streams and channels. This means that you'd need to specify the channel used in
 * the DMA_InitTypeDef struct in your board's 'bsp_obj.c', in addition to defining the dma instance in dma_user_conf_t.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project Related */
#include CMSIS_device_header
#include <Driver_DMA.h>
#include <cmsis_os.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description : DMA generating macro.
 * Input       : block_     - dma block number.
 *               instance_   - dma instance. In stm32F4, this is actually the dma stream.
 *               user_conf_ - dma user configuration (of type dma_user_conf_t).
 **********************************************************************************************************************/
#define DMA_GENERATE_OBJECT(block_, instance_, user_conf_)                                                             \
    extern dma_resources_t DMA##block_##_##instance_##_RESOURCES;                                                      \
    static int32_t         DMA##block_##_##instance_##_Initialize(ARM_DMA_SignalEvent_t cb_event)                      \
    {                                                                                                                  \
        return dma_Initialize(&DMA##block_##_##instance_##_RESOURCES, cb_event);                                       \
    }                                                                                                                  \
    static int32_t DMA##block_##_##instance_##_Uninitialize(void)                                                      \
    {                                                                                                                  \
        return dma_Uninitialize(&DMA##block_##_##instance_##_RESOURCES);                                               \
    }                                                                                                                  \
    static int32_t DMA##block_##_##instance_##_PowerControl(ARM_POWER_STATE state)                                     \
    {                                                                                                                  \
        return dma_PowerControl(&DMA##block_##_##instance_##_RESOURCES, state);                                        \
    }                                                                                                                  \
    static int32_t DMA##block_##_##instance_##_Transfer(const void* source, void* dest, uint32_t num)                  \
    {                                                                                                                  \
        return dma_Transfer(&DMA##block_##_##instance_##_RESOURCES, (uint32_t*)source, dest, num);                     \
    }                                                                                                                  \
    static uint32_t DMA##block_##_##instance_##_GetTransferCount(void)                                                 \
    {                                                                                                                  \
        return 0; /* Not yet implemented */                                                                            \
    }                                                                                                                  \
    static int32_t DMA##block_##_##instance_##_Control(uint32_t control, uint32_t arg)                                 \
    {                                                                                                                  \
        return dma_Control(&DMA##block_##_##instance_##_RESOURCES, control, arg);                                      \
    }                                                                                                                  \
    static ARM_DMA_STATUS DMA##block_##_##instance_##_GetStatus(void)                                                  \
    {                                                                                                                  \
        return dma_GetStatus(&DMA##block_##_##instance_##_RESOURCES);                                                  \
    }                                                                                                                  \
                                                                                                                       \
    ARM_DRIVER_DMA Driver_DMA##block_##_##instance_ = {                                                                \
        .Initialize       = DMA##block_##_##instance_##_Initialize,                                                    \
        .Uninitialize     = DMA##block_##_##instance_##_Uninitialize,                                                  \
        .PowerControl     = DMA##block_##_##instance_##_PowerControl,                                                  \
        .Transfer         = DMA##block_##_##instance_##_Transfer,                                                      \
        .GetTransferCount = DMA##block_##_##instance_##_GetTransferCount,                                              \
        .Control          = DMA##block_##_##instance_##_Control,                                                       \
        .GetStatus        = DMA##block_##_##instance_##_GetStatus,                                                     \
    };                                                                                                                 \
                                                                                                                       \
    static DMA_HandleTypeDef DMA##block_##_##instance_##_handle;                                                       \
                                                                                                                       \
    dma_resources_t DMA##block_##_##instance_##_RESOURCES = {                                                          \
        .dma_handle      = &DMA##block_##_##instance_##_handle,                                                        \
        .state           = DMA_NOT_INITIALIZED,                                                                        \
        .user_conf       = &user_conf_,                                                                                \
        .block_num       = DMA_BLOCK_DMA##block_,                                                                      \
        .sems_created    = false,                                                                                      \
        .block1_sem      = NULL,                                                                                       \
        .block2_sem      = NULL,                                                                                       \
        .block1_sem_attr = OS_ATTR_STATIC_SEMAPHORE_CREATE(block1_sem_attr),                                           \
        .block2_sem_attr = OS_ATTR_STATIC_SEMAPHORE_CREATE(block2_sem_attr),                                           \
    };

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef enum {
    DMA_NOT_INITIALIZED,
    DMA_NOT_POWERED,
    DMA_POWERED,
} dma_state_t;

typedef enum {
    DMA_BLOCK_DMA1,
    DMA_BLOCK_DMA2,
} dma_block_num_t;

/* DMA user configuration struct */
typedef struct {
#if defined(DMA1_Stream0)
    DMA_Stream_TypeDef* dma_instance; /* DMA instance - implemented by hardware as a stream */
#else
    DMA_Channel_TypeDef* dma_instance; /* DMA instance - implemented by hardware as a channel */
#endif
    IRQn_Type                    irq;      /* DMA IRQ Handler */
    DMA_InitTypeDef const* const dma_init; /* DMA Configuration Structure definition */
    void*                        Parent;   /* DMA parent object (NULL if not relevant) */
#if defined(DMAMUX1)
    uint32_t const ll_mux_req;     /* DMA request - one of LL_DMAMUX_REQ_* (such as LL_DMAMUX_REQ_SUBGHZSPI_RX) */
    uint32_t const ll_dma_channel; /* The DMA channel - one of LL_DMA_CHANNEL_* */
#endif
} dma_user_conf_t;

/* DMA resources struct*/
typedef struct {
    DMA_HandleTypeDef* dma_handle; /* DMA handle Structure definition */
    dma_state_t        state;      /* Driver state */
    dma_user_conf_t*   user_conf;  /* DMA user configuration struct */
    ARM_DMA_STATUS     status;     /* DMA status */
    dma_block_num_t    block_num;  /* DMA block number */
    bool               sems_created;
    osSemaphoreId_t    block1_sem;
    osSemaphoreId_t    block2_sem;
    osSemaphoreAttr_t  block1_sem_attr;
    osSemaphoreAttr_t  block2_sem_attr;
} dma_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

/* Initialize DMA interface */
int32_t dma_Initialize(dma_resources_t* p_this, ARM_DMA_SignalEvent_t cb_event);

/* De-initialize DMA interface */
int32_t dma_Uninitialize(dma_resources_t* p_this);

/* Control DMA Interface Power */
int32_t dma_PowerControl(dma_resources_t* p_this, ARM_POWER_STATE state);

/* Start transferring data using the DMA */
int32_t dma_Transfer(dma_resources_t* p_this, uint32_t* source, uint32_t* dest, uint32_t data_length);

/* Control DMA Interface */
int32_t dma_Control(dma_resources_t* p_this, uint32_t control, uint32_t arg);

/* Get DMA status */
ARM_DMA_STATUS dma_GetStatus(dma_resources_t* p_this);