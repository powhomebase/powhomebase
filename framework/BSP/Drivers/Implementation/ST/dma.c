/***********************************************************************************************************************
 * Description : STM32 dma driver implementation.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "dma.h"

/* Project Related */
#include <global.h>
#include <proj_exception.h>
#include <syscalls.h>
#include <proj_assert.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* L4,F4 and WL devices has 2 equal blocks of DMA, each one with 8 channels. U5 has 2 blocks of DMA, the GPDMA and
   LPDMA. In order to keep terminology conventions, to GPDMA we will refer as block1 and to LPDMA as block2  */
#if defined(STM32U5)
#define DMA1_MAX_SEMAPHORE_VALUE (15)
#define DMA2_MAX_SEMAPHORE_VALUE (3)
#else
#define DMA1_MAX_SEMAPHORE_VALUE (7)
#define DMA2_MAX_SEMAPHORE_VALUE (7)
#endif


/**********************************************************************************************************************/
/* Private Functions Declarations                                                                                     */
/**********************************************************************************************************************/

static void dma_clock_enable(dma_resources_t* p_this);
static void dma_clock_disable(dma_resources_t* p_this);

#if defined(DMAMUX1)
static void dmamux_clock_enable(void);
static void dmamux_clock_disable(void);
#endif

/**********************************************************************************************************************/
/* Public Functions Implementation                                                                                    */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description : Initialize DMA interface
 * Input       : cb_event - User callback
 *               p_this   - Pointer to interface resources
 * Return      : execution status
 **********************************************************************************************************************/
int32_t dma_Initialize(dma_resources_t* p_this, ARM_DMA_SignalEvent_t cb_event)
{
    ARGUMENT_UNUSED(cb_event);

    /* If it is the first time this driver is initialized create the internal semaphores  */
    if (!p_this->sems_created)
    {
        p_this->sems_created = true;
        p_this->block1_sem = osSemaphoreNew(DMA1_MAX_SEMAPHORE_VALUE, DMA1_MAX_SEMAPHORE_VALUE, &p_this->block1_sem_attr);
        proj_assert(p_this->block1_sem != NULL);

        p_this->block2_sem = osSemaphoreNew(DMA2_MAX_SEMAPHORE_VALUE, DMA2_MAX_SEMAPHORE_VALUE, &p_this->block2_sem_attr);
        proj_assert(p_this->block2_sem != NULL);
    }
    
    p_this->dma_handle->Instance = p_this->user_conf->dma_instance;

    /* If dma is being used by other peripheral (such as SPI) we need to register it to the dma as a parent */
    if (p_this->user_conf->Parent)
    {
        p_this->dma_handle->Parent = p_this->user_conf->Parent;
    }

    dma_clock_enable(p_this);

    #if defined(STM32U5)
    if (p_this->user_conf->dma_linked_list_init != NULL)
    {
        /* If dma_linked_list_init struct is not NULL -> we are in Linked List mode */
        DMA_NodeConfTypeDef dma_node_config = {
            .NodeType = (p_this->block_num == DMA_BLOCK_DMA1) ? DMA_GPDMA_LINEAR_NODE : DMA_LPDMA_LINEAR_NODE,
            .Init = *(p_this->user_conf->dma_init),
        };

        HAL_DMAEx_List_BuildNode(&dma_node_config, &p_this->dma_node);

        HAL_DMAEx_List_InsertNode(&p_this->dma_list, NULL, &p_this->dma_node);

        HAL_DMAEx_List_SetCircularMode(&p_this->dma_list);

        p_this->dma_handle->InitLinkedList = *(p_this->user_conf->dma_linked_list_init);

        HAL_DMAEx_List_Init(p_this->dma_handle);

        HAL_DMAEx_List_LinkQ(p_this->dma_handle, &p_this->dma_list);
    }
    else
    #endif
    {
        /* Prepare the DMA_HandleTypeDef struct for HAL_DMA_Init() */
        p_this->dma_handle->Init = *(p_this->user_conf->dma_init);

        /* For a given stream / channel, program the required configuration through the following parameters:
        Channel request, Transfer Direction, Source and Destination data formats,
        Circular or Normal mode, Stream / Channel Priority level, Source and Destination Increment mode
        using HAL_DMA_Init() function. */
        HAL_DMA_Init(p_this->dma_handle);
    }

#if defined(DMAMUX1)
    LL_DMA_SetPeriphRequest(
        p_this->dma_handle->DmaBaseAddress, p_this->user_conf->ll_dma_channel, p_this->user_conf->ll_mux_req);
#endif

    dma_clock_disable(p_this);

    p_this->state = DMA_NOT_POWERED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : De-initialize DMA interface
 * Input       : p_this - Pointer to interface resources
 * Return      : execution status
 **********************************************************************************************************************/
int32_t dma_Uninitialize(dma_resources_t* p_this)
{
    p_this->state = DMA_NOT_INITIALIZED;

    HAL_DMA_DeInit(p_this->dma_handle);

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
* Description : Control DMA Interface Power
* Input       : state         - Power state
                p_this - Pointer to interface resources
* Return      : execution status
 **********************************************************************************************************************/
int32_t dma_PowerControl(dma_resources_t* p_this, ARM_POWER_STATE state)
{
    if (DMA_NOT_INITIALIZED == p_this->state)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (p_this->state == DMA_NOT_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            p_this->state = DMA_NOT_POWERED;

            dma_clock_disable(p_this);

            nvic_enable_irq(p_this->user_conf->irq, false);

            return ARM_DRIVER_OK;
        }

        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        case ARM_POWER_FULL:
        {
            if (p_this->state == DMA_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            nvic_enable_irq(p_this->user_conf->irq, true);

            p_this->state = DMA_POWERED;

            dma_clock_enable(p_this);

            return ARM_DRIVER_OK;
        }

        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}

/***********************************************************************************************************************
* Description : Start transferring data using the DMA
* Input       : source      - Pointer to buffer with data to send
                dest        - Pointer to buffer receiving data
                data_length - Number of data items to transfer
                p_this      - Pointer to interface resources
* Return      : execution status
 **********************************************************************************************************************/
int32_t dma_Transfer(dma_resources_t* p_this, uint32_t* source, uint32_t* dest, uint32_t data_length)
{
    ARGUMENT_UNUSED(source);
    ARGUMENT_UNUSED(dest);
    ARGUMENT_UNUSED(data_length);
    ARGUMENT_UNUSED(p_this);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
* Description : Control DMA Interface
* Input       : control - Operation
                arg     - Argument of operation (optional)
                p_this  - Pointer to interface resources
* Return      : execution status
 **********************************************************************************************************************/
int32_t dma_Control(dma_resources_t* p_this, uint32_t control, uint32_t arg)
{
    ARGUMENT_UNUSED(control);
    ARGUMENT_UNUSED(arg);
    ARGUMENT_UNUSED(p_this);

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description : Get dma status
 * Input       : p_this - Pointer to interface resources
 * Return      : Driver's status
 **********************************************************************************************************************/
ARM_DMA_STATUS dma_GetStatus(dma_resources_t* p_this)
{
    return p_this->status;
}

/**********************************************************************************************************************/
/* Private Functions Implementation                                                                                   */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description : Enable DMA peripheral clock
 * Input       : p_this - Pointer to interface resources
 * Return      : None
 **********************************************************************************************************************/
static void dma_clock_enable(dma_resources_t* p_this)
{
#if defined(DMAMUX1)
    /* Enable the DMAMUX at the first time we need a DMA */
    if ((osSemaphoreGetCount(p_this->block1_sem) == DMA1_MAX_SEMAPHORE_VALUE)
        && (osSemaphoreGetCount(p_this->block2_sem) == DMA2_MAX_SEMAPHORE_VALUE))
    {
        dmamux_clock_enable();
    }
#endif

    if (p_this->block_num == DMA_BLOCK_DMA1)
    {
        /* Is it the first channel in the block */
        int32_t this_block_active_channels = osSemaphoreGetCount(p_this->block1_sem);

        if (this_block_active_channels == DMA1_MAX_SEMAPHORE_VALUE)
        {
            /* if it is the first channel in the block activate block */
#if defined(__HAL_RCC_GPDMA1_CLK_ENABLE)
            __HAL_RCC_GPDMA1_CLK_ENABLE();
            __HAL_RCC_GPDMA1_CLK_SLEEP_ENABLE();
#else
            __HAL_RCC_DMA1_CLK_ENABLE();
#if defined(__HAL_RCC_DMA1_CLK_SLEEP_ENABLE)
            __HAL_RCC_DMA1_CLK_SLEEP_ENABLE();
#endif
#endif
        }

        if (osOK != osSemaphoreAcquire(p_this->block1_sem, osWaitForever))
        {
            proj_exception();
        }
    }
    else if (p_this->block_num == DMA_BLOCK_DMA2)
    {
        /* Is it the first channel in the block */
        int32_t this_block_active_channels = osSemaphoreGetCount(p_this->block2_sem);

        if (this_block_active_channels == DMA2_MAX_SEMAPHORE_VALUE)
        {
#if defined(__HAL_RCC_LPDMA1_CLK_ENABLE)
            __HAL_RCC_LPDMA1_CLK_ENABLE();
            __HAL_RCC_LPDMA1_CLK_SLEEP_ENABLE();
#else
            /* if it is the first channel in the block activate block */
            __HAL_RCC_DMA2_CLK_ENABLE();
#if defined(__HAL_RCC_DMA2_CLK_SLEEP_ENABLE)
            __HAL_RCC_DMA2_CLK_SLEEP_ENABLE();
#endif
#endif
        }

        if (osOK != osSemaphoreAcquire(p_this->block2_sem, osWaitForever))
        {
            proj_exception();
        }
    }
}

/***********************************************************************************************************************
 * Description : Disable DMA peripheral clock
 * Input       : p_this - pointer to interface resources
 * Return:     : None
 **********************************************************************************************************************/
static void dma_clock_disable(dma_resources_t* p_this)
{
    if (p_this->block_num == DMA_BLOCK_DMA1)
    {
        if (osOK != osSemaphoreRelease(p_this->block1_sem))
        {
            proj_exception();
        }

        /* Is it the last channel in the block */
        int32_t this_block_active_channels = osSemaphoreGetCount(p_this->block1_sem);

        if (this_block_active_channels == DMA1_MAX_SEMAPHORE_VALUE)
        {
            /* if it was the last channel in the block deactivate block */
#if defined(__HAL_RCC_GPDMA1_CLK_DISABLE)
            __HAL_RCC_GPDMA1_CLK_DISABLE();
            __HAL_RCC_GPDMA1_CLK_SLEEP_DISABLE();
#else
            __HAL_RCC_DMA1_CLK_DISABLE();
#if defined(__HAL_RCC_DMA1_CLK_SLEEP_DISABLE)
            __HAL_RCC_DMA1_CLK_SLEEP_DISABLE();
#endif
#endif
        }
    }
    else if (p_this->block_num == DMA_BLOCK_DMA2)
    {
        if (osOK != osSemaphoreRelease(p_this->block2_sem))
        {
            proj_exception();
        }

        /* Is it the last channel in the block */
        int32_t this_block_active_channels = osSemaphoreGetCount(p_this->block2_sem);

        if (this_block_active_channels == DMA2_MAX_SEMAPHORE_VALUE)
        {
            /* if it was the last channel in the block deactivate block */
#if defined(__HAL_RCC_LPDMA1_CLK_DISABLE)
            __HAL_RCC_LPDMA1_CLK_DISABLE();
            __HAL_RCC_LPDMA1_CLK_SLEEP_DISABLE();
#else
            __HAL_RCC_DMA2_CLK_DISABLE();
#if defined(__HAL_RCC_DMA2_CLK_SLEEP_DISABLE)
            __HAL_RCC_DMA2_CLK_SLEEP_DISABLE();
#endif
#endif
        }
    }

#if defined(DMAMUX1)
    /* Disable the DMAMUX only if we do not use any DMA channel */
    if ((osSemaphoreGetCount(p_this->block1_sem) == DMA1_MAX_SEMAPHORE_VALUE)
        && (osSemaphoreGetCount(p_this->block2_sem) == DMA2_MAX_SEMAPHORE_VALUE))
    {
        dmamux_clock_disable();
    }
#endif
}

#if defined(DMAMUX1)
/***********************************************************************************************************************
 * Description : Enable DMAMUX clocks.
 **********************************************************************************************************************/
static void dmamux_clock_enable(void)
{
    if (!__HAL_RCC_DMAMUX1_IS_CLK_ENABLED())
    {
        __HAL_RCC_DMAMUX1_CLK_ENABLE();
        __HAL_RCC_DMAMUX1_CLK_SLEEP_ENABLE();
    }
}

/***********************************************************************************************************************
 * Description : Disable DMAMUX clocks.
 **********************************************************************************************************************/
static void dmamux_clock_disable(void)
{
    if (__HAL_RCC_DMAMUX1_IS_CLK_ENABLED())
    {
        __HAL_RCC_DMAMUX1_CLK_SLEEP_DISABLE();
        __HAL_RCC_DMAMUX1_CLK_DISABLE();
    }
}
#endif