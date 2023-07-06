/**********************************************************************************************************************/
/* Description: Wrapper for creation and usage of RTOS object attributes                                              */
/* Usage      : When creating any rtx object, you can allocate the necessary memory during compile time, instead of   */
/*              allocating from the dynamic memory at runtime, to do that you must allocate static memory and pass    */
/*              the appropriate pointers through the attributes argument.                                             */
/*              This header provides simple to use macros that do all that work for you.                              */
/*                                                                                                                    */
/*              The macros MUST be used ONLY when creating objects of STATIC storage duration, otherwise it WILL      */
/*              cause undefined behavior!                                                                             */
/*              Please note, that each macro call creates a new independent array in memory, and should be used only  */
/*              for the creation of a SINGLE object.                                                                  */
/*                                                                                                                    */
/*              Example usage:                                                                                        */
/*              l3_objs.c:                                                                                            */
/*              ...                                                                                                   */
/*              static decoder_conf_t const decoder_conf = {                                                          */
/*                  .p_uncoded_pool_params                                                                            */
/*                      = OS_PARAMS_STATIC_MEMORY_POOL_CREATE_PTR(cnc_decoded_buff, 2, 100),                          */
/*                  .p_uncoded_session_info_pool_params =                                                             */
/*                     OS_PARAMS_STATIC_MEMORY_POOL_CREATE_PTR(cnc_decoded_buff_session,                              */
/*                                                             2,                                                     */
/*                                                             sizeof(l2_proto_session_info_t)),                      */
/*                  ...                                                                                               */
/*              };                                                                                                    */
/*                                                                                                                    */
/*              cnc_decoder_t cnc_rf_decoder = {                                                                      */
/*                  .base =                                                                                           */
/*                      {                                                                                             */
/*                          ...                                                                                       */
/*                          .p_conf = &decoder_conf,                                                                  */
/*                          ...                                                                                       */
/*                      },                                                                                            */
/*              };                                                                                                    */
/*              ...                                                                                                   */
/*                                                                                                                    */
/*              decoder_das.c:                                                                                        */
/*              ...                                                                                                   */
/*              osMemoryPoolParameters_t *p_pool_params = p_this->p_conf->p_uncoded_pool_params;                      */
/*                                                                                                                    */
/*              if (p_pool_params != NULL)                                                                            */
/*              {                                                                                                     */
/*                  p_this->uncoded_buffers_pool =                                                                    */
/*                      osMemoryPoolNew(p_pool_params->block_count, p_pool_params->block_size, p_pool_params->attr);  */
/*              ...                                                                                                   */
/*              }                                                                                                     */
/*                                                                                                                    */
/*              Notice that `decoder_conf` has a static lifetime!                                                     */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Library headers */
#include <cmsis_os2.h>
#include <proj_assert.h>
#include <rtx_os.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* ------------------------------------- OS function wrappers ------------------------------------------------------- */

/***********************************************************************************************************************
 * Description: Safely allocate `size_` bytes from a memory pool.
 * Inputs     : id_      - Memory pool's ID.
 *              size_    - Size to allocate.
 *              timeout_ - Timeout for allocation.
 * Note       : This macro just asserts that the pool's blocks are large enough for the requested allocation and that
 *              the allocation succeeds.
 *              The underlying allocation will still be the pool's block size.
 **********************************************************************************************************************/
#define osMemoryPoolAllocChecked(id_, size_, timeout_)                                                                 \
    ({                                                                                                                 \
        void* res;                                                                                                     \
        proj_assert(osMemoryPoolGetBlockSize(id_) >= (size_));                                                         \
        res = osMemoryPoolAlloc(id_, timeout_);                                                                        \
        proj_assert(res != NULL);                                                                                      \
        res;                                                                                                           \
    })

/* Same as `osMemoryPoolAllocChecked` but without assert when allocation fails to let the user decide how to handle  */
#define osMemoryPoolAllocCheckedNoAssert(id_, size_, timeout_)                                                         \
    ({                                                                                                                 \
        void* res;                                                                                                     \
        proj_assert(osMemoryPoolGetBlockSize(id_) >= (size_));                                                         \
        res = osMemoryPoolAlloc(id_, timeout_);                                                                        \
        res;                                                                                                           \
    })

#if defined(CONFIG_OS_DEBUG_NAMES)
#define NAME(name_) #name_
#else
#define NAME(name_) NULL
#endif

/* ------------------------------------- Generate Struct ------------------------------------------------------------ */

#define OS_ATTR_STATIC_THREAD_CREATE(_name, _priority, _stack_size)     /**/                                           \
    {                                                                   /**/                                           \
        .name           = NAME(Thread _name),                           /**/                                           \
            .attr_bits  = osThreadDetached,                             /**/                                           \
            .cb_mem     = (__ALIGNED(4) uint8_t[osRtxThreadCbSize]){0}, /**/                                           \
            .cb_size    = osRtxThreadCbSize,                            /**/                                           \
            .stack_mem  = (__ALIGNED(8) uint8_t[_stack_size]){0},       /**/                                           \
            .stack_size = _stack_size,                                  /**/                                           \
            .priority   = _priority,                                    /**/                                           \
    }                                                                   /**/

#define OS_ATTR_STATIC_TIMER_CREATE(_name)                            /**/                                             \
    {                                                                 /**/                                             \
        .name          = NAME(Timer _name),                           /**/                                             \
            .attr_bits = 0,                                           /**/                                             \
            .cb_mem    = (__ALIGNED(4) uint8_t[osRtxTimerCbSize]){0}, /**/                                             \
            .cb_size   = osRtxTimerCbSize,                            /**/                                             \
    }                                                                 /**/

#define OS_ATTR_STATIC_EVENT_FLAGS_CREATE(_name)                           /**/                                        \
    {                                                                      /**/                                        \
        .name          = NAME(Event Flags _name),                          /**/                                        \
            .attr_bits = 0,                                                /**/                                        \
            .cb_mem    = (__ALIGNED(4) uint8_t[osRtxEventFlagsCbSize]){0}, /**/                                        \
            .cb_size   = osRtxEventFlagsCbSize,                            /**/                                        \
    }                                                                      /**/

#define OS_ATTR_STATIC_MUTEX_CREATE(_name, _attr_bits)                /**/                                             \
    {                                                                 /**/                                             \
        .name          = NAME(Mutex _name),                           /**/                                             \
            .attr_bits = _attr_bits,                                  /**/                                             \
            .cb_mem    = (__ALIGNED(4) uint8_t[osRtxMutexCbSize]){0}, /**/                                             \
            .cb_size   = osRtxMutexCbSize,                            /**/                                             \
    }                                                                 /**/

#define OS_ATTR_STATIC_SEMAPHORE_CREATE(_name)                            /**/                                         \
    {                                                                     /**/                                         \
        .name          = NAME(Semaphore _name),                           /**/                                         \
            .attr_bits = 0,                                               /**/                                         \
            .cb_mem    = (__ALIGNED(4) uint8_t[osRtxSemaphoreCbSize]){0}, /**/                                         \
            .cb_size   = osRtxSemaphoreCbSize,                            /**/                                         \
    }                                                                     /**/

#define OS_ATTR_STATIC_MEMORY_POOL_CREATE(_name, _block_count, _block_size)                            /**/            \
    {                                                                                                  /**/            \
        .name          = NAME(Memory Pool _name),                                                      /**/            \
            .attr_bits = 0,                                                                            /**/            \
            .cb_mem    = (__ALIGNED(4) uint8_t[osRtxMemoryPoolCbSize]){0},                             /**/            \
            .cb_size   = osRtxMemoryPoolCbSize,                                                        /**/            \
            .mp_mem    = (__ALIGNED(4) uint8_t[osRtxMemoryPoolMemSize(_block_count, _block_size)]){0}, /**/            \
            .mp_size   = (osRtxMemoryPoolMemSize(_block_count, _block_size)),                          /**/            \
    }                                                                                                  /**/

#define OS_ATTR_STATIC_MESSAGE_QUEUE_CREATE(_name, _msg_count, _msg_size)                            /**/              \
    {                                                                                                /**/              \
        .name          = NAME(Message Queue _name),                                                  /**/              \
            .attr_bits = 0,                                                                          /**/              \
            .cb_mem    = (__ALIGNED(4) uint8_t[osRtxMessageQueueCbSize]){0},                         /**/              \
            .cb_size   = osRtxMessageQueueCbSize,                                                    /**/              \
            .mq_mem    = (__ALIGNED(4) uint8_t[osRtxMessageQueueMemSize(_msg_count, _msg_size)]){0}, /**/              \
            .mq_size   = (osRtxMessageQueueMemSize(_msg_count, _msg_size)),                          /**/              \
    }                                                                                                /**/

#define OS_PARAMS_STATIC_MEMORY_POOL_CREATE(_name, _block_count, _block_size)                   /**/                   \
    {                                                                                           /**/                   \
        .attr            = OS_ATTR_STATIC_MEMORY_POOL_CREATE(_name, _block_count, _block_size), /**/                   \
            .block_count = _block_count,                                                        /**/                   \
            .block_size  = _block_size,                                                         /**/                   \
    }                                                                                           /**/

#define OS_PARAMS_STATIC_MESSAGE_QUEUE_CREATE(_name, _msg_count, _msg_size)                 /**/                       \
    {                                                                                       /**/                       \
        .attr          = OS_ATTR_STATIC_MESSAGE_QUEUE_CREATE(_name, _msg_count, _msg_size), /**/                       \
            .msg_count = _msg_count,                                                        /**/                       \
            .msg_size  = _msg_size,                                                         /**/                       \
    }

/* ------------------------------------- Generate Struct and Get Pointer -------------------------------------------- */

#define OS_ATTR_STATIC_THREAD_CREATE_PTR(_name, _priority, _stack_size)                                                \
    &(osThreadAttr_t const)OS_ATTR_STATIC_THREAD_CREATE(_name, _priority, _stack_size)

#define OS_ATTR_STATIC_TIMER_CREATE_PTR(_name) &(osTimerAttr_t const)OS_ATTR_STATIC_TIMER_CREATE(_name)

#define OS_ATTR_STATIC_EVENT_FLAGS_CREATE_PTR(_name) &(osEventFlagsAttr_t const)OS_ATTR_STATIC_EVENT_FLAGS_CREATE(_name)

#define OS_ATTR_STATIC_MUTEX_CREATE_PTR(_name, _attr_bits)                                                             \
    &(osMutexAttr_t const)OS_ATTR_STATIC_MUTEX_CREATE(_name, _attr_bits)

#define OS_ATTR_STATIC_SEMAPHORE_CREATE_PTR(_name) &(osSemaphoreAttr_t const)OS_ATTR_STATIC_SEMAPHORE_CREATE(_name)

#define OS_ATTR_STATIC_MEMORY_POOL_CREATE_PTR(_name, _block_count, _block_size)                                        \
    &(osMemoryPoolAttr_t const)OS_ATTR_STATIC_MEMORY_POOL_CREATE(_name, _block_count, _block_size)

#define OS_ATTR_STATIC_MESSAGE_QUEUE_CREATE_PTR(_name, _msg_count, _msg_size)                                          \
    &(osMessageQueueAttr_t const)OS_ATTR_STATIC_MESSAGE_QUEUE_CREATE(_name, _msg_count, _msg_size)

#define OS_PARAMS_STATIC_MEMORY_POOL_CREATE_PTR(_name, _block_count, _block_size)                                      \
    &(osMemoryPoolParameters_t const)OS_PARAMS_STATIC_MEMORY_POOL_CREATE(_name, _block_count, _block_size)

#define OS_PARAMS_STATIC_MESSAGE_QUEUE_CREATE_PTR(_name, _msg_count, _msg_size)                                        \
    &(osMessageQueueParameters_t const)OS_PARAMS_STATIC_MESSAGE_QUEUE_CREATE(_name, _msg_count, _msg_size)

/* ------------------------------ Deprecated - should be replaced with the macros above ----------------------------- */

#if defined(CONFIG_OS_DEBUG_NAMES)
#define CMSIS_OS_DEFAULT_POOL_ATTR_DECLARE(_name)                                                                      \
    const osMemoryPoolAttr_t os_pool_attr_##_name = {                                                                  \
        .name = #_name,                                                                                                \
    }
#define CMSIS_OS_DEFAULT_POOL_ATTR_REFERENCE(_name) (&os_pool_attr_##_name)
#define CMSIS_OS_DEFAULT_SEMAPHORE_ATTR_DECLARE(_name)                                                                 \
    const osSemaphoreAttr_t os_sem_attr_##_name = {                                                                    \
        .name = #_name,                                                                                                \
    }
#define CMSIS_OS_DEFAULT_SEMAPHORE_ATTR_REFERENCE(_name) (&os_sem_attr_##_name)
#else
#define CMSIS_OS_DEFAULT_POOL_ATTR_DECLARE(_name)        /**/
#define CMSIS_OS_DEFAULT_POOL_ATTR_REFERENCE(_name)      NULL
#define CMSIS_OS_DEFAULT_SEMAPHORE_ATTR_DECLARE(_name)   /**/
#define CMSIS_OS_DEFAULT_SEMAPHORE_ATTR_REFERENCE(_name) NULL
#endif

#define QUEUE_PRIORITY_NORMAL 0
#define QUEUE_PRIORITY_HIGH   1

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef struct {
    osMemoryPoolAttr_t attr;
    uint32_t           block_count;
    uint32_t           block_size;
} const osMemoryPoolParameters_t;

typedef struct {
    osMessageQueueAttr_t attr;
    uint32_t             msg_count;
    uint32_t             msg_size;
} const osMessageQueueParameters_t;