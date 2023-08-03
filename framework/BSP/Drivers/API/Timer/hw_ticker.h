/***********************************************************************************************************************
 * Description: HW Ticker API.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project Related */
#include <Driver_Common.h>
#include <proj_assert.h>

/* Libraries */
#include <cmsis_compiler.h>
#include <stdbool.h>
#include <stdint.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Generate a HW Ticker base class compound literal.
 * Inputs     : m_       - virtual methods
 *              channel_ - channel of hardware timer
 **********************************************************************************************************************/
#define HW_TICKER_GENERATE_COMPOUND_LITERAL(m_, channel_)                                                              \
    (hw_ticker_t)                                                                                                      \
    {                                                                                                                  \
        .p_conf =                                                                                                      \
            &(hw_ticker_conf_t){                                                                                       \
                .m       = &m_,                                                                                        \
                .channel = channel_,                                                                                   \
            },                                                                                                         \
        .is_running = false,                                                                                           \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef struct _hw_ticker hw_ticker_t;

typedef struct {
    int32_t (*Start)(hw_ticker_t* p_this, uint32_t value);
    int32_t (*Stop)(hw_ticker_t* p_this);
    int32_t (*Set)(hw_ticker_t* p_this, uint32_t value);
} const hw_ticker_methods_t;

typedef struct {
    hw_ticker_methods_t* m;
    uint32_t             channel;
} const hw_ticker_conf_t;

typedef struct _hw_ticker {
    hw_ticker_conf_t* const p_conf;
    bool                    is_running;
} hw_ticker_t;

/**********************************************************************************************************************/
/* Interface Function Declarations                                                                                    */
/**********************************************************************************************************************/

/* Start Generating Ticks, first tick comes after `value` steps */
__INLINE int32_t hw_ticker_start(hw_ticker_t* p_this, uint32_t value)
{
    if (p_this->is_running)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR_BUSY;
    }

    int32_t res = p_this->p_conf->m->Start(p_this, value);
    if (res == ARM_DRIVER_OK)
    {
        p_this->is_running = true;
    }

    return res;
}

/* Stop generating ticks */
__INLINE int32_t hw_ticker_stop(hw_ticker_t* p_this)
{
    if (!p_this->is_running)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    int32_t res = p_this->p_conf->m->Stop(p_this);
    if (res == ARM_DRIVER_OK)
    {
        p_this->is_running = false;
    }

    return res;
}

/* Set capture compare to value */
__INLINE int32_t hw_ticker_set(hw_ticker_t* p_this, uint32_t value)
{
    if (!p_this->is_running)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    return p_this->p_conf->m->Set(p_this, value);
}