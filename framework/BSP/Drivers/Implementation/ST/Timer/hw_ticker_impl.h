/***********************************************************************************************************************
 * Description: Header file of HW Ticker for STM32 devices.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* API */
#include <hw_ticker.h>

/* Project Related */
#include <macros.h>

/* Libraries */
#include <assert.h>
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Generate a HW Ticker object.
 * Inputs     : name_    - object name
 *              num_     - timer num
 *              channel_ - time channel to use for tick generation
 *********************************************************************************************************************/
#define HW_TICKER_GENERATE_OBJECT(name_, num_, channel_)                                                               \
                                                                                                                       \
    static_assert(IS_TIM_INSTANCE(TIM##num_), "TIM" #num_ " does not exist.");                                         \
                                                                                                                       \
    static_assert(IS_TIM_CCX_INSTANCE(TIM##num_, TIM_CHANNEL_##channel_),                                              \
                  "TIM" #num_ " doesn't have channel " #channel_);                                                     \
                                                                                                                       \
    hw_ticker_impl_t name_ = {                                                                                         \
        .base       = HW_TICKER_GENERATE_COMPOUND_LITERAL(hw_ticker_impl_m, channel_),                                 \
        .p_instance = TIM##num_,                                                                                       \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef struct {
    hw_ticker_t        base;
    TIM_TypeDef *const p_instance;
} hw_ticker_impl_t;

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Virtual Methods */
extern hw_ticker_methods_t const hw_ticker_impl_m;