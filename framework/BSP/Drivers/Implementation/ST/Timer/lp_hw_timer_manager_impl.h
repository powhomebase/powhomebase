/***********************************************************************************************************************
 * Description: Header file of Low-Power HW Timer Manager for STM32L4 and STM32WL55.
 **********************************************************************************************************************/
#pragma once


/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* API */
#include <hw_timer_manager.h>

/* Libraries */
#include CMSIS_device_header
#include <assert.h>
#if defined(STM32L4)
#include <stm32l4xx_ll_lptim.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_lptim.h>
#endif

/* Some ST's have an LPTIM peripheral and some not. This define indicates if we have it or not */
#if defined(LPTIM1)

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#define LP_HW_TIMER_MANAGER_CLOCK_CONTROL_DECLARE(num_)                                                                \
                                                                                                                       \
    static_assert(num_ == 1 || num_ == 2, "LPTIM" #num_ " doesn't exist.");                                            \
                                                                                                                       \
    void lp_hw_timer_manager_clock_enable_##num_(void);                                                                \
    void lp_hw_timer_manager_clock_disable_##num_(void)

#define LP_HW_TIMER_MANAGER_CLOCK_CONTROL_DEFINE(num_)                                                                 \
                                                                                                                       \
    static_assert(num_ == 1 || num_ == 2, "LPTIM" #num_ " doesn't exist.");                                            \
                                                                                                                       \
    void lp_hw_timer_manager_clock_enable_##num_(void)                                                                 \
    {                                                                                                                  \
        __HAL_RCC_LPTIM##num_##_CLK_ENABLE();                                                                          \
        __HAL_RCC_LPTIM##num_##_CLK_SLEEP_ENABLE();                                                                    \
    }                                                                                                                  \
                                                                                                                       \
    void lp_hw_timer_manager_clock_disable_##num_(void)                                                                \
    {                                                                                                                  \
        __HAL_RCC_LPTIM##num_##_CLK_DISABLE();                                                                         \
        __HAL_RCC_LPTIM##num_##_CLK_SLEEP_DISABLE();                                                                   \
    }

#define LP_HW_TIMER_MANAGER_OBJECT_REFERENCE(num_) lp_hw_timer_manager_impl_##num_

/***********************************************************************************************************************
 * Description: Generate a LP HW Timer Manager object.
 * Inputs     : num_              - lp-timer num, valid options: 1, 2
 *              step_time_ns_   - requested time of each step, if no possible division exists to generate the
 *                                requested time from the timer's source clock, the manager will return an error
 *                                upon init.
 *              steps_per_period_ - how many steps should the counter count before restarting the count, should be
 *                                  within the underlying timer's physical range (16 or 32 bit).
 **********************************************************************************************************************/
#define LP_HW_TIMER_MANAGER_GENERATE_OBJECT(num_, step_time_ns_, steps_per_period_)                                    \
                                                                                                                       \
    static_assert(num_ == 1 || num_ == 2, "LPTIM" #num_ " doesn't exist.");                                            \
                                                                                                                       \
    static_assert(step_time_ns_ > 0, "Counter step time must be larger than 0.");                                      \
                                                                                                                       \
    static_assert(step_time_ns_ <= NS_IN_S, "Counter step time must be less than or equal to one second.");            \
                                                                                                                       \
    static_assert(steps_per_period_ > 0, "Timer must take at least 1 step per period");                                \
                                                                                                                       \
    static_assert(steps_per_period_ <= UINT16_MAX,                                                                     \
                  "Requested `steps_per_period` exced LPTIM" #num_ "s capabilities.");                                 \
                                                                                                                       \
    LP_HW_TIMER_MANAGER_CLOCK_CONTROL_DEFINE(num_);                                                                    \
                                                                                                                       \
    void LPTIM##num_##_IRQHandler(void)                                                                                \
    {                                                                                                                  \
        extern lp_hw_timer_manager_impl_t LP_HW_TIMER_MANAGER_OBJECT_REFERENCE(num_);                                  \
        lp_hw_timer_manager_irq(&LP_HW_TIMER_MANAGER_OBJECT_REFERENCE(num_));                                          \
    }                                                                                                                  \
                                                                                                                       \
    lp_hw_timer_manager_impl_t LP_HW_TIMER_MANAGER_OBJECT_REFERENCE(num_) = {                                          \
        ._base =                                                                                                       \
            {                                                                                                          \
                ._p_conf =                                                                                             \
                    &(hw_timer_manager_conf_t){                                                                        \
                        .m                = &lp_hw_timer_manager_impl_m,                                               \
                        .step_time_ns     = (step_time_ns_),                                                           \
                        .steps_per_period = (steps_per_period_),                                                       \
                    },                                                                                                 \
                ._state = HW_TIMER_MANAGER_STATE_UNINITIALIZED,                                                        \
            },                                                                                                         \
        ._p_conf =                                                                                                     \
            &(lp_hw_timer_manager_impl_conf_t){                                                                        \
                .p_instance    = LPTIM##num_,                                                                          \
                .clock_enable  = lp_hw_timer_manager_clock_enable_##num_,                                              \
                .clock_disable = lp_hw_timer_manager_clock_disable_##num_,                                             \
                .irq_n         = LPTIM##num_##_IRQn,                                                                   \
            },                                                                                                         \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*lp_hw_timer_manager_clock_control_t)(void);

typedef struct {
    lp_hw_timer_manager_clock_control_t clock_enable;
    lp_hw_timer_manager_clock_control_t clock_disable;
    LPTIM_TypeDef*                      p_instance;
    IRQn_Type                           irq_n;
} const lp_hw_timer_manager_impl_conf_t;

typedef struct {
    hw_timer_manager_t                     _base;
    lp_hw_timer_manager_impl_conf_t* const _p_conf;
} lp_hw_timer_manager_impl_t;

/* Defines bitmap shift of STM32L4 specific events */
typedef enum {
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_EXT_TRIG = HW_TIMER_MANAGER_EVENT_SHIFT_NUM, /* External Trigger */
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_UP,                                          /* Direction changed to UP */
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_DOWN,                                        /* Direction changed to DOWN */
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_UE,                                          /* Update Event */
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_COMPARE_OK,                                  /* Compare reg update ok */
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_ARR_OK,                                      /* ARR reg update ok */
    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_REP_OK,                                      /* Repetition reg update ok */

    LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_NUM,
} lp_hw_timer_manager_impl_event_shift_e;

static_assert(LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_NUM <= 32, "Can encode up to 32 events on 32 bits");

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Virtual Methods */
extern hw_timer_manager_methods_t const lp_hw_timer_manager_impl_m;

/**********************************************************************************************************************/
/* Protected Function Declarations                                                                                    */
/**********************************************************************************************************************/

/* ------------------------------------- IRQ Handlers ----------------------------------------------------------------*/

/* Low Power Timer 1 and 2 IRQ handler */
void lp_hw_timer_manager_irq(lp_hw_timer_manager_impl_t* p_this);

#endif