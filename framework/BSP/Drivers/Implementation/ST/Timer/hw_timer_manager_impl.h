/***********************************************************************************************************************
 * Description: Header file of HW Timer Manager for STM32 devices.
 *
 * Notes      : While most timers have a single IRQ handler, others (TIM1 and TIM8) have multiple.
 *              In some devices, some timers share their IRQ handlers with other timers/peripherals:
 *              - In STM32L4, timers (TIM15/16/17) share their IRQ handlers with TIM1.
 *              - In STM32F4, timers (TIM9/10/11) share their IRQ handlers with TIM1, and (TIM12/13/14) with TIM8.
 *              - IMPORTANT: In both STM32L4 & STM32F4, TIM6 and the DAC share an interrupt line!
 *                           Currently this module disables and enables the IRQ within the NVIC, making it incompatible
 *                           with the DAC (do not use both simultaneously).
 *
 *              To simplify the driver and increase its performance, we decided to leave implementing the IRQ to the
 *              end-user, simply implement the IRQ handler of each timer you use, you are advised to use the
 *              following templates (for STM32L4):
 *
 *              For all (currently known) devices:
 *
 *              * Timers 2, 3, 4, 5, 7
 *                void TIMn_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(n));
 *                }
 *
 *              For STM32L4 specifically:
 *
 *              * Timers 1, 15
 *                void TIM1_BRK_TIM15_IRQHandler(void)
 *                {
 *                    // Leave the appropriate ones, depending on which timers are in use.
 *                    hw_timer_manager_irq_brk(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(15));
 *                }
 *
 *              * Timers 1, 16
 *                void TIM1_UP_TIM16_IRQHandler(void)
 *                {
 *                    // Leave the appropriate ones, depending on which timers are in use.
 *                    hw_timer_manager_irq_up(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(16));
 *                }
 *
 *              * Timers 1, 17
 *                void TIM1_TRG_COM_TIM17_IRQHandler(void)
 *                {
 *                    // Leave the appropriate ones, depending on which timers are in use.
 *                    hw_timer_manager_irq_trg_com(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(17));
 *                }
 *
 *              * Timer 8
 *                void TIM8_BRK_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_brk(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                }
 *
 *                void TIM8_UP_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_up(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                }
 *
 *                void TIM8_TRG_COM_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_trg_com(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                }
 *
 *              For STM32F4 specifically:
 *
 *              * Timers 1, 9
 *                void TIM1_BRK_TIM9_IRQHandler(void)
 *                {
 *                    // Leave the appropriate ones, depending on which timers are in use.
 *                    hw_timer_manager_irq_brk(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(9));
 *                }
 *
 *              * Timers 1, 10
 *                void TIM1_UP_TIM10_IRQHandler(void)
 *                {
 *                    // Leave the appropriate ones, depending on which timers are in use.
 *                    hw_timer_manager_irq_up(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(10));
 *                }
 *
 *              * Timers 1, 11
 *                void TIM1_TRG_COM_TIM11_IRQHandler(void)
 *                {
 *                    // Leave the appropriate ones, depending on which timers are in use.
 *                    hw_timer_manager_irq_trg_com(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(11));
 *                }
 *
 *              * Timers 8, 12
 *                void TIM8_BRK_TIM12_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_brk(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(12));
 *                }
 *
 *              * Timers 8, 13
 *                void TIM8_UP_TIM13_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_up(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(13));
 *                }
 *
 *              * Timers 8, 14
 *                void TIM8_TRG_COM_TIM14_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_trg_com(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(14));
 *                }
 *
 *              For both STM32L4 & STM32F4:
 *
 *              * Timer 1
 *                void TIM1_CC_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_cc(&HW_TIMER_MANAGER_REFERENCE_OBJECT(1));
 *                }
 *
 *              * Timer 8
 *                void TIM8_CC_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq_cc(&HW_TIMER_MANAGER_REFERENCE_OBJECT(8));
 *                }
 *
 *              * Timer 6
 *                void TIM6_DAC_IRQHandler(void)
 *                {
 *                    hw_timer_manager_irq(&HW_TIMER_MANAGER_REFERENCE_OBJECT(6));
 *                    // Handle DAC underrun irq below/above
 *                }
 *
 * Simply copy the relevant template, to your objs file and uncomment it.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* API */
#include <hw_timer_manager.h>

/* Project Related */
#include <assert.h>
#include <macros.h>

/* Libraries */
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#if !defined(STM32F1)
#define _HAL_RCC_TIM_CLK_SLEEP_ENABLE(num_)  __HAL_RCC_TIM##num_##_CLK_SLEEP_ENABLE()
#define _HAL_RCC_TIM_CLK_SLEEP_DISABLE(num_) __HAL_RCC_TIM##num_##_CLK_SLEEP_DISABLE()
#else
/* No support in STM32F1 */
#define _HAL_RCC_TIM_CLK_SLEEP_ENABLE(num_)  (void)num_
#define _HAL_RCC_TIM_CLK_SLEEP_DISABLE(num_) (void)num_
#endif /* STM32F1 */

#if defined(CONFIG_DEBUG)
#define _HAL_DBGMCU_FREEZE_TIMER(num_)   __HAL_DBGMCU_FREEZE_TIM##num_()
#define _HAL_DBGMCU_UNFREEZE_TIMER(num_) __HAL_DBGMCU_UNFREEZE_TIM##num_()
#else
#define _HAL_DBGMCU_FREEZE_TIMER(num_)   {}
#define _HAL_DBGMCU_UNFREEZE_TIMER(num_) {}
#endif

#define HW_TIMER_MANAGER_CLOCK_CONTROL_DEFINE(num_)                                                                    \
    void hw_timer_manager_clock_enable_##num_(void)                                                                    \
    {                                                                                                                  \
        __HAL_RCC_TIM##num_##_CLK_ENABLE();                                                                            \
        _HAL_RCC_TIM_CLK_SLEEP_ENABLE(num_);                                                                           \
        _HAL_DBGMCU_FREEZE_TIMER(num_);                                                                                \
    }                                                                                                                  \
                                                                                                                       \
    void hw_timer_manager_clock_disable_##num_(void)                                                                   \
    {                                                                                                                  \
        __HAL_RCC_TIM##num_##_CLK_DISABLE();                                                                           \
        _HAL_RCC_TIM_CLK_SLEEP_DISABLE(num_);                                                                          \
        _HAL_DBGMCU_UNFREEZE_TIMER(num_);                                                                              \
    }

#define HW_TIMER_MANAGER_NVIC_CONTROL_DECLARE(num_)                                                                    \
    void hw_timer_manager_nvic_enable_##num_(void);                                                                    \
    void hw_timer_manager_nvic_disable_##num_(void)

#define HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(num_)                                                                     \
    void hw_timer_manager_nvic_enable_##num_(void)                                                                     \
    {                                                                                                                  \
        NVIC_ClearPendingIRQ(TIM##num_##_IRQn);                                                                        \
        NVIC_EnableIRQ(TIM##num_##_IRQn);                                                                              \
    }                                                                                                                  \
                                                                                                                       \
    void hw_timer_manager_nvic_disable_##num_(void)                                                                    \
    {                                                                                                                  \
        NVIC_DisableIRQ(TIM##num_##_IRQn);                                                                             \
    }

#define HW_TIMER_MANAGER_REFERENCE_OBJECT(num_) hw_timer_manager_impl_##num_

/***********************************************************************************************************************
 * Description: Generate a HW Timer Manager object.
 * Inputs     : num_               - timer num, valid options:
 *                                       STM32L4 - 1 to 8, 15 to 17.
 *                                       STM32F1 - 1 to 8.
 *                                       STM32F4 - 1 to 14.
 *              step_time_ns_      - requested time of each step, if no possible division exists to generate the
 *                                   requested time from the timer's source clock, the manager will return an error
 *                                   upon init.
 *              steps_per_period_  - how many steps should the counter count before restarting the count, should be
 *                                   within the underlying timer's physical range (16 or 32 bit).
 **********************************************************************************************************************/
#define HW_TIMER_MANAGER_GENERATE_OBJECT(num_, step_time_ns_, steps_per_period_)                                       \
                                                                                                                       \
    static_assert(IS_TIM_INSTANCE(TIM##num_), "TIM" #num_ " does not exist.");                                         \
                                                                                                                       \
    static_assert(steps_per_period_ <= (IS_TIM_32B_COUNTER_INSTANCE(TIM##num_) ? UINT32_MAX : UINT16_MAX),             \
                  "Requested steps_per_period exceeds TIM" #num_ "s capabilities.");                                   \
                                                                                                                       \
    static_assert(step_time_ns_ > 0, "Counter step time must be larger than 0.");                                      \
                                                                                                                       \
    static_assert(step_time_ns_ <= NS_IN_S, "Counter step time must be less than or equal to one second.");            \
                                                                                                                       \
    static_assert(steps_per_period_ > 0, "Timer must take at least 1 step per period");                                \
                                                                                                                       \
    HW_TIMER_MANAGER_CLOCK_CONTROL_DEFINE(num_);                                                                       \
    HW_TIMER_MANAGER_NVIC_CONTROL_DECLARE(num_);                                                                       \
                                                                                                                       \
    hw_timer_manager_impl_t HW_TIMER_MANAGER_REFERENCE_OBJECT(num_) = {                                                \
        ._base =                                                                                                       \
            {                                                                                                          \
                ._p_conf =                                                                                             \
                    &(hw_timer_manager_conf_t){                                                                        \
                        .m                      = &hw_timer_manager_impl_m,                                            \
                        .step_time_ns           = (step_time_ns_),                                                     \
                        .steps_per_period       = (steps_per_period_),                                                 \
                        .max_allowed_freq_shift = 0,                                                                   \
                    },                                                                                                 \
                ._state = HW_TIMER_MANAGER_STATE_UNINITIALIZED,                                                        \
            },                                                                                                         \
        ._p_conf =                                                                                                     \
            &(hw_timer_manager_impl_conf_t){                                                                           \
                .p_instance    = TIM##num_,                                                                            \
                .clock_enable  = hw_timer_manager_clock_enable_##num_,                                                 \
                .clock_disable = hw_timer_manager_clock_disable_##num_,                                                \
                .nvic_enable   = hw_timer_manager_nvic_enable_##num_,                                                  \
                .nvic_disable  = hw_timer_manager_nvic_disable_##num_,                                                 \
            },                                                                                                         \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Clock enable function */
typedef void (*hw_timer_manager_clock_control_t)(void);
typedef void (*hw_timer_manager_nvic_control_t)(void);

typedef struct {
    hw_timer_manager_clock_control_t clock_enable;
    hw_timer_manager_clock_control_t clock_disable;
    hw_timer_manager_clock_control_t nvic_enable;
    hw_timer_manager_clock_control_t nvic_disable;
    TIM_TypeDef *                    p_instance;
} const hw_timer_manager_impl_conf_t;

typedef struct {
    hw_timer_manager_t                  _base;
    hw_timer_manager_impl_conf_t *const _p_conf;
} hw_timer_manager_impl_t;

/* Defines bitmap shift of STM32 specific events */
typedef enum {
    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_SYSTEM_BREAK = HW_TIMER_MANAGER_EVENT_SHIFT_NUM, /* System Break */
    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_BREAK,                                           /* Break */
    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_BREAK_2,                                         /* Break 2 */
    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_TRIGGER,                                         /* Trigger */
    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_COM,                                             /* COM */
    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_UPDATE,                                          /* UPDATE */

    HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_NUM,
} hw_timer_manager_impl_event_shift_e;

static_assert(HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_NUM <= 32, "Can encode up to 32 events on 32 bits");

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Virtual Methods */
extern hw_timer_manager_methods_t const hw_timer_manager_impl_m;

/**********************************************************************************************************************/
/* Protected Function Declarations                                                                                    */
/**********************************************************************************************************************/

/* ------------------------------------- IRQ Handlers ----------------------------------------------------------------*/

/* General TIMn IRQ handler */
void hw_timer_manager_irq(hw_timer_manager_impl_t *p_this);

/* Timer 1/8 IRQ handler - Break IRQs */
void hw_timer_manager_irq_brk(hw_timer_manager_impl_t *p_this);

/* Timer 1/8 IRQ handler - Up IRQs */
void hw_timer_manager_irq_up(hw_timer_manager_impl_t *p_this);

/* Timer 1/8 IRQ handler - Trigger and COM IRQs */
void hw_timer_manager_irq_trg_com(hw_timer_manager_impl_t *p_this);

/* Timer 1/8 IRQ handler - Capture Compare IRQs */
void hw_timer_manager_irq_cc(hw_timer_manager_impl_t *p_this);
