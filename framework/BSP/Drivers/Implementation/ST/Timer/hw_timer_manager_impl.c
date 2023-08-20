/***********************************************************************************************************************
 * Description: Implementation file of HW Timer Manager for STM32 devices.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own Header */
#include "hw_timer_manager_impl.h"

/* Libraries */
#if defined(STM32F1)
#include <stm32f1xx_ll_tim.h>
#elif defined(STM32F4)
#include <stm32f4xx_ll_tim.h>
#elif defined(STM32L4)
#include <stm32l4xx_ll_tim.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_tim.h>
#elif defined(STM32U5)
#include <stm32u5xx_ll_tim.h>
#endif

/* Project Related */
#if !defined(STM32U5)
#include <atomic.h>
#endif
#include <global.h>
#include <proj_exception.h>

/**********************************************************************************************************************/
/* Interface Virtual Method Declarations                                                                              */
/**********************************************************************************************************************/

static int32_t  initialize(hw_timer_manager_t* p_this, hw_timer_event_cb_t callback, void* arg);
static int32_t  uninitialize(hw_timer_manager_t* p_this);
static int32_t  power_control(hw_timer_manager_t* p_this, ARM_POWER_STATE state);
static uint32_t get_steps_current(hw_timer_manager_t* p_this);
static bool     is_overflow_pending(hw_timer_manager_t* p_this);

/**********************************************************************************************************************/
/* Private Function Declarations                                                                                      */
/**********************************************************************************************************************/

/* Get Timer's source clock frequency */
__STATIC_INLINE uint32_t get_src_clk_freq(hw_timer_manager_impl_t* p_this);

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Virtual Methods */
hw_timer_manager_methods_t const hw_timer_manager_impl_m = {
    .Initialize        = initialize,
    .Uninitialize      = uninitialize,
    .PowerControl      = power_control,
    .GetStepsCurrent   = get_steps_current,
    .IsOverflowPending = is_overflow_pending,
};

#if defined(STM32L4)
/* TIM1 shares interrupts with TIM15, TIM16, TIM17 */
#if defined(TIM1) && defined(TIM15)
static atomic_uint32_t flag_tim1_tim15 = 0;
#endif

#if defined(TIM1) && defined(TIM16)
static atomic_uint32_t flag_tim1_tim16 = 0;
#endif

#if defined(TIM1) && defined(TIM17)
static atomic_uint32_t flag_tim1_tim17 = 0;
#endif
#endif /* STM32L4 */

#if defined(STM32F4)
/* TIM1 shares interrupts with TIM9, TIM10, TIM11 */
#if defined(TIM1) && defined(TIM9)
static atomic_uint32_t flag_tim1_tim9 = 0;
#endif

#if defined(TIM1) && defined(TIM10)
static atomic_uint32_t flag_tim1_tim10 = 0;
#endif

#if defined(TIM1) && defined(TIM11)
static atomic_uint32_t flag_tim1_tim11 = 0;
#endif

/* TIM8 shares interrupts with TIM12, TIM13, TIM14 */
#if defined(TIM8) && defined(TIM12)
static atomic_uint32_t flag_tim8_tim12 = 0;
#endif

#if defined(TIM8) && defined(TIM13)
static atomic_uint32_t flag_tim8_tim13 = 0;
#endif

#if defined(TIM8) && defined(TIM14)
static atomic_uint32_t flag_tim8_tim14 = 0;
#endif
#endif /* STM32F4 */

/**********************************************************************************************************************/
/* Interface Virtual Method Definitions                                                                               */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Initializes the HW Timer Manager
 * Input      : p_interface - pointer to self
 *              callback    - event callback
 *              arg         - event callback argument
 * Return     : ARM_DRIVER return code
 * Notes      : performs some checks on the configuration, then initializes the HW Timer.
 *              additionaly, many asserts are done in the parent class/at compile time, repeating them here is
 *              redundant.
 **********************************************************************************************************************/
static int32_t initialize(hw_timer_manager_t* p_interface, hw_timer_event_cb_t callback, void* arg)
{
    hw_timer_manager_impl_t* p_this = (hw_timer_manager_impl_t*)p_interface;

    p_this->_base._src_clk_freq = get_src_clk_freq(p_this);
    if (p_this->_base._src_clk_freq == 0)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    bool     is_step_time_possible = false;
    uint32_t div                   = hw_timer_manager_calc_prescaler_factor(&(p_this->_base), &is_step_time_possible);
    if (!is_step_time_possible || div > UINT16_MAX)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    LL_TIM_InitTypeDef init_conf = {
        /* Set the default configuration */
        .Prescaler   = div - 1,
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        /* Actual period is [0, AutoReload], giving us a period of `AutoReload + 1` steps, to correct that we set
         * `AutoReload` to `steps_per_period - 1` */
        .Autoreload        = p_this->_base._p_conf->steps_per_period - 1,
        .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = (uint8_t)0x00,
    };

    /* Enable timer clock only for configuration sequence */
    p_this->_p_conf->clock_enable();

    ErrorStatus res = LL_TIM_Init(p_this->_p_conf->p_instance, &init_conf);
    LL_TIM_DisableCounter(p_this->_p_conf->p_instance);

    if (res != SUCCESS)
    {
        p_this->_p_conf->clock_disable();
        return ARM_DRIVER_ERROR;
    }

    /* Clear all pending interrupts and enable update interrupt */
    CLEAR_REG(p_this->_p_conf->p_instance->SR);
    LL_TIM_EnableIT_UPDATE(p_this->_p_conf->p_instance);

    p_this->_p_conf->clock_disable();

    if (res != SUCCESS)
    {
        return ARM_DRIVER_ERROR;
    }

    p_this->_p_conf->nvic_enable();

    p_interface->__event_callback     = callback;
    p_interface->__event_callback_arg = arg;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Uninitializes the HW Timer Manager
 * Input      : p_interface - pointer to self
 * Return     : ARM_DRIVER return code
 * Notes      : Disables the timer and restores HW Timer to default state
 **********************************************************************************************************************/
static int32_t uninitialize(hw_timer_manager_t* p_interface)
{
    hw_timer_manager_impl_t* p_this = (hw_timer_manager_impl_t*)p_interface;

    p_this->_p_conf->nvic_disable();

    p_this->_p_conf->clock_enable();
    ErrorStatus res = LL_TIM_DeInit(p_this->_p_conf->p_instance);
    p_this->_p_conf->clock_disable();

    p_interface->__event_callback     = NULL;
    p_interface->__event_callback_arg = NULL;

    if (res != SUCCESS)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Controls power state of the HW Timer
 * Input      : p_interface - pointer to self
 *              state       - desired power state
 * Return     : ARM_DRIVER return code
 **********************************************************************************************************************/
static int32_t power_control(hw_timer_manager_t* p_interface, ARM_POWER_STATE state)
{
    hw_timer_manager_impl_t* p_this = (hw_timer_manager_impl_t*)p_interface;

    switch (state)
    {
        case ARM_POWER_FULL:
        {
            p_this->_p_conf->clock_enable();
            LL_TIM_EnableCounter(p_this->_p_conf->p_instance);

            return ARM_DRIVER_OK;
        }
        case ARM_POWER_OFF:
        {
            LL_TIM_DisableCounter(p_this->_p_conf->p_instance);
            p_this->_p_conf->p_instance->SR = 0;

            p_this->_p_conf->clock_disable();

            return ARM_DRIVER_OK;
        }
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        default:
        {
            proj_assert(false);
            return ARM_DRIVER_ERROR;
        }
    }
}

/***********************************************************************************************************************
 * Description: Returns the current value of the HW Timer's counter
 * Input      : p_interface - pointer to self
 * Return     : current value of counter
 **********************************************************************************************************************/
static uint32_t get_steps_current(hw_timer_manager_t* p_interface)
{
    hw_timer_manager_impl_t* p_this = (hw_timer_manager_impl_t*)p_interface;

    return LL_TIM_GetCounter(p_this->_p_conf->p_instance);
}

/***********************************************************************************************************************
 * Description: Returns whether there's a pending overflow IRQ.
 * Input      : p_interface - pointer to self
 * Return     : whether there's a pending overflow.
 **********************************************************************************************************************/
static bool is_overflow_pending(hw_timer_manager_t* p_interface)
{
    hw_timer_manager_impl_t* p_this = (hw_timer_manager_impl_t*)p_interface;

    return LL_TIM_IsActiveFlag_UPDATE(p_this->_p_conf->p_instance)
           && LL_TIM_IsEnabledIT_UPDATE(p_this->_p_conf->p_instance);
}

/**********************************************************************************************************************/
/* Protected Function Definitions                                                                                     */
/**********************************************************************************************************************/

/* ------------------------------------- IRQ Handlers ----------------------------------------------------------------*/

/***********************************************************************************************************************
 * Description: General TIMn IRQ handler
 * Input      : p_this - pointer to self
 * Notes      : not all events are checked, since they are not needed, add more flag checks as they are needed.
 **********************************************************************************************************************/
void hw_timer_manager_irq(hw_timer_manager_impl_t* p_this)
{
    uint32_t event = 0;

    TIM_TypeDef* p_instance = p_this->_p_conf->p_instance;

    if (LL_TIM_IsActiveFlag_UPDATE(p_instance) && LL_TIM_IsEnabledIT_UPDATE(p_instance))
    {
        LL_TIM_ClearFlag_UPDATE(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_UPDATE;
    }

    if (LL_TIM_IsActiveFlag_CC1(p_instance) && LL_TIM_IsEnabledIT_CC1(p_instance))
    {
        LL_TIM_ClearFlag_CC1(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC1;
    }

    if (LL_TIM_IsActiveFlag_CC2(p_instance) && LL_TIM_IsEnabledIT_CC2(p_instance))
    {
        LL_TIM_ClearFlag_CC2(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC2;
    }

    if (LL_TIM_IsActiveFlag_CC3(p_instance) && LL_TIM_IsEnabledIT_CC3(p_instance))
    {
        LL_TIM_ClearFlag_CC3(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC3;
    }

    if (LL_TIM_IsActiveFlag_CC4(p_instance) && LL_TIM_IsEnabledIT_CC4(p_instance))
    {
        LL_TIM_ClearFlag_CC4(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC4;
    }

    if (LL_TIM_IsActiveFlag_CC1OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC1OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC1_OVERFLOW;
    }

    if (LL_TIM_IsActiveFlag_CC2OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC2OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC2_OVERFLOW;
    }

    if (LL_TIM_IsActiveFlag_CC3OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC3OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC3_OVERFLOW;
    }

    if (LL_TIM_IsActiveFlag_CC4OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC4OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC4_OVERFLOW;
    }

    p_this->_base.__event_callback(p_this->_base.__event_callback_arg, event);
}

/***********************************************************************************************************************
 * Description: Timer 1/8 IRQ handler - Break IRQs
 * Input      : p_this - pointer to self
 * Notes      : not all events are checked, since they are not needed, add more flag checks as they are needed.
 **********************************************************************************************************************/
void hw_timer_manager_irq_brk(hw_timer_manager_impl_t* p_this)
{
    ARGUMENT_UNUSED(p_this);
}

/***********************************************************************************************************************
 * Description: Timer 1/8 IRQ handler - Update IRQs
 * Input      : p_this - pointer to self
 * Notes      : not all events are checked, since they are not needed, add more flag checks as they are needed.
 **********************************************************************************************************************/
void hw_timer_manager_irq_up(hw_timer_manager_impl_t* p_this)
{
    uint32_t event = 0;

    TIM_TypeDef* p_instance = p_this->_p_conf->p_instance;

    if (LL_TIM_IsActiveFlag_UPDATE(p_instance) && LL_TIM_IsEnabledIT_UPDATE(p_instance))
    {
        LL_TIM_ClearFlag_UPDATE(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_UPDATE;
    }

    p_this->_base.__event_callback(p_this->_base.__event_callback_arg, event);
}

/***********************************************************************************************************************
 * Description: Timer 1/8 IRQ handler - Trigger and Commutation IRQs
 * Input      : p_this - pointer to self
 * Notes      : not all events are checked, since they are not needed, add more flag checks as they are needed.
 **********************************************************************************************************************/
void hw_timer_manager_irq_trg_com(hw_timer_manager_impl_t* p_this)
{
    ARGUMENT_UNUSED(p_this);
}

/***********************************************************************************************************************
 * Description: Timer 1/8 IRQ handler - Capture Compare IRQs
 * Input      : p_this - pointer to self
 * Notes      : not all events are checked, since they are not needed, add more flag checks as they are needed.
 **********************************************************************************************************************/
void hw_timer_manager_irq_cc(hw_timer_manager_impl_t* p_this)
{
    uint32_t event = 0;

    TIM_TypeDef* p_instance = p_this->_p_conf->p_instance;

    if (LL_TIM_IsActiveFlag_CC1(p_instance) && LL_TIM_IsEnabledIT_CC1(p_instance))
    {
        LL_TIM_ClearFlag_CC1(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC1;
    }

    if (LL_TIM_IsActiveFlag_CC2(p_instance) && LL_TIM_IsEnabledIT_CC2(p_instance))
    {
        LL_TIM_ClearFlag_CC2(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC2;
    }

    if (LL_TIM_IsActiveFlag_CC3(p_instance) && LL_TIM_IsEnabledIT_CC3(p_instance))
    {
        LL_TIM_ClearFlag_CC3(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC3;
    }

    if (LL_TIM_IsActiveFlag_CC4(p_instance) && LL_TIM_IsEnabledIT_CC4(p_instance))
    {
        LL_TIM_ClearFlag_CC4(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC4;
    }

    if (LL_TIM_IsActiveFlag_CC1OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC1OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC1_OVERFLOW;
    }

    if (LL_TIM_IsActiveFlag_CC2OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC2OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC2_OVERFLOW;
    }

    if (LL_TIM_IsActiveFlag_CC3OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC3OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC3_OVERFLOW;
    }

    if (LL_TIM_IsActiveFlag_CC4OVR(p_instance))
    {
        LL_TIM_ClearFlag_CC4OVR(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC4_OVERFLOW;
    }

    p_this->_base.__event_callback(p_this->_base.__event_callback_arg, event);
}

/**********************************************************************************************************************/
/* Private Function Definitions                                                                                       */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Returns the frequency of the HW Timer's source clock
 * Input      : p_this - pointer to self
 * Return     : HW Timer's source clock frequency
 * Notes      : if the configured instance is unsupported, returns 0
 **********************************************************************************************************************/
__STATIC_INLINE uint32_t get_src_clk_freq(hw_timer_manager_impl_t* p_this)
{
    RCC_ClkInitTypeDef rcc_conf  = {0};
    uint32_t           f_latency = 0;

    HAL_RCC_GetClockConfig(&rcc_conf, &f_latency);

    RET_VAL_UNUSED(f_latency);

    switch ((uint32_t)p_this->_p_conf->p_instance)
    {
        /* In all currently supported ST families,
         * TIM2-TIM7 are fed from PCLK1. */
#if defined(TIM2)
        case (uint32_t)TIM2:
#endif
#if defined(TIM3)
        case (uint32_t)TIM3:
#endif
#if defined(TIM4)
        case (uint32_t)TIM4:
#endif
#if defined(TIM5)
        case (uint32_t)TIM5:
#endif
#if defined(TIM6)
        case (uint32_t)TIM6:
#endif
#if defined(TIM7)
        case (uint32_t)TIM7:
#endif
            /* In STM32F4, TIM12-TIM14 are also fed from PCLK1. */
#if defined(TIM12)
        case (uint32_t)TIM12:
#endif
#if defined(TIM13)
        case (uint32_t)TIM13:
#endif
#if defined(TIM14)
        case (uint32_t)TIM14:
#endif
        {
            /* These timers are fed from PCLK1, controlled by the APB1 prescaler */

            /* The timer clock frequencies are automatically defined by hardware. There are two cases:
             * 1. If the APB prescaler equals 1, the timer clock frequencies are set to the same frequency as that of
             * the APB domain.
             * 2. Otherwise, they are set to twice (x2) the frequency of the APB domain. */
            return HAL_RCC_GetPCLK1Freq() * ((rcc_conf.APB1CLKDivider == RCC_HCLK_DIV1) ? 1 : 2);
        }
#if defined(TIM1)
        case (uint32_t)TIM1:
#endif
#if defined(TIM8)
        case (uint32_t)TIM8:
#endif
#if defined(TIM9)
        case (uint32_t)TIM9:
#endif
#if defined(TIM10)
        case (uint32_t)TIM10:
#endif
#if defined(TIM11)
        case (uint32_t)TIM11:
#endif
#if defined(TIM15)
        case (uint32_t)TIM15:
#endif
#if defined(TIM16)
        case (uint32_t)TIM16:
#endif
#if defined(TIM17)
        case (uint32_t)TIM17:
#endif
        {
            /* These timers are fed from PCLK2, controlled by the APB2 prescaler */

            /* The timer clock frequencies are automatically defined by hardware. There are two cases:
             * 1. If the APB prescaler equals 1, the timer clock frequencies are set to the same frequency as that of
             * the APB domain.
             * 2. Otherwise, they are set to twice (x2) the frequency of the APB domain. */
            return HAL_RCC_GetPCLK2Freq() * ((rcc_conf.APB2CLKDivider == RCC_HCLK_DIV1) ? 1 : 2);
        }
        default:
        {
            proj_assert(false);
            return 0;
        }
    }
}

/* ---------------------- Timer NVIC enable functions (mostly needed for TIM1 and TIM15/16/17) ---------------------- */

/* TIM2,3,4,5,7 - General purpose and basic timers - don't share interrupts in devices we've seen so far */

#if defined(TIM2)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(2)
#endif

#if defined(TIM3)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(3)
#endif

#if defined(TIM4)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(4)
#endif

#if defined(TIM5)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(5)
#endif

#if defined(TIM7)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(7)
#endif

/* TIM6 - In STM32F4 & STM32L4, shares interrupt handler with the DAC */
#if defined(TIM6)
#if defined(STM32F4) || defined(STM32L4)
void hw_timer_manager_nvic_enable_6(void)
{
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
void hw_timer_manager_nvic_disable_6(void)
{
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
}
#else
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(6)
#endif
#endif

#if defined(STM32U5)
#if defined(TIM1)
void hw_timer_manager_nvic_enable_1(void)
{
    NVIC_ClearPendingIRQ(TIM1_BRK_IRQn);
    NVIC_EnableIRQ(TIM1_BRK_IRQn);

    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    NVIC_ClearPendingIRQ(TIM1_TRG_COM_IRQn);
    NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);

    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void hw_timer_manager_nvic_disable_1(void)
{
    NVIC_DisableIRQ(TIM1_BRK_IRQn);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
    NVIC_DisableIRQ(TIM1_CC_IRQn);
}
#endif

#if defined(TIM8)
void hw_timer_manager_nvic_enable_8(void)
{
    NVIC_ClearPendingIRQ(TIM8_BRK_IRQn);
    NVIC_EnableIRQ(TIM8_BRK_IRQn);

    NVIC_ClearPendingIRQ(TIM8_UP_IRQn);
    NVIC_EnableIRQ(TIM8_UP_IRQn);

    NVIC_ClearPendingIRQ(TIM8_TRG_COM_IRQn);
    NVIC_EnableIRQ(TIM8_TRG_COM_IRQn);

    NVIC_ClearPendingIRQ(TIM8_CC_IRQn);
    NVIC_EnableIRQ(TIM8_CC_IRQn);
}

void hw_timer_manager_nvic_disable_8(void)
{
    NVIC_DisableIRQ(TIM8_BRK_IRQn);
    NVIC_DisableIRQ(TIM8_UP_IRQn);
    NVIC_DisableIRQ(TIM8_TRG_COM_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
}
#endif

#if defined(TIM15)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(15)
#endif

#if defined(TIM16)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(16)
#endif

#if defined(TIM17)
HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(17)
#endif

#endif

#if defined(STM32L4)
#if defined(TIM1)
void hw_timer_manager_nvic_enable_1(void)
{
#if defined(TIM15) || defined(TIM16) || defined(TIM17)
    uint32_t flag_prev;
#endif

#if defined(TIM15)
    flag_prev = atomic_fetch_add(&flag_tim1_tim15, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_BRK_TIM15_IRQn);
        NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
#if defined(TIM15)
    }
#endif

#if defined(TIM16)
    flag_prev = atomic_fetch_add(&flag_tim1_tim16, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
        NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
#if defined(TIM16)
    }
#endif

#if defined(TIM17)
    flag_prev = atomic_fetch_add(&flag_tim1_tim17, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM17_IRQn);
        NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
#if defined(TIM17)
    }
#endif

    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void hw_timer_manager_nvic_disable_1(void)
{
#if defined(TIM15) || defined(TIM16) || defined(TIM17)
    uint32_t flag_prev;
#endif

#if defined(TIM15)
    flag_prev = atomic_fetch_sub(&flag_tim1_tim15, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
#if defined(TIM15)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

#if defined(TIM16)
    flag_prev = atomic_fetch_sub(&flag_tim1_tim16, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
#if defined(TIM16)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

#if defined(TIM17)
    flag_prev = atomic_fetch_sub(&flag_tim1_tim17, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
#if defined(TIM17)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

    NVIC_DisableIRQ(TIM1_CC_IRQn);
}
#endif

#if defined(TIM8)
void hw_timer_manager_nvic_enable_8(void)
{
    NVIC_ClearPendingIRQ(TIM8_BRK_IRQn);
    NVIC_EnableIRQ(TIM8_BRK_IRQn);

    NVIC_ClearPendingIRQ(TIM8_UP_IRQn);
    NVIC_EnableIRQ(TIM8_UP_IRQn);

    NVIC_ClearPendingIRQ(TIM8_TRG_COM_IRQn);
    NVIC_EnableIRQ(TIM8_TRG_COM_IRQn);

    NVIC_ClearPendingIRQ(TIM8_CC_IRQn);
    NVIC_EnableIRQ(TIM8_CC_IRQn);
}

void hw_timer_manager_nvic_disable_8(void)
{
    NVIC_DisableIRQ(TIM8_BRK_IRQn);
    NVIC_DisableIRQ(TIM8_UP_IRQn);
    NVIC_DisableIRQ(TIM8_TRG_COM_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
}
#endif

#if defined(TIM15)
void hw_timer_manager_nvic_enable_15(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim1_tim15, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_BRK_TIM15_IRQn);
        NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
#if defined(TIM1)
    }
#endif
}

void hw_timer_manager_nvic_disable_15(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim1_tim15, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
#if defined(TIM1)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM16)
void hw_timer_manager_nvic_enable_16(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim1_tim16, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
        NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
#if defined(TIM1)
    }
#endif
}

void hw_timer_manager_nvic_disable_16(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim1_tim16, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
#if defined(TIM1)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM17)
void hw_timer_manager_nvic_enable_17(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim1_tim17, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM17_IRQn);
        NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
#if defined(TIM1)
    }
#endif
}

void hw_timer_manager_nvic_disable_17(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim1_tim17, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
#if defined(TIM1)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif
#endif /* STM32L4 */

#if defined(STM32F4)
#if defined(TIM1)
void hw_timer_manager_nvic_enable_1(void)
{
#if defined(TIM9) || defined(TIM10) || defined(TIM11)
    uint32_t flag_prev;
#endif

#if defined(TIM9)
    flag_prev = atomic_fetch_add(&flag_tim1_tim9, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
#if defined(TIM9)
    }
#endif

#if defined(TIM10)
    flag_prev = atomic_fetch_add(&flag_tim1_tim10, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
#if defined(TIM10)
    }
#endif

#if defined(TIM11)
    flag_prev = atomic_fetch_add(&flag_tim1_tim11, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
        NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
#if defined(TIM11)
    }
#endif

    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void hw_timer_manager_nvic_disable_1(void)
{
#if defined(TIM9) || defined(TIM10) || defined(TIM11)
    uint32_t flag_prev;
#endif

#if defined(TIM9)
    flag_prev = atomic_fetch_sub(&flag_tim1_tim9, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
#if defined(TIM9)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

#if defined(TIM10)
    flag_prev = atomic_fetch_sub(&flag_tim1_tim10, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
#if defined(TIM10)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

#if defined(TIM11)
    flag_prev = atomic_fetch_sub(&flag_tim1_tim11, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
#if defined(TIM11)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

    NVIC_DisableIRQ(TIM1_CC_IRQn);
}
#endif

#if defined(TIM9)
void hw_timer_manager_nvic_enable_9(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim1_tim9, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
#if defined(TIM1)
    }
#endif
}

void hw_timer_manager_nvic_disable_9(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim1_tim9, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
#if defined(TIM1)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM10)
void hw_timer_manager_nvic_enable_10(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim1_tim10, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
#if defined(TIM1)
    }
#endif
}

void hw_timer_manager_nvic_disable_10(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim1_tim10, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
#if defined(TIM1)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM11)
void hw_timer_manager_nvic_enable_11(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim1_tim11, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
        NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
#if defined(TIM1)
    }
#endif
}

void hw_timer_manager_nvic_disable_11(void)
{
#if defined(TIM1)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim1_tim11, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
#if defined(TIM1)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM8)
void hw_timer_manager_nvic_enable_8(void)
{
#if defined(TIM12) || defined(TIM13) || defined(TIM14)
    uint32_t flag_prev;
#endif

#if defined(TIM12)
    flag_prev = atomic_fetch_add(&flag_tim8_tim12, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
        NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
#if defined(TIM12)
    }
#endif

#if defined(TIM13)
    flag_prev = atomic_fetch_add(&flag_tim8_tim13, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
        NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
#if defined(TIM13)
    }
#endif

#if defined(TIM14)
    flag_prev = atomic_fetch_add(&flag_tim8_tim14, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM8_TRG_COM_TIM14_IRQn);
        NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
#if defined(TIM14)
    }
#endif

    NVIC_ClearPendingIRQ(TIM8_CC_IRQn);
    NVIC_EnableIRQ(TIM8_CC_IRQn);
}

void hw_timer_manager_nvic_disable_8(void)
{
#if defined(TIM12) || defined(TIM13) || defined(TIM14)
    uint32_t flag_prev;
#endif

#if defined(TIM12)
    flag_prev = atomic_fetch_sub(&flag_tim8_tim12, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
#if defined(TIM12)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

#if defined(TIM13)
    flag_prev = atomic_fetch_sub(&flag_tim8_tim13, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
#if defined(TIM13)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

#if defined(TIM14)
    flag_prev = atomic_fetch_sub(&flag_tim8_tim14, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
#if defined(TIM14)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif

    NVIC_DisableIRQ(TIM1_CC_IRQn);
}
#endif

#if defined(TIM12)
void hw_timer_manager_nvic_enable_12(void)
{
#if defined(TIM8)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim8_tim12, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
        NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
#if defined(TIM8)
    }
#endif
}

void hw_timer_manager_nvic_disable_12(void)
{
#if defined(TIM8)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim8_tim12, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
#if defined(TIM8)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM13)
void hw_timer_manager_nvic_enable_13(void)
{
#if defined(TIM8)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim8_tim13, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
        NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
#if defined(TIM8)
    }
#endif
}

void hw_timer_manager_nvic_disable_13(void)
{
#if defined(TIM8)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim8_tim13, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
#if defined(TIM8)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif

#if defined(TIM14)
void hw_timer_manager_nvic_enable_14(void)
{
#if defined(TIM8)
    uint32_t flag_prev = atomic_fetch_add(&flag_tim8_tim14, 1);
    if (flag_prev == 0)
    {
#endif
        NVIC_ClearPendingIRQ(TIM8_TRG_COM_TIM14_IRQn);
        NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
#if defined(TIM8)
    }
#endif
}

void hw_timer_manager_nvic_disable_14(void)
{
#if defined(TIM8)
    uint32_t flag_prev = atomic_fetch_sub(&flag_tim8_tim14, 1);
    if (flag_prev == 1)
    {
#endif
        NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
#if defined(TIM8)
    }
    else if (flag_prev == 0)
    {
        proj_exception();
    }
#endif
}
#endif
#endif /* STM32F4 */

/* STM32F1 MCUs timer interrupts depend on flash size.
 * STM32F105, STM32F107 and MCUs with 768kB - 1MB flash (XL-density), are different than the rest. */
#if defined(STM32F1)
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB)                       \
    || defined(STM32F101xE) || defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6)                    \
    || defined(STM32F103xB) || defined(STM32F103xE)

void hw_timer_manager_nvic_enable_1(void)
{
    NVIC_ClearPendingIRQ(TIM1_BRK_IRQn);
    NVIC_EnableIRQ(TIM1_BRK_IRQn);

    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    NVIC_ClearPendingIRQ(TIM1_TRG_COM_IRQn);
    NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);

    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void hw_timer_manager_nvic_disable_1(void)
{
    NVIC_DisableIRQ(TIM1_BRK_IRQn);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
    NVIC_DisableIRQ(TIM1_CC_IRQn);
}

#endif /* Other STM32F1 devices */
#endif /* STM32F1 */

#if defined(STM32WL)
void hw_timer_manager_nvic_enable_1(void)
{
    NVIC_ClearPendingIRQ(TIM1_BRK_IRQn);
    NVIC_EnableIRQ(TIM1_BRK_IRQn);

    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    NVIC_ClearPendingIRQ(TIM1_TRG_COM_IRQn);
    NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);

    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void hw_timer_manager_nvic_disable_1(void)
{
    NVIC_DisableIRQ(TIM1_BRK_IRQn);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
    NVIC_DisableIRQ(TIM1_CC_IRQn);
}

HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(16);

HW_TIMER_MANAGER_NVIC_CONTROL_DEFINE(17);
#endif /* STM32WL */