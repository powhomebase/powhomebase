/***********************************************************************************************************************
 * Description: Implementation file of HW Timer Manager for STM32L4 and STM32WL55.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own Header */
#include "lp_hw_timer_manager_impl.h"

/* Libraries */
#if defined(STM32L4)
#include <stm32l4xx_ll_lptim.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_lptim.h>
#endif

/* Project Related */
#include <global.h>

/* Some ST's have an LPTIM peripheral and some not. This define indicates if we have it or not */
#if defined(LPTIM1)

/**********************************************************************************************************************/
/* Interface Virtual Method Declarations                                                                              */
/**********************************************************************************************************************/

static int32_t  initialize(hw_timer_manager_t* p_interface, hw_timer_event_cb_t callback, void* arg);
static int32_t  uninitialize(hw_timer_manager_t* p_interface);
static int32_t  power_control(hw_timer_manager_t* p_interface, ARM_POWER_STATE state);
static uint32_t get_steps_current(hw_timer_manager_t* p_interface);

/**********************************************************************************************************************/
/* Private Function Definitions                                                                                       */
/**********************************************************************************************************************/

/* Get Timer's source clock frequency */
__STATIC_INLINE uint32_t get_src_clk_freq(lp_hw_timer_manager_impl_t* p_this);

/* Calculate prescaler config param, also check if wanted config is at all possible */
__STATIC_INLINE uint32_t calc_prescaler(lp_hw_timer_manager_impl_t* p_this, bool* p_is_possible);

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Virtual Methods */
hw_timer_manager_methods_t const lp_hw_timer_manager_impl_m = {
    .Initialize      = initialize,
    .Uninitialize    = uninitialize,
    .PowerControl    = power_control,
    .GetStepsCurrent = get_steps_current,
};

/**********************************************************************************************************************/
/* Interface Virtual Method Definitions                                                                               */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: initializes the LP HW Timer Manager
 * Input      : p_interface - pointer to self
 *              callback    - event callback
 *              arg         - event callback argument
 * Return     : ARM_DRIVER return code
 * Notes      : performs some checks on the configuration, then initializes the HW Timer
 **********************************************************************************************************************/
static int32_t initialize(hw_timer_manager_t* p_interface, hw_timer_event_cb_t callback, void* arg)
{
    lp_hw_timer_manager_impl_t* p_this = (lp_hw_timer_manager_impl_t*)p_interface;

    p_this->_base._src_clk_freq = get_src_clk_freq(p_this);
    if (p_this->_base._src_clk_freq == 0)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    bool     is_step_time_possible = false;
    uint32_t prescaler             = calc_prescaler(p_this, &is_step_time_possible);
    if (!is_step_time_possible)

    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    LL_LPTIM_InitTypeDef init_conf;
    LL_LPTIM_StructInit(&init_conf);
    init_conf.ClockSource = LL_LPTIM_CLK_SOURCE_INTERNAL;
    init_conf.Prescaler   = prescaler;

    /* Enable timer clock only for configuration sequence */
    p_this->_p_conf->clock_enable();

    /* Initialize and configure LPTIM */
    ErrorStatus res = LL_LPTIM_Init(p_this->_p_conf->p_instance, &init_conf);

    if (res != SUCCESS)
    {
        p_this->_p_conf->clock_disable();
        return ARM_DRIVER_ERROR;
    }

    /* Enable LPTIM to modify ARR register */
    LL_LPTIM_Enable(p_this->_p_conf->p_instance);

    LL_LPTIM_SetAutoReload(p_this->_p_conf->p_instance, p_this->_base._p_conf->steps_per_period - 1);
    while (!LL_LPTIM_IsActiveFlag_ARROK(p_this->_p_conf->p_instance)) continue;
    LL_LPTIM_ClearFlag_ARROK(p_this->_p_conf->p_instance);

    LL_LPTIM_Disable(p_this->_p_conf->p_instance);

    /* Clear all pending interrupts and enable update interrupt */
    LL_LPTIM_ClearFLAG_ARRM(p_this->_p_conf->p_instance);
    LL_LPTIM_EnableIT_ARRM(p_this->_p_conf->p_instance);

    p_this->_p_conf->clock_disable();

    NVIC_ClearPendingIRQ(p_this->_p_conf->irq_n);
    NVIC_EnableIRQ(p_this->_p_conf->irq_n);

    p_interface->__event_callback     = callback;
    p_interface->__event_callback_arg = arg;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: uninitializes the LP HW Timer Manager
 * Input      : p_interface - pointer to self
 * Return     : ARM_DRIVER return code
 * Notes      : Disables the timer and restores HW Timer to default state
 **********************************************************************************************************************/
static int32_t uninitialize(hw_timer_manager_t* p_interface)
{
    lp_hw_timer_manager_impl_t* p_this = (lp_hw_timer_manager_impl_t*)p_interface;

    p_interface->__event_callback     = NULL;
    p_interface->__event_callback_arg = NULL;

    NVIC_DisableIRQ(p_this->_p_conf->irq_n);

    p_this->_p_conf->clock_enable();

    ErrorStatus res = LL_LPTIM_DeInit(p_this->_p_conf->p_instance);

    p_this->_p_conf->clock_disable();

    return ((res == SUCCESS) ? ARM_DRIVER_OK : ARM_DRIVER_ERROR);
}

/***********************************************************************************************************************
 * Description: controls power state of the LP HW Timer
 * Input      : p_interface - pointer to self
 *              state       - desired power state
 * Return     : ARM_DRIVER return code
 **********************************************************************************************************************/
static int32_t power_control(hw_timer_manager_t* p_interface, ARM_POWER_STATE state)
{
    lp_hw_timer_manager_impl_t* p_this = (lp_hw_timer_manager_impl_t*)p_interface;

    switch (state)
    {
        case ARM_POWER_FULL:
        {
            /* Already running? nothing to change */
            if (p_this->_base._state == HW_TIMER_MANAGER_STATE_RUNNING_LOW_POWER)
            {
                return ARM_DRIVER_OK;
            }
            /* Not running? start running */
            else if (p_this->_base._state == HW_TIMER_MANAGER_STATE_INITIALIZED)
            {
                p_this->_p_conf->clock_enable();

                LL_LPTIM_Enable(p_this->_p_conf->p_instance);
                LL_LPTIM_StartCounter(p_this->_p_conf->p_instance, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

                return ARM_DRIVER_OK;
            }
            /* WTF? */
            else
            {
                proj_assert(false);
                return ARM_DRIVER_ERROR;
            }
        }
        case ARM_POWER_LOW:
        {
#if defined(LPTIM1)
            /* Only LPTIM1 can continue running in low power mode */
            if (p_this->_p_conf->p_instance != LPTIM1)
            {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            /* Can run in low power mode only off of LSE and LSI */
            else if (__HAL_RCC_GET_LPTIM1_SOURCE() != RCC_LPTIM1CLKSOURCE_LSE
                     && __HAL_RCC_GET_LPTIM1_SOURCE() != RCC_LPTIM1CLKSOURCE_LSI)
            {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            /* Already running? nothing to change */
            else if (p_this->_base._state == HW_TIMER_MANAGER_STATE_RUNNING)
            {
                return ARM_DRIVER_OK;
            }
            /* Not running? start running */
            else if (p_this->_base._state == HW_TIMER_MANAGER_STATE_INITIALIZED)
            {
                p_this->_p_conf->clock_enable();

                LL_LPTIM_Enable(p_this->_p_conf->p_instance);
                LL_LPTIM_StartCounter(p_this->_p_conf->p_instance, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

                return ARM_DRIVER_OK;
            }
            /* WTF? */
            else
            {
                proj_assert(false);
                return ARM_DRIVER_ERROR;
            }
#else
            /* LPTIM2 doesn't run in low power modes, don't bother checking if LPTIM1 doesn't exist */
            return ARM_DRIVER_ERROR_UNSUPPORTED;
#endif
        }
        case ARM_POWER_OFF:
        {
            LL_LPTIM_Disable(p_this->_p_conf->p_instance);

            p_this->_p_conf->clock_disable();

            return ARM_DRIVER_OK;
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
    lp_hw_timer_manager_impl_t* p_this = (lp_hw_timer_manager_impl_t*)p_interface;

    uint32_t res = LL_LPTIM_GetCounter(p_this->_p_conf->p_instance);
    uint32_t prev_res;

    /* If the timer is running off an asynchronous clock, we have to read until two readings give the same result */
    do
    {
        prev_res = res;
        res      = LL_LPTIM_GetCounter(p_this->_p_conf->p_instance);
    } while (res != prev_res);

    return res;
}

/**********************************************************************************************************************/
/* Protected Function Definitions                                                                                     */
/**********************************************************************************************************************/

/* ------------------------------------- IRQ Handlers ----------------------------------------------------------------*/

/***********************************************************************************************************************
 * Description: LP Timer 1 and 2 IRQ handler
 * Input      : p_this - pointer to self
 * Return     : None.
 **********************************************************************************************************************/
void lp_hw_timer_manager_irq(lp_hw_timer_manager_impl_t* p_this)
{
    uint32_t event = 0;

    LPTIM_TypeDef* p_instance = p_this->_p_conf->p_instance;

    if (LL_LPTIM_IsActiveFlag_CMPM(p_instance) && LL_LPTIM_IsEnabledIT_CMPM(p_instance))
    {
        LL_LPTIM_ClearFLAG_CMPM(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_CC1;
    }

    if (LL_LPTIM_IsActiveFlag_ARRM(p_instance) && LL_LPTIM_IsEnabledIT_ARRM(p_instance))
    {
        LL_LPTIM_ClearFLAG_ARRM(p_instance);
        event |= 1U << HW_TIMER_MANAGER_EVENT_SHIFT_UPDATE;
    }

    if (LL_LPTIM_IsActiveFlag_EXTTRIG(p_instance) && LL_LPTIM_IsEnabledIT_EXTTRIG(p_instance))
    {
        LL_LPTIM_ClearFlag_EXTTRIG(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_EXT_TRIG;
    }

    if (LL_LPTIM_IsActiveFlag_ARROK(p_instance) && LL_LPTIM_IsEnabledIT_ARROK(p_instance))
    {
        LL_LPTIM_ClearFlag_ARROK(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_ARR_OK;
    }

    if (LL_LPTIM_IsActiveFlag_CMPOK(p_instance) && LL_LPTIM_IsEnabledIT_CMPOK(p_instance))
    {
        LL_LPTIM_ClearFlag_CMPOK(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_COMPARE_OK;
    }

    if (LL_LPTIM_IsActiveFlag_UP(p_instance) && LL_LPTIM_IsEnabledIT_UP(p_instance))
    {
        LL_LPTIM_ClearFlag_UP(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_UP;
    }

    if (LL_LPTIM_IsActiveFlag_DOWN(p_instance) && LL_LPTIM_IsEnabledIT_DOWN(p_instance))
    {
        LL_LPTIM_ClearFlag_DOWN(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_DOWN;
    }

#if defined(LPTIM_RCR_REP)
    if (LL_LPTIM_IsActiveFlag_UE(p_instance) && LL_LPTIM_IsEnabledIT_UE(p_instance))
    {
        LL_LPTIM_ClearFlag_UE(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_DOWN;
    }

    if (LL_LPTIM_IsActiveFlag_REPOK(p_instance) && LL_LPTIM_IsEnabledIT_REPOK(p_instance))
    {
        LL_LPTIM_ClearFlag_REPOK(p_instance);
        event |= 1U << LP_HW_TIMER_MANAGER_IMPL_EVENT_SHIFT_DOWN;
    }
#endif

    p_this->_base.__event_callback(p_this->_base.__event_callback_arg, event);
}

/**********************************************************************************************************************/
/* Private Function Definitions                                                                                       */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: returns the frequency of the HW Timer's source clock
 * Input      : p_this - pointer to self
 * Return     : HW Timer's source clock frequency
 * Notes      : if the configured instance is unsupported, returns 0
 **********************************************************************************************************************/
__STATIC_INLINE uint32_t get_src_clk_freq(lp_hw_timer_manager_impl_t* p_this)
{
    switch ((uint32_t)p_this->_p_conf->p_instance)
    {
#if defined(LPTIM1)
        case (uint32_t)LPTIM1:
        {
            uint32_t tim_clk_src = __HAL_RCC_GET_LPTIM1_SOURCE();

            switch (tim_clk_src)
            {
                case RCC_LPTIM1CLKSOURCE_HSI:
                {
                    return HSI_VALUE;
                }
                case RCC_LPTIM1CLKSOURCE_LSI:
                {
                    return LSI_VALUE;
                }
                case RCC_LPTIM1CLKSOURCE_LSE:
                {
                    return LSE_VALUE;
                }
                case RCC_LPTIM1CLKSOURCE_PCLK1:
                {
                    return HAL_RCC_GetPCLK1Freq();
                }
                default:
                {
                    proj_assert(false);
                    return 0;
                }
            }
        }
#endif
#if defined(LPTIM2)
        case (uint32_t)LPTIM2:
        {
            uint32_t tim_clk_src = __HAL_RCC_GET_LPTIM2_SOURCE();

            switch (tim_clk_src)
            {
                case RCC_LPTIM2CLKSOURCE_HSI:
                {
                    return HSI_VALUE;
                }
                case RCC_LPTIM2CLKSOURCE_LSI:
                {
                    return LSI_VALUE;
                }
                case RCC_LPTIM2CLKSOURCE_LSE:
                {
                    return LSE_VALUE;
                }
                case RCC_LPTIM2CLKSOURCE_PCLK1:
                {
                    return HAL_RCC_GetPCLK1Freq();
                }
                default:
                {
                    proj_assert(false);
                    return 0;
                }
            }
        }
#endif
        default:
        {
            proj_assert(false);
            return 0;
        }
    }
}

/***********************************************************************************************************************
 * Description: Calculates necessary prescaler to achieve requested step time.
 * Input      : p_this        - pointer to self
 *              p_is_possible - return value - is the configured step time possible
 * Return     : prescaler conf value
 * Notes      : this function should be called *after* `p_this->_base._src_clk_freq` is set.
 **********************************************************************************************************************/
__STATIC_INLINE uint32_t calc_prescaler(lp_hw_timer_manager_impl_t* p_this, bool* p_is_possible)
{
    uint32_t div = hw_timer_manager_calc_prescaler_factor(&(p_this->_base), p_is_possible);

    if (!(*p_is_possible))
    {
        return 0;
    }

    switch (div)
    {
        case 1:
        {
            return LPTIM_PRESCALER_DIV1;
        }
        case 2:
        {
            return LPTIM_PRESCALER_DIV2;
        }
        case 4:
        {
            return LPTIM_PRESCALER_DIV4;
        }
        case 8:
        {
            return LPTIM_PRESCALER_DIV8;
        }
        case 16:
        {
            return LPTIM_PRESCALER_DIV16;
        }
        case 32:
        {
            return LPTIM_PRESCALER_DIV32;
        }
        case 64:
        {
            return LPTIM_PRESCALER_DIV64;
        }
        case 128:
        {
            return LPTIM_PRESCALER_DIV128;
        }
        default:
        {
            *p_is_possible = false;
            return 0;
        }
    }
}

#endif