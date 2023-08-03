/***********************************************************************************************************************
 * Description: HW Timer Manager API inline functions translation unit
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own Header */
#include "hw_timer_manager.h"

/* Library Headers */
#include <math.h>

/**********************************************************************************************************************/
/* Interface Function Definitions                                                                                     */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Initializes the HW Timer Manager
 * Input      : p_this    - pointer to self
 *              callback  - event callback
 *              arg       - event callback argument
 * Return     : ARM_DRIVER return code
 * Notes      : performs some checks on the configuration, then initializes the HW Timer
 **********************************************************************************************************************/
int32_t hw_timer_manager_initialize(hw_timer_manager_t* p_this, hw_timer_event_cb_t callback, void* arg)
{
    proj_assert(p_this != NULL);
    proj_assert(p_this->_p_conf != NULL);
    proj_assert(p_this->_p_conf->m != NULL);
    proj_assert(p_this->_p_conf->m->Initialize != NULL);

    proj_assert(callback != NULL);

    if (p_this->_state != HW_TIMER_MANAGER_STATE_UNINITIALIZED)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    int32_t res = p_this->_p_conf->m->Initialize(p_this, callback, arg);

    if (res == ARM_DRIVER_OK)
    {
        p_this->_state = HW_TIMER_MANAGER_STATE_INITIALIZED;
    }
    else
    {
        p_this->_state = HW_TIMER_MANAGER_STATE_ERROR;
    }

    return res;
}

/***********************************************************************************************************************
 * Description: Uninitializes the HW Timer Manager
 * Input      : p_this - pointer to self
 * Return     : ARM_DRIVER return code
 * Notes      : Disables the timer and restores HW Timer to default state
 **********************************************************************************************************************/
int32_t hw_timer_manager_uninitialize(hw_timer_manager_t* p_this)
{
    proj_assert(p_this != NULL);
    proj_assert(p_this->_p_conf != NULL);
    proj_assert(p_this->_p_conf->m != NULL);
    proj_assert(p_this->_p_conf->m->Uninitialize != NULL);

    if (p_this->_state != HW_TIMER_MANAGER_STATE_INITIALIZED)
    {
        proj_assert(false);
        return ARM_DRIVER_ERROR;
    }

    int32_t res = p_this->_p_conf->m->Uninitialize(p_this);

    if (res == ARM_DRIVER_OK)
    {
        p_this->_state = HW_TIMER_MANAGER_STATE_UNINITIALIZED;
    }
    else
    {
        p_this->_state = HW_TIMER_MANAGER_STATE_ERROR;
    }

    return res;
}

/***********************************************************************************************************************
 * Description: Controls power state of the HW Timer
 * Input      : p_this - pointer to self
 *              state  - desired power state
 * Return     : ARM_DRIVER return code
 **********************************************************************************************************************/
int32_t hw_timer_manager_power_control(hw_timer_manager_t* p_this, ARM_POWER_STATE state)
{
    proj_assert(p_this != NULL);
    proj_assert(p_this->_p_conf != NULL);
    proj_assert(p_this->_p_conf->m != NULL);
    proj_assert(p_this->_p_conf->m->PowerControl != NULL);

    switch (p_this->_state)
    {
        case HW_TIMER_MANAGER_STATE_RUNNING:
        {
            if (state == ARM_POWER_FULL)
            {
                /* Already running at full power! */
                proj_assert(false);
                return ARM_DRIVER_ERROR;
            }

            break;
        }
        case HW_TIMER_MANAGER_STATE_RUNNING_LOW_POWER:
        {
            if (state == ARM_POWER_LOW)
            {
                /* Already running at low power! */
                proj_assert(false);
                return ARM_DRIVER_ERROR;
            }

            break;
        }
        case HW_TIMER_MANAGER_STATE_INITIALIZED:
        {
            if (state == ARM_POWER_OFF)
            {
                /* Already powered off! */
                proj_assert(false);
                return ARM_DRIVER_ERROR;
            }

            break;
        }
        default:
        {
            proj_assert(false);
            return ARM_DRIVER_ERROR;
        }
    }

    int32_t res = p_this->_p_conf->m->PowerControl(p_this, state);

    if (res == ARM_DRIVER_OK)
    {
        switch (state)
        {
            case ARM_POWER_FULL:
            {
                p_this->_state = HW_TIMER_MANAGER_STATE_RUNNING;
                break;
            }
            case ARM_POWER_LOW:
            {
                p_this->_state = HW_TIMER_MANAGER_STATE_RUNNING_LOW_POWER;
                break;
            }
            case ARM_POWER_OFF:
            {
                p_this->_state = HW_TIMER_MANAGER_STATE_INITIALIZED;
                break;
            }
            default:
            {
                proj_assert(false);
                return ARM_DRIVER_ERROR;
            }
        }
    }
    else if (res != ARM_DRIVER_ERROR_UNSUPPORTED)
    {
        p_this->_state = HW_TIMER_MANAGER_STATE_ERROR;
    }

    return res;
}

/***********************************************************************************************************************
 * Description: Returns the step time of the HW Timer's counter
 * Input      : p_this - pointer to self
 * Return     : frequency
 **********************************************************************************************************************/
extern float hw_timer_manager_get_step_time_ns(hw_timer_manager_t* p_this);

/***********************************************************************************************************************
 * Description: Returns the amount of steps in a single period
 * Input      : p_this - pointer to self
 * Return     : maximum value of counter
 **********************************************************************************************************************/
extern uint32_t hw_timer_manager_get_steps_per_period(hw_timer_manager_t* p_this);

/***********************************************************************************************************************
 * Description: Returns the current value of the HW Timer's counter
 * Input      : p_this - pointer to self
 * Return     : current value of counter
 **********************************************************************************************************************/
uint32_t hw_timer_manager_get_steps_current(hw_timer_manager_t* p_this)
{
    proj_assert(p_this != NULL);
    proj_assert(p_this->_p_conf != NULL);
    proj_assert(p_this->_p_conf->m != NULL);
    proj_assert(p_this->_p_conf->m->GetStepsCurrent != NULL);

    if (p_this->_state != HW_TIMER_MANAGER_STATE_RUNNING && p_this->_state != HW_TIMER_MANAGER_STATE_RUNNING_LOW_POWER)
    {
        proj_assert(false);
        return 0;
    }

    return p_this->_p_conf->m->GetStepsCurrent(p_this);
}

/***********************************************************************************************************************
 * Description: Returns whether there's a pending overflow IRQ.
 * Input      : p_interface - pointer to self
 * Return     : whether there's a pending overflow.
 **********************************************************************************************************************/
bool hw_timer_manager_is_overflow_pending(hw_timer_manager_t* p_this)
{
    proj_assert(p_this != NULL);
    proj_assert(p_this->_p_conf != NULL);
    proj_assert(p_this->_p_conf->m != NULL);
    proj_assert(p_this->_p_conf->m->IsOverflowPending != NULL);

    return p_this->_p_conf->m->IsOverflowPending(p_this);
}

/**********************************************************************************************************************/
/* Protected Function Definitions                                                                                     */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Calculates necessary division to achieve requested step time.
 * Input      : p_this        - pointer to self
 *              p_is_possible - return value - is the configured step time possible
 * Return     : source clock frequency division factor
 * Notes      : this function should be called *after* `p_this->._src_clk_freq` is set.
 **********************************************************************************************************************/
uint32_t hw_timer_manager_calc_prescaler_factor(hw_timer_manager_t* p_this, bool* p_is_possible)
{
    *p_is_possible = false;

    float requested_freq = NS_IN_S / p_this->_p_conf->step_time_ns;

    /* find closest frequency with an integer prescaler */
    uint32_t prescaler = round(p_this->_src_clk_freq / requested_freq);

    /* find the difference between the real frequency and the requested one */
    float freq_diff = ((float)p_this->_src_clk_freq / (float)prescaler) - requested_freq;

    if (fabs(freq_diff) > p_this->_p_conf->max_allowed_freq_shift)
    {
        proj_assert(false);
        return 0;
    }

    *p_is_possible = true;

    return prescaler;
}