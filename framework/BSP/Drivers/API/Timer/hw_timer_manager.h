/***********************************************************************************************************************
 * Description: HW Timer Manager API.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Libraries */
#include <Driver_Common.h>

/* Project related */
#include <assert.h>
#include <proj_assert.h>
#include <time_units.h>

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Generic HW Timer manager event callback */
typedef void (*hw_timer_event_cb_t)(void* arg, uint32_t event);

/* Defines bitmap shift of standard events - should cover _most_ of the needs, can be expanded to device specific events
   up to 32 events; the expansion should happen in the device specific implementation headers, and the bitmap should
   be requested by the generic implementation from the underlying hw abstraction layer */
typedef enum {
    HW_TIMER_MANAGER_EVENT_SHIFT_UPDATE,       /* Counter overflow/underflow */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC1,          /* Capture compare channel 1 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC2,          /* Capture compare channel 2 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC3,          /* Capture compare channel 3 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC4,          /* Capture compare channel 4 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC5,          /* Capture compare channel 5 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC6,          /* Capture compare channel 6 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC7,          /* Capture compare channel 7 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC8,          /* Capture compare channel 8 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC1_OVERFLOW, /* Capture compare channel overflow 1 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC2_OVERFLOW, /* Capture compare channel overflow 2 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC3_OVERFLOW, /* Capture compare channel overflow 3 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC4_OVERFLOW, /* Capture compare channel overflow 4 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC5_OVERFLOW, /* Capture compare channel overflow 5 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC6_OVERFLOW, /* Capture compare channel overflow 6 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC7_OVERFLOW, /* Capture compare channel overflow 7 */
    HW_TIMER_MANAGER_EVENT_SHIFT_CC8_OVERFLOW, /* Capture compare channel overflow 8 */

    HW_TIMER_MANAGER_EVENT_SHIFT_NUM,
} hw_timer_manager_event_shift_e;

static_assert(HW_TIMER_MANAGER_EVENT_SHIFT_NUM <= 32, "Can encode up to 32 events on 32 bits");

typedef enum {
    HW_TIMER_MANAGER_STATE_UNINITIALIZED,
    HW_TIMER_MANAGER_STATE_INITIALIZED,
    HW_TIMER_MANAGER_STATE_RUNNING_LOW_POWER,
    HW_TIMER_MANAGER_STATE_RUNNING,
    HW_TIMER_MANAGER_STATE_ERROR,
} hw_timer_manager_state_e;

/* Forward declaration */
typedef struct _hw_timer_manager hw_timer_manager_t;

typedef struct {
    /* Initialize the HW Timer Manager */
    int32_t (*Initialize)(hw_timer_manager_t* p_this, hw_timer_event_cb_t callback, void* arg);
    /* Uninitialize the HW Timer Manager */
    int32_t (*Uninitialize)(hw_timer_manager_t* p_this);
    /* Control power state of the HW Timer */
    int32_t (*PowerControl)(hw_timer_manager_t* p_this, ARM_POWER_STATE state);
    /* Get current counter value */
    uint32_t (*GetStepsCurrent)(hw_timer_manager_t* p_this);
    /* Returns whether there's a pending overflow IRQ */
    bool (*IsOverflowPending)(hw_timer_manager_t* p_this);
} const hw_timer_manager_methods_t;

typedef struct {
    hw_timer_manager_methods_t* m;
    float                       step_time_ns;
    uint32_t                    steps_per_period;
    float                       max_allowed_freq_shift;
} const hw_timer_manager_conf_t;

typedef struct _hw_timer_manager {
    hw_timer_manager_conf_t* const _p_conf;
    hw_timer_event_cb_t            __event_callback;
    void*                          __event_callback_arg;
    hw_timer_manager_state_e       _state;
    uint32_t                       _src_clk_freq;
} hw_timer_manager_t;

/**********************************************************************************************************************/
/* Interface Function Declarations                                                                                    */
/**********************************************************************************************************************/

/* Initialize the HW Timer Manager:
   - configure HW Timer according to const configuration
   - register event callback and callback argument */
int32_t hw_timer_manager_initialize(hw_timer_manager_t* p_this, hw_timer_event_cb_t callback, void* arg);

/* Uninitialize the HW Timer Manager */
int32_t hw_timer_manager_uninitialize(hw_timer_manager_t* p_this);

/* Control power state of the HW Timer */
int32_t hw_timer_manager_power_control(hw_timer_manager_t* p_this, ARM_POWER_STATE state);

/* Get step time of HW Timer's counter */
__INLINE float hw_timer_manager_get_step_time_ns(hw_timer_manager_t* p_this)
{
    return p_this->_p_conf->step_time_ns;
}

/* Get how many steps are in a single overflow period */
__INLINE uint32_t hw_timer_manager_get_steps_per_period(hw_timer_manager_t* p_this)
{
    return p_this->_p_conf->steps_per_period;
}

/* Get current counter value */
uint32_t hw_timer_manager_get_steps_current(hw_timer_manager_t* p_this);

/* Returns whether there's a pending overflow IRQ  */
bool hw_timer_manager_is_overflow_pending(hw_timer_manager_t* p_this);

/**********************************************************************************************************************/
/* Protected Function Declarations                                                                                    */
/**********************************************************************************************************************/

/* Calculate division factor necessary to achieve configured step time, returns if possible in `p_is_possible` */
uint32_t hw_timer_manager_calc_prescaler_factor(hw_timer_manager_t* p_this, bool* p_is_possible);
