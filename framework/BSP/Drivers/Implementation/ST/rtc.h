/***********************************************************************************************************************
 * Description: STM32L4 RTC driver header file.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Related Project's headers */
#include <Driver_RTC.h>
#include <global.h>

/* Libraries */
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#define HSE_DIV32_VALUE       (HSE_VALUE / 32)
#define DRIVER_RTC_NUM_ALARMS 2 /* L4 supports alarms A&B */

/***********************************************************************************************************************
 * Description    : Generates a RTC driver object.
 * Input          : p_user_conf_ - Pointer to user configuration.
 *                  clk_src_     - Choose the RTC source clock from the list:
 *                                     LSI  - low speed internal clk
 *                                     HSE_DIV32  - high speed external clk divided by 32
 *                                     LSE  - low speed external clk
 ***********************************************************************************************************************/
#define RTC_GENERATE_OBJECT(p_user_conf_, clk_src_)                                                                    \
    extern rtc_resources_t RTC_RESOURCES;                                                                              \
                                                                                                                       \
    static int32_t RTC_Initialize(void)                                                                                \
    {                                                                                                                  \
        return rtc_initialize(&RTC_RESOURCES);                                                                         \
    }                                                                                                                  \
    static int32_t RTC_Uninitialize(void)                                                                              \
    {                                                                                                                  \
        return rtc_uninitialize(&RTC_RESOURCES);                                                                       \
    }                                                                                                                  \
    static int32_t RTC_PowerControl(ARM_POWER_STATE state)                                                             \
    {                                                                                                                  \
        return rtc_power_control(&RTC_RESOURCES, state);                                                               \
    }                                                                                                                  \
    static int32_t RTC_SetTime(tm_t time)                                                                              \
    {                                                                                                                  \
        return rtc_set_time(&RTC_RESOURCES, time);                                                                     \
    }                                                                                                                  \
    static int32_t RTC_GetTime(tm_t* p_time)                                                                           \
    {                                                                                                                  \
        return rtc_get_time(&RTC_RESOURCES, p_time);                                                                   \
    }                                                                                                                  \
    static int32_t RTC_SetAlarm(tm_t time, arm_driver_rtc_alarm_e alarm, ARM_RTC_AlarmCb_t cb, void* arg)              \
    {                                                                                                                  \
        return rtc_set_alarm(&RTC_RESOURCES, alarm, time, cb, arg);                                                    \
    }                                                                                                                  \
    static int32_t RTC_RemoveAlarm(arm_driver_rtc_alarm_e alarm)                                                       \
    {                                                                                                                  \
        return rtc_remove_alarm(&RTC_RESOURCES, alarm);                                                                \
    }                                                                                                                  \
                                                                                                                       \
    RTC_IRQ_HANDLER();                                                                                                 \
                                                                                                                       \
    ARM_DRIVER_RTC Driver_RTC = {                                                                                      \
        .Initialize   = RTC_Initialize,                                                                                \
        .Uninitialize = RTC_Uninitialize,                                                                              \
        .PowerControl = RTC_PowerControl,                                                                              \
        .GetTime      = RTC_GetTime,                                                                                   \
        .SetTime      = RTC_SetTime,                                                                                   \
        .SetAlarm     = RTC_SetAlarm,                                                                                  \
        .RemoveAlarm  = RTC_RemoveAlarm,                                                                               \
    };                                                                                                                 \
                                                                                                                       \
    static rtc_conf_t rtc_conf = {                                                                                     \
        .p_user_conf  = p_user_conf_,                                                                                  \
        .instance     = RTC,                                                                                           \
        .alarm_irq    = RTC_Alarm_IRQn,                                                                                \
        .src_clk_freq = clk_src_##_VALUE,                                                                              \
    };                                                                                                                 \
                                                                                                                       \
    rtc_resources_t RTC_RESOURCES = {                                                                                  \
        .p_conf = &rtc_conf,                                                                                           \
        .state  = RTC_STATE_UNINITIALIZED,                                                                             \
    };

#if defined(STM32L4)
#define RTC_IRQ_HANDLER()                                                                                              \
    void RTC_Alarm_IRQHandler(void)                                                                                    \
    {                                                                                                                  \
        HAL_RTC_AlarmIRQHandler(&RTC_RESOURCES.rtc_handler);                                                           \
    }   
#elif defined(STM32U5)
#define RTC_IRQ_HANDLER()                                                                                              \
    void RTC_IRQHandler(void)                                                                                          \
    {                                                                                                                  \
        HAL_RTC_AlarmIRQHandler(&RTC_RESOURCES.rtc_handler);                                                           \
    }   
#endif
/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef enum {
    RTC_STATE_UNINITIALIZED,
    RTC_STATE_INITIALIZED,
    RTC_STATE_POWERED,
} rtc_state_t;

typedef struct {
    RTC_InitTypeDef* init;
    uint64_t         resolution_ns;
} const rtc_user_conf_t;

typedef struct {
    rtc_user_conf_t* p_user_conf;
    RTC_TypeDef*     instance;
    IRQn_Type        alarm_irq;
    uint32_t         src_clk_freq;
} const rtc_conf_t;

typedef struct {
    ARM_RTC_AlarmCb_t callback;
    void*             arg;
} rtc_alarm_resources_t;

typedef struct {
    /* ST RTC handler struct */
    /* this must be the struct itself (and not a pointer) so we could upcast the handle pointer to the resource pointer
     * in the implementation of the weak function */
    RTC_HandleTypeDef     rtc_handler;
    rtc_conf_t*           p_conf;
    rtc_alarm_resources_t alarm_resources[DRIVER_RTC_NUM_ALARMS];
    rtc_state_t           state;
} rtc_resources_t;

/**********************************************************************************************************************/
/* Function Declarations                                                                                              */
/**********************************************************************************************************************/

int32_t rtc_initialize(rtc_resources_t* p_this);
int32_t rtc_uninitialize(rtc_resources_t* p_this);
int32_t rtc_power_control(rtc_resources_t* p_this, ARM_POWER_STATE state);
int32_t rtc_set_time(rtc_resources_t* p_this, tm_t time);
int32_t rtc_get_time(rtc_resources_t* p_this, tm_t* p_time);
int32_t rtc_set_alarm(rtc_resources_t*       p_this,
                      arm_driver_rtc_alarm_e alarm,
                      tm_t                   time,
                      ARM_RTC_AlarmCb_t      cb,
                      void*                  arg);
int32_t rtc_remove_alarm(rtc_resources_t* p_this, arm_driver_rtc_alarm_e alarm);
