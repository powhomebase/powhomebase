/***********************************************************************************************************************
 * Description: STM32L4 RTC driver implementation.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "rtc.h"

/* Related Project's headers */
#include <proj_assert.h>
#include <syscalls.h>
#include <time_units.h>
#include <time_utils.h>

/* Library headers */
#include <time.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#define RTC2TM_YEAR(year_)   (year_ + 100)
#define RTC2TM_MONTH(month_) (month_ - 1)
#define TM2RTC_YEAR(year_)   (year_ - 100)
#define TM2RTC_MONTH(month_) (month_ + 1)

/**********************************************************************************************************************/
/* Private Function Declarations                                                                                      */
/**********************************************************************************************************************/

static void convert_time_tm_to_hal(tm_t*            gen_time_struct,
                                   RTC_TimeTypeDef* rtc_time_struct,
                                   RTC_DateTypeDef* rtc_date_struct);
static void convert_hal_to_time_tm(RTC_TimeTypeDef* rtc_time_struct,
                                   RTC_DateTypeDef* rtc_date_struct,
                                   tm_t*            gen_time_struct);

/**********************************************************************************************************************/
/* Public Functions Implementation                                                                                    */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Initializes the driver
 * Input      : p_this - Pointer to self
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_initialize(rtc_resources_t* p_this)
{
    if (p_this->state == RTC_STATE_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    p_this->rtc_handler.Instance = p_this->p_conf->instance;

    p_this->rtc_handler.Init = *(p_this->p_conf->p_user_conf->init);

    /* Calculate prescales value from rtc's resolution (only if the values in configuration were set to 0). This is not
       optimal. For optimal value, set values in configuration */
    if (p_this->rtc_handler.Init.AsynchPrediv == 0 && p_this->rtc_handler.Init.SynchPrediv == 0)
    {
        uint32_t prescale_A = (uint32_t)(((double)(p_this->p_conf->src_clk_freq) / S_TO_NS(1))
                                             * (double)p_this->p_conf->p_user_conf->resolution_ns
                                         - 1);
        uint32_t prescale_S = 0;

        if (prescale_A > 127)
        {
            prescale_A = 127;
            prescale_S = (uint32_t)(((double)(p_this->p_conf->src_clk_freq) / S_TO_NS(1))
                                    * (double)p_this->p_conf->p_user_conf->resolution_ns / (prescale_A + 1))
                         - 1;
        }

        p_this->rtc_handler.Init.AsynchPrediv = prescale_A;
        p_this->rtc_handler.Init.SynchPrediv  = prescale_S;

        if (p_this->rtc_handler.Init.SynchPrediv > 0x7FFF)
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }

    /* Update state */
    p_this->state = RTC_STATE_INITIALIZED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Uninitializes the driver
 * Input      : p_this - Pointer to self
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_uninitialize(rtc_resources_t* p_this)
{
    ARGUMENT_UNUSED(p_this);
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description: Sets driver's power
 * Input      : p_this - Pointer to self
 *              state  - Power state to set
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_power_control(rtc_resources_t* p_this, ARM_POWER_STATE state)
{
    if (p_this->state == RTC_STATE_UNINITIALIZED)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (p_this->state == RTC_STATE_INITIALIZED)
            {
                return ARM_DRIVER_OK;
            }

            p_this->state = RTC_STATE_INITIALIZED;

            nvic_enable_irq(p_this->p_conf->alarm_irq, false);
            __HAL_RCC_RTC_DISABLE();

            return ARM_DRIVER_OK;
        }
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        case ARM_POWER_FULL:
        {
            if (p_this->state == RTC_STATE_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            __HAL_RCC_RTC_ENABLE();
            nvic_enable_irq(p_this->p_conf->alarm_irq, true);

            p_this->state = RTC_STATE_POWERED;

            return ARM_DRIVER_OK;
        }
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}

/***********************************************************************************************************************
 * Description: Sets the time to the RTC driver
 * Input      : p_this - Pointer to self
 *              time   - Time to set
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_set_time(rtc_resources_t* p_this, tm_t time)
{
    RTC_TimeTypeDef rtc_time_struct = {0};
    RTC_DateTypeDef rtc_date_struct = {0};

    /* The RTC can count only from the year 2000. So if time_date->tm_year (represent the number of years sence 1900)
       is smaller than 100, return error */
    if (time.tm_year < 100)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    convert_time_tm_to_hal(&time, &rtc_time_struct, &rtc_date_struct);

    if (HAL_RTC_SetTime(&p_this->rtc_handler, &rtc_time_struct, RTC_FORMAT_BIN) != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    if (HAL_RTC_SetDate(&p_this->rtc_handler, &rtc_date_struct, RTC_FORMAT_BIN) != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Gets the time from the RTC driver
 * Input      : p_this - Pointer to self
 *              p_time - Pointer to time struct (output parameter)
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_get_time(rtc_resources_t* p_this, tm_t* p_time)
{
    RTC_TimeTypeDef time_struct1 = {0}, time_struct2 = {0};
    RTC_DateTypeDef date_struct1 = {0}, date_struct2 = {0};

    /* Disable write protection in HAL as requested before wait for synchro */
    __HAL_RTC_WRITEPROTECTION_DISABLE(&p_this->rtc_handler);

    /* Use the hal function for busy wait on the shadow registers for syncronization when calling get_time_rtc right
     * after set_time_rtc. */
    HAL_StatusTypeDef hal_res = HAL_RTC_WaitForSynchro(&p_this->rtc_handler);

    /* Re-enable write protection in HAL after wait for synchro */
    __HAL_RTC_WRITEPROTECTION_ENABLE(&p_this->rtc_handler);

    if (HAL_OK != hal_res)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Read date & time register twice until two sequential reads are identicals - values might change as we read it so
     * we make sure all values are consistent with each other */
    do
    {
        if (HAL_RTC_GetTime(&p_this->rtc_handler, &time_struct1, RTC_FORMAT_BIN) != HAL_OK)
        {
            return ARM_DRIVER_ERROR;
        }
        if (HAL_RTC_GetDate(&p_this->rtc_handler, &date_struct1, RTC_FORMAT_BIN) != HAL_OK)
        {
            return ARM_DRIVER_ERROR;
        }
        if (HAL_RTC_GetTime(&p_this->rtc_handler, &time_struct2, RTC_FORMAT_BIN) != HAL_OK)
        {
            return ARM_DRIVER_ERROR;
        }
        if (HAL_RTC_GetDate(&p_this->rtc_handler, &date_struct2, RTC_FORMAT_BIN) != HAL_OK)
        {
            return ARM_DRIVER_ERROR;
        }
    } while ((date_struct1.Date != date_struct2.Date) || (time_struct1.Seconds != time_struct2.Seconds));

    convert_hal_to_time_tm(&time_struct1, &date_struct1, p_time);

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Sets alarm to the RTC driver
 * Input      : p_this - Pointer to self
 *              alarm  - Which alarm to set
 *              time   - Time of the alarm
 *              cb     - Callback function
 *              arg    - Callback argument
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_set_alarm(rtc_resources_t* p_this, arm_driver_rtc_alarm_e alarm, tm_t time, ARM_RTC_AlarmCb_t cb, void* arg)
{
    if (alarm >= DRIVER_RTC_NUM_ALARMS)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (cb == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Make sure alarm is later than now */
    tm_t current_time = {0};
    if (rtc_get_time(p_this, &current_time) != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    uint32_t current_timestamp = time_utils_mktime(&current_time);
    uint32_t alarm_timestamp   = time_utils_mktime(&time);

    if (alarm_timestamp <= current_timestamp)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    RTC_AlarmTypeDef sAlarm = {0};

    p_this->alarm_resources[alarm].callback = cb;
    p_this->alarm_resources[alarm].arg      = arg;

    uint32_t hal_alarm = (alarm == ARM_DRIVER_RTC_ALARM_0) ? RTC_ALARM_A : RTC_ALARM_B;

    sAlarm.Alarm                    = hal_alarm;
    sAlarm.AlarmDateWeekDay         = time.tm_mday;
    sAlarm.AlarmDateWeekDaySel      = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmMask                = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask       = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.Hours          = time.tm_hour;
    sAlarm.AlarmTime.Minutes        = time.tm_min;
    sAlarm.AlarmTime.SecondFraction = 0;
    sAlarm.AlarmTime.Seconds        = time.tm_sec;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmTime.SubSeconds     = 0;
    sAlarm.AlarmTime.TimeFormat     = RTC_HOURFORMAT12_AM;

    if (HAL_RTC_SetAlarm_IT(&p_this->rtc_handler, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Removes alarm of the RTC driver
 * Input      : p_this - Pointer to self
 *              alarm  - Which alarm to remove
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t rtc_remove_alarm(rtc_resources_t* p_this, arm_driver_rtc_alarm_e alarm)
{
    if (alarm >= DRIVER_RTC_NUM_ALARMS)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    uint32_t hal_alarm = (alarm == ARM_DRIVER_RTC_ALARM_0) ? RTC_ALARM_A : RTC_ALARM_B;

    if (HAL_RTC_DeactivateAlarm(&p_this->rtc_handler, hal_alarm) != HAL_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    p_this->alarm_resources[alarm].callback = NULL;
    p_this->alarm_resources[alarm].arg      = NULL;

    return ARM_DRIVER_OK;
}

/**********************************************************************************************************************/
/* HAL Callbacks                                                                                                      */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: HAL callback of RTC's alarm A
 * Input      : hrtc - Pointer to RTC handler
 **********************************************************************************************************************/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef* hrtc)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    rtc_resources_t*       p_this  = (rtc_resources_t*)hrtc;
    rtc_alarm_resources_t* p_alarm = &p_this->alarm_resources[ARM_DRIVER_RTC_ALARM_0];

    proj_assert(p_alarm->callback != NULL);

    /* Call the alarm handler */
    if (p_alarm->callback != NULL)
    {
        p_alarm->callback(p_alarm->arg);
    }
}

/***********************************************************************************************************************
 * Description: HAL callback of RTC's alarm B
 * Input      : hrtc - Pointer to RTC handler
 **********************************************************************************************************************/
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef* hrtc)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    rtc_resources_t*       p_this  = (rtc_resources_t*)hrtc;
    rtc_alarm_resources_t* p_alarm = &p_this->alarm_resources[ARM_DRIVER_RTC_ALARM_1];

    proj_assert(p_alarm->callback != NULL);

    /* Call the alarm handler */
    if (p_alarm->callback != NULL)
    {
        p_alarm->callback(p_alarm->arg);
    }
}

/**********************************************************************************************************************/
/* Private Functions Implementations                                                                                  */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Converts `struct tm` (from time.h) library to HAL's time struct
 * Input          : gen_time_struct - time.h date and time struct
 *                  rtc_time_struct - HAL time struct
 *                  rtc_date_struct - HAL date struct
 **********************************************************************************************************************/
static void convert_time_tm_to_hal(tm_t*            gen_time_struct,
                                   RTC_TimeTypeDef* rtc_time_struct,
                                   RTC_DateTypeDef* rtc_date_struct)
{
    rtc_time_struct->Hours   = gen_time_struct->tm_hour;
    rtc_time_struct->Minutes = gen_time_struct->tm_min;
    rtc_time_struct->Seconds = gen_time_struct->tm_sec;
    rtc_date_struct->Year    = TM2RTC_YEAR(gen_time_struct->tm_year);
    rtc_date_struct->Month   = TM2RTC_MONTH(gen_time_struct->tm_mon);
    rtc_date_struct->Date    = gen_time_struct->tm_mday;
    rtc_date_struct->WeekDay = RTC_WEEKDAY_MONDAY;
}

/***********************************************************************************************************************
 * Description    : Converts HAL's time struct to time.h `struct tm`
 * Input          : rtc_time_struct - HAL time struct
 *                  rtc_date_struct - HAL date struct
 *                  gen_time_struct - time.h date and time struct
 **********************************************************************************************************************/
static void convert_hal_to_time_tm(RTC_TimeTypeDef* rtc_time_struct,
                                   RTC_DateTypeDef* rtc_date_struct,
                                   tm_t*            gen_time_struct)
{
    gen_time_struct->tm_hour = rtc_time_struct->Hours;
    gen_time_struct->tm_min  = rtc_time_struct->Minutes;
    gen_time_struct->tm_sec  = rtc_time_struct->Seconds;
    gen_time_struct->tm_year = RTC2TM_YEAR(rtc_date_struct->Year);
    gen_time_struct->tm_mon  = RTC2TM_MONTH(rtc_date_struct->Month);
    gen_time_struct->tm_mday = rtc_date_struct->Date;
}
