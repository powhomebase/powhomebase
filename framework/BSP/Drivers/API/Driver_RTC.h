#ifndef __DRIVER_RTC_H
#define __DRIVER_RTC_H

#include <Driver_Common.h>
/* Required for the `tm` struct */
#include <time.h>

typedef void (*ARM_RTC_AlarmCb_t)(void* arg);  ///< Pointer to \ref ARM_RTC_AlarmCb_t : RTC Alarm Callback.
typedef struct tm tm_t;

// No peripherals with more than two alarms are known yet. Should extend if exists in the future.
typedef enum {
    ARM_DRIVER_RTC_ALARM_0,
    ARM_DRIVER_RTC_ALARM_1,

    ARM_DRIVER_RTC_ALARM_NUM,
} arm_driver_rtc_alarm_e;

// Function documentation
/**
  \fn          int32_t ARM_RTC_Initialize (void)
  \brief       Initialize RTC Interface.
  \return      \ref execution_status

  \fn          int32_t ARM_RTC_Uninitialize (void)
  \brief       De-initialize RTC Interface.
  \return      \ref execution_status

  \fn          int32_t ARM_RTC_PowerControl (ARM_POWER_STATE state)
  \brief       Control RTC Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status

  \fn          int32_t ARM_RTC_SetTime (tm_t time)
  \brief       Set RTC current date and time
  \param[in]   time  Time to set
  \return      \ref execution_status

  \fn          int32_t ARM_RTC_GetTime (tm_t *p_time)
  \brief       Get RTC current date and time
  \param[out]  p_time  Current time
  \return      \ref execution_status

  \fn          int32_t ARM_RTC_SetAlarm (tm_t time, arm_driver_rtc_alarm_e alarm, ARM_RTC_AlarmCb_t cb, void *arg)
  \brief       Set RTC alarm
  \param[in]   alarm Which alarm to set
  \param[in]   time  Alarm time
  \param[in]   cb    Alarm callback
  \param[in]   arg   Alarm argument
  \return      \ref execution_status

  \fn          int32_t ARM_RTC_RemoveAlarm (arm_driver_rtc_alarm_e alarm)
  \brief       Remove RTC alarm
  \param[in]   alarm Which alarm to set
  \return      \ref execution_status
*/

/**
\brief Access structure of the RTC Driver.
*/
typedef struct _ARM_DRIVER_RTC {
    int32_t (*Initialize)(void);
    int32_t (*Uninitialize)(void);
    int32_t (*PowerControl)(ARM_POWER_STATE state);
    int32_t (*SetTime)(tm_t time);
    int32_t (*GetTime)(tm_t* p_time);
    int32_t (*SetAlarm)(tm_t time, arm_driver_rtc_alarm_e alarm, ARM_RTC_AlarmCb_t cb, void* arg);
    int32_t (*RemoveAlarm)(arm_driver_rtc_alarm_e alarm);
} const ARM_DRIVER_RTC;

#endif /* __DRIVER_RTC_H */