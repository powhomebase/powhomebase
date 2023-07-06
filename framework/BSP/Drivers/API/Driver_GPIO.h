/*
 * Example of GPIO driver interface
 */

#ifndef __DRIVER_GPIO_H
#define __DRIVER_GPIO_H

#include "Driver_Common.h"

#define ARM_GPIO_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 00) /* API version */

#define ARM_GPIO_CONF_MAKE(open, sleep) ((open) | ((sleep) << ARM_GPIO_CONF_END_POS))

#define ARM_GPIO_AF_MAKE(open, sleep) ((open) | ((sleep) << ARM_GPIO_AF_END_POS))

#define ARM_GPIO_PERIPHERAL_CONTROL(control, arg) Driver_GPIO.Control(NULL, control, arg)

/* Defines a disconnected pin, used when we would like
   to preserve the logic in SW but we used HW to control
   the pin. for example constant power supply for
   periphrial vs SW controlled, or an unused interrupt
   line. */
#define GPIO_NOT_CONNECTED ((ARM_GPIO_PIN *)(0xffffffff))

/****** GPIO Commands *****/
#define ARM_GPIO_CONFIGURE_STATE (0x00)
#define ARM_GPIO_SET_STATE       (0x01)
#define ARM_GPIO_SET_INT_TRIGGER (0x02)
#define ARM_GPIO_SET_INT_HANDLER (0x03)
#define ARM_GPIO_SET_AF          (0x04)
#define ARM_GPIO_SET_WAKEUP      (0x05)
#define ARM_GPIO_ENABLE_WAKEUP   (0x06)
#define ARM_GPIO_CHECK_WAKEUP    (0x07)
#define ARM_GPIO_SET_SWD         (0x08)
#define ARM_GPIO_SET_IRQ         (0x09)

/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_STATE *****/
#define ARM_GPIO_MOD_POS   (0U)
#define ARM_GPIO_MOD_MASK  (07U << ARM_GPIO_MOD_POS)
#define ARM_GPIO_DISABLED  (00U << ARM_GPIO_MOD_POS)
#define ARM_GPIO_INPUT     (01U << ARM_GPIO_MOD_POS)
#define ARM_GPIO_OUTPUT    (02U << ARM_GPIO_MOD_POS)
#define ARM_GPIO_AF_INPUT  (03U << ARM_GPIO_MOD_POS)
#define ARM_GPIO_AF_OUTPUT (04U << ARM_GPIO_MOD_POS)
#define ARM_GPIO_ANALOG    (05U << ARM_GPIO_MOD_POS)
/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_OUTPUT_TYPE *****/
#define ARM_GPIO_OUTPUT_TYPE_POS  (3U)
#define ARM_GPIO_OUTPUT_TYPE_MASK (03U << ARM_GPIO_OUTPUT_TYPE_POS)
#define ARM_GPIO_OUTPUT_PUSH_PULL (00U << ARM_GPIO_OUTPUT_TYPE_POS)
#define ARM_GPIO_OUTPUT_WIRED_OR  (01U << ARM_GPIO_OUTPUT_TYPE_POS)
#define ARM_GPIO_OUTPUT_WIRED_AND (02U << ARM_GPIO_OUTPUT_TYPE_POS)
/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_PULLUP_PULLDOWN *****/
#define ARM_GPIO_PU_PD_POS     (5U)
#define ARM_GPIO_PU_PD_MASK    (03U << ARM_GPIO_PU_PD_POS)
#define ARM_GPIO_PULL_DISABLED (00U << ARM_GPIO_PU_PD_POS)
#define ARM_GPIO_PULL_UP       (01U << ARM_GPIO_PU_PD_POS)
#define ARM_GPIO_PULL_DOWN     (02U << ARM_GPIO_PU_PD_POS)
#define ARM_GPIO_PULL_DOUT     (03U << ARM_GPIO_PU_PD_POS)
/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_DRIVE *****/
#define ARM_GPIO_DRIVE_POS      (7U)
#define ARM_GPIO_DRIVE_MASK     (01U << ARM_GPIO_DRIVE_POS)
#define ARM_GPIO_DRIVE_DISABLED (00U << ARM_GPIO_DRIVE_POS)
#define ARM_GPIO_DRIVE_ENABLED  (01U << ARM_GPIO_DRIVE_POS)
/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_SPEED *****/
#define ARM_GPIO_SPEED_POS    (8U)
#define ARM_GPIO_SPEED_MASK   (03U << ARM_GPIO_SPEED_POS)
#define ARM_GPIO_SPEED_NORMAL (00U << ARM_GPIO_SPEED_POS)
#define ARM_GPIO_SPEED_SLOW   (01U << ARM_GPIO_SPEED_POS)
#define ARM_GPIO_SPEED_HIGH   (02U << ARM_GPIO_SPEED_POS)
/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_FILTER *****/
#define ARM_GPIO_FILTER_POS      (10U)
#define ARM_GPIO_FILTER_MASK     (01U << ARM_GPIO_FILTER_POS)
#define ARM_GPIO_FILTER_DISABLED (00U << ARM_GPIO_FILTER_POS)
#define ARM_GPIO_FILTER_ENABLED  (01U << ARM_GPIO_FILTER_POS)
/****** GPIO Control Codes: ARM_GPIO_CONFIG_STATE: GPIO_DOUT (Default out) *****/
#define ARM_GPIO_DOUT_POS  (11U)
#define ARM_GPIO_DOUT_MASK (01U << ARM_GPIO_DOUT_POS)
#define ARM_GPIO_DOUT_LOW  (00U << ARM_GPIO_DOUT_POS)
#define ARM_GPIO_DOUT_HIGH (01U << ARM_GPIO_DOUT_POS)

#define ARM_GPIO_CONF_END_POS    (12U)
#define ARM_GPIO_CONF_OPEN_MASK  ((1 << ARM_GPIO_CONF_END_POS) - 1)
#define ARM_GPIO_CONF_CLOSE_MASK (ARM_GPIO_CONF_OPEN_MASK << ARM_GPIO_CONF_END_POS)

/****** GPIO Arg Codes: ARM_GPIO_SET_INT_TRIGGER: GPIO_TRIGGER *****/
#define ARM_GPIO_INT_TRIGGER_NONE    (00U)
#define ARM_GPIO_INT_TRIGGER_RISING  (01U)
#define ARM_GPIO_INT_TRIGGER_FALLING (02U)
#define ARM_GPIO_INT_TRIGGER_BOTH    (03U)

/****** GPIO Arg Codes: ARM_GPIO_SET_AF *****/
#define ARM_GPIO_AF_END_POS    (8U)
#define ARM_GPIO_AF_OPEN_MASK  ((1 << ARM_GPIO_AF_END_POS) - 1)
#define ARM_GPIO_AF_CLOSE_MASK (ARM_GPIO_AF_OPEN_MASK << ARM_GPIO_AF_END_POS)


/****** GPIO Control Codes: ARM_GPIO_SET_WAKEUP : POLARITY *****/
#define ARM_GPIO_WAKEUP_POLARITY_POS  (0U)
#define ARM_GPIO_WAKEUP_POLARITY_MASK (01U << ARM_GPIO_WAKEUP_POLARITY_POS)
#define ARM_GPIO_WAKEUP_POLARITY_LOW  (00U << ARM_GPIO_WAKEUP_POLARITY_POS)
#define ARM_GPIO_WAKEUP_POLARITY_HIGH (01U << ARM_GPIO_WAKEUP_POLARITY_POS)

/****** GPIO Control Codes: ARM_GPIO_SET_WAKEUP : ENABLE *****/
#define ARM_GPIO_WAKEUP_ENABLE_POS  (1U)
#define ARM_GPIO_WAKEUP_ENABLE_MASK (01U << ARM_GPIO_WAKEUP_ENABLE_POS)
#define ARM_GPIO_WAKEUP_DISABLE     (00U << ARM_GPIO_WAKEUP_ENABLE_POS)
#define ARM_GPIO_WAKEUP_ENABLE      (01U << ARM_GPIO_WAKEUP_ENABLE_POS)

/****** GPIO Arg Codes: ARM_GPIO_ENABLE_WAKEUP: RETENTION *****/
#define RETENTION    (00U)
#define NO_RETENTION (01U)

/****** GPIO Arg Codes: ARM_GPIO_ENABLE_SWD                *****/
#define DISABLE_SWD (00U)
#define ENABLE_SWD  (01U)

/**
\brief GPIO Status
*/
typedef struct _ARM_GPIO_STATUS {
    uint32_t pins_open : 8;  ///< Running flag
} ARM_GPIO_STATUS;

/**
\brief GPIO pin
*/
#ifdef TEST
typedef uint32_t ARM_GPIO_PIN;
#else
typedef struct gpio_pin_obj ARM_GPIO_PIN;
#endif

/**
\brief GPIO interrupt handler
*/
typedef void (*ARM_GPIO_Interrupt_t)(void);

/**
\brief GPIO pin states
*/
typedef enum _ARM_GPIO_PIN_LEVEL {
    LOW  = 0x00,
    HIGH = 0x01,
} ARM_GPIO_PIN_LEVEL;

typedef enum _ARM_GPIO_PIN_STATE {
    OPEN  = 0x00,
    CLOSE = 0x01,
} ARM_GPIO_PIN_STATE;


// Function documentation
/**
  \fn          int32_t ARM_GPIO_Initialize (void)
  \brief       Initialize GPIO Interface.
  \param[in]   cb_event  Pointer to \ref ARM_GPIO_SignalEvent
  \return      \ref execution_status

  \fn          int32_t ARM_GPIO_Uninitialize (void)
  \brief       De-initialize GPIO Interface.
  \return      \ref execution_status

  \fn          int32_t ARM_GPIO_PowerControl (ARM_POWER_STATE state)
  \brief       Control GPIO Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status

  \fn          int32_t ARM_GPIO_Write (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
  \brief       Start transmitting data as GPIO Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[in]   data          Pointer to buffer with data to transmit to GPIO Slave
  \param[in]   num           Number of data bytes to transmit
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      \ref execution_status

  \fn          int32_t ARM_GPIO_Read (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
  \brief       Start receiving data as GPIO Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[out]  data          Pointer to buffer for data to receive from GPIO Slave
  \param[in]   num           Number of data bytes to receive
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      \ref execution_status

  \fn          int32_t ARM_GPIO_Control (uint32_t control, uint32_t arg)
  \brief       Control GPIO Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      \ref execution_status

  \fn          ARM_GPIO_STATUS ARM_GPIO_GetStatus (void)
  \brief       Get GPIO status.
  \return      GPIO status \ref ARM_GPIO_STATUS
*/

/**
\brief GPIO Driver Capabilities.
*/
typedef struct _ARM_GPIO_CAPABILITIES {
    uint32_t speed_support : 1;
} ARM_GPIO_CAPABILITIES;

/* Initialize GPIO */
int32_t gpio_initialize(void);

/* Uninitialize GPIO */
int32_t gpio_uninitialize(void);

/* Power control of GPIO */
int32_t gpio_power_control(ARM_POWER_STATE state);

/* Set GPIO pin to high\low */
int32_t gpio_write(ARM_GPIO_PIN *pin, ARM_GPIO_PIN_LEVEL state);

/* Get GPIO pin state */
ARM_GPIO_PIN_LEVEL gpio_read(ARM_GPIO_PIN *pin);

/* GPIO control function */
int32_t gpio_control(ARM_GPIO_PIN *pin, uint32_t control, uint32_t arg);

/* Get GPIO status */
ARM_GPIO_STATUS gpio_get_status(void);

typedef struct {
    uint32_t conf;
    uint32_t alt_fun;
} ARM_GPIO_PIN_CONF;

#endif /* __DRIVER_GPIO_H */
