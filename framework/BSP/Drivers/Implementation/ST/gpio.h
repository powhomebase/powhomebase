/***********************************************************************************************************************
 * Description : STM32 gpio driver header file.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Related Project's headers */
#include <Driver_GPIO.h>
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#define GPIO_GENERATE_OBJECT(gpio_name, port, pin)                                                                     \
    gpio_pin_obj_t GPIO_##gpio_name = {                                                                                \
        .gpio_port = port,                                                                                             \
        .pin_num   = pin,                                                                                              \
    }

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* GPIO Pin struct */
typedef struct gpio_pin_obj {
    GPIO_TypeDef *     gpio_port;                /* GPIO Port */
    uint16_t           pin_num;                  /* GPIO Pin*/
    uint32_t           open_mode;                /* GPIO open mode */
    uint32_t           close_mode;               /* GPIO close mode */
    uint8_t            open_alternate_function;  /* GPIO open mode alternate function */
    uint8_t            close_alternate_function; /* GPIO close mode alternate function */
    uint32_t           open_pullup : 2;          /* GPIO open mode Pull-up */
    uint32_t           close_pullup : 2;         /* GPIO close mode Pull-up */
    uint32_t           open_speed : 2;           /* GPIO open speed */
    uint32_t           close_speed : 2;          /* GPIO close speed */
    unsigned           is_open : 1;              /* GPIO open flag */
    ARM_GPIO_PIN_LEVEL open_out : 1;             /* GPIO Open state output */
    ARM_GPIO_PIN_LEVEL close_out : 1;            /* GPIO close state output */
} gpio_pin_obj_t;

typedef struct {
    bool     exti15_10_irq_enable;
    uint32_t exti15_10_irq_priority;
    bool     exti9_5_irq_enable;
    uint32_t exti9_5_irq_priority;
    bool     exti4_irq_enable;
    uint32_t exti4_irq_priority;
    bool     exti3_irq_enable;
    uint32_t exti3_irq_priority;
    bool     exti2_irq_enable;
    uint32_t exti2_irq_priority;
    bool     exti1_irq_enable;
    uint32_t exti1_irq_priority;
    bool     exti0_irq_enable;
    uint32_t exti0_irq_priority;
} gpio_user_conf_t;