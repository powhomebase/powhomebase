/***********************************************************************************************************************
 * Description: STM32 gpio driver implementation.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "gpio.h"

/* Project related */
#include <proj_assert.h>
#include <syscalls.h>
#if defined(STM32F1)
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_ll_exti.h>
#elif defined(STM32L4)
#include <stm32l4xx_hal_gpio.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_ll_exti.h>
#elif defined(STM32F4)
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_ll_exti.h>
#elif defined(STM32WL)
#include <stm32wlxx_hal_gpio.h>
#include <stm32wlxx_hal_rcc.h>
#include <stm32wlxx_ll_exti.h>
#endif

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#if !defined(STM32F1)
#define _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(port_) __HAL_RCC_GPIO##port_##_CLK_SLEEP_DISABLE()
#define _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(port_)  __HAL_RCC_GPIO##port_##_CLK_SLEEP_ENABLE()
#else
/* No support in STM32F1 */
#define _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(port_) {}
#define _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(port_) {}
#endif /* STM32F1 */

/* Define the number of pins per port. */
#define PINS_PER_PORT                  (16)
#define HIGHEST_PREEMPT_PRIORITY_VALUE (0x10)

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* gpio irq conf, defined in the BSP objs file */
extern gpio_user_conf_t gpio_conf;

static ARM_GPIO_STATUS DriverStatus = {
    .pins_open = 0, /* All pins are closed. */
};

/* The GPIO handlers that will be triggered on each pin number interrupt. */
static ARM_GPIO_Interrupt_t gpio_handlers[PINS_PER_PORT];

/* ---------------------------------------------------- SWD pins ---------------------------------------------------- */
#if defined(CONFIG_DISABLE_SWD_PINS)
extern ARM_GPIO_PIN GPIO_SWD_IO;
extern ARM_GPIO_PIN GPIO_SWD_CLK;
#endif

/**********************************************************************************************************************/
/* Private Function Declarations                                                                                      */
/**********************************************************************************************************************/

#if defined(CONFIG_DISABLE_SWD_PINS)
/* Control SWD pins */
static int32_t gpio_set_swd(bool enable);
#endif

/* Convert 1-hot gpio index to position */
__STATIC_INLINE uint8_t get_gpio_pos(uint16_t pin_num);

/* GPIO pin set or clear pending IRQ */
__STATIC_INLINE int32_t gpio_set_irq(ARM_GPIO_PIN *pin, bool enable);

/* Clear all pending interrupts */
static void gpio_nvic_clear_irq(void);

/* Enable/Disable irqs in the NVIC */
static void gpio_nvic_enable_irq(bool enable);

/***********************************************************************************************************************
 * Description : Initialize GPIO module
 * Return      : execution code
 **********************************************************************************************************************/
int32_t gpio_initialize(void)
{
    if (gpio_conf.exti15_10_irq_enable)
    {
        proj_assert(gpio_conf.exti15_10_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI15_10_IRQn, gpio_conf.exti15_10_irq_priority);
    }

    if (gpio_conf.exti9_5_irq_enable)
    {
        proj_assert(gpio_conf.exti9_5_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI9_5_IRQn, gpio_conf.exti9_5_irq_priority);
    }

    if (gpio_conf.exti4_irq_enable)
    {
        proj_assert(gpio_conf.exti4_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI4_IRQn, gpio_conf.exti4_irq_priority);
    }

    if (gpio_conf.exti3_irq_enable)
    {
        proj_assert(gpio_conf.exti3_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI3_IRQn, gpio_conf.exti3_irq_priority);
    }

    if (gpio_conf.exti2_irq_enable)
    {
        proj_assert(gpio_conf.exti2_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI2_IRQn, gpio_conf.exti2_irq_priority);
    }

    if (gpio_conf.exti1_irq_enable)
    {
        proj_assert(gpio_conf.exti1_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI1_IRQn, gpio_conf.exti1_irq_priority);
    }

    if (gpio_conf.exti0_irq_enable)
    {
        proj_assert(gpio_conf.exti0_irq_priority < HIGHEST_PREEMPT_PRIORITY_VALUE);
        nvic_set_priority(EXTI0_IRQn, gpio_conf.exti0_irq_priority);
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : De-initialize GPIO module
 * Return      : execution code
 ************************************************************************************************************************/
int32_t gpio_uninitialize(void)
{
    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : Control GPIO module power
 * Input       : state - power state
 * Return      : execution status
 ************************************************************************************************************************/
int32_t gpio_power_control(ARM_POWER_STATE state)
{
    switch (state)
    {
        /* Turn off GPIO module */
        case ARM_POWER_OFF:
        {
            /* Disable clock for GPIO module. */
#if defined(GPIOA)
            __HAL_RCC_GPIOA_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(A);
#endif /* GIPOA */

#if defined(GPIOB)
            __HAL_RCC_GPIOB_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(B);
#endif /* GIPOB */

#if defined(GPIOC)
            __HAL_RCC_GPIOC_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(C);
#endif /* GIPOC */

#if defined(GPIOD)
            __HAL_RCC_GPIOD_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(D);
#endif /* GIPOD */

#if defined(GPIOE)
            __HAL_RCC_GPIOE_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(E);
#endif /* GIPOE*/

#if defined(GPIOF)
            __HAL_RCC_GPIOF_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(F);
#endif /* GPIOF */

#if defined(GPIOG)
            __HAL_RCC_GPIOG_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(G);
#endif /* GPIOG */

#if defined(GPIOH)
            __HAL_RCC_GPIOH_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(H);
#endif /* GPIOH */

#if defined(GPIOI)
            __HAL_RCC_GPIOI_CLK_DISABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_DISABLE(I);
#endif /* GPIOI */

#if defined(PWR_CR2_IOSV)
            /* Disable vddio2 supply */
            HAL_PWREx_DisableVddIO2();
#endif /* PWR_CR2_IOSV */

#if defined(STM32F1)
            __HAL_RCC_AFIO_CLK_DISABLE();
#endif /* STM32F1 */

            gpio_nvic_enable_irq(false);
            gpio_nvic_clear_irq();

            return ARM_DRIVER_OK;
        }
        /* Low power mode - NOT SUPPORTED */
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        /* Turn on GPIO module */
        case ARM_POWER_FULL:
        {
            /* Enable clock for GPIO module. */
#if defined(GPIOA)
            __HAL_RCC_GPIOA_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(A);
#endif /* GPIOA */

#if defined(GPIOB)
            __HAL_RCC_GPIOB_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(B);
#endif /* GPIOB */

#if defined(GPIOC)
            __HAL_RCC_GPIOC_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(C);
#endif /* GPIOC */

#if defined(GPIOD)
            __HAL_RCC_GPIOD_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(D);
#endif /* GPIOD */

#if defined(GPIOE)
            __HAL_RCC_GPIOE_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(E);
#endif /* GPIOE */

#if defined(GPIOF)
            __HAL_RCC_GPIOF_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(F);
#endif /* GPIOAF*/

#if defined(GPIOG)
            __HAL_RCC_GPIOG_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(G);
#endif /* GPIOG */

#if defined(GPIOH)
            __HAL_RCC_GPIOH_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(H);
#endif /* GPIOH */

#if defined(GPIOI)
            __HAL_RCC_GPIOI_CLK_ENABLE();
            _HAL_RCC_GPIO_CLK_SLEEP_ENABLE(I);
#endif /* GPIOI */

#if defined(PWR_CR2_IOSV)
            /* Enable vddio2 supply for GPIO_PG[2:15] */
            HAL_PWREx_EnableVddIO2();
#endif /* PWR_CR2_IOSV */

#if defined(STM32F1)
            __HAL_RCC_AFIO_CLK_ENABLE();
#endif /* STM32F1 */

            gpio_nvic_clear_irq();
            gpio_nvic_enable_irq(true);

            return ARM_DRIVER_OK;
        }
        /* Invalid power state */
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}

/***********************************************************************************************************************
* Description : Set GPIO pin level
* Input       : pin - Pointer to GPIO pin
                lvl - Level to set pin to
* Return      : execution status
***********************************************************************************************************************/
int32_t gpio_write(ARM_GPIO_PIN *pin, ARM_GPIO_PIN_LEVEL lvl)
{
    /* Make sure the given pin is not null */
    if (pin == NULL)
    {
        proj_assert(false);

        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Make sure the given pin is connected */
    if (pin == GPIO_NOT_CONNECTED)
    {
        return ARM_DRIVER_OK;
    }

    /* Check to see what level the pin should be set to */
    switch (lvl)
    {
        /* Set GPIO pin to HIGH */
        case HIGH:
        {
            /* Write HIGH to the given pin */
            HAL_GPIO_WritePin(pin->gpio_port, pin->pin_num, GPIO_PIN_SET);
            break;
        }
        /* Set GPIO pin to LOW */
        case LOW:
        {
            /* Write LOW to the given pin */
            HAL_GPIO_WritePin(pin->gpio_port, pin->pin_num, GPIO_PIN_RESET);
            break;
        }
        /* Invalid pin state given*/
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description : Read GPIO pin level
 * Input       : pin - Pointer to GPIO pin
 * Return      : the given pin's level
 ************************************************************************************************************************/
ARM_GPIO_PIN_LEVEL gpio_read(ARM_GPIO_PIN *pin)
{
    proj_assert(pin != NULL);

    /* Return LOW in case the GPIO is not connected or NULL */
    if (pin == NULL || pin == GPIO_NOT_CONNECTED)
    {
        return LOW;
    }

    return (ARM_GPIO_PIN_LEVEL)HAL_GPIO_ReadPin(pin->gpio_port, pin->pin_num);
}

/***********************************************************************************************************************
* Description : Control GPIO pin
* Input       : pin     - Pointer to GPIO pin
                control - Operation
                arg     - Argument of operation (optional)
* Return      : execution status
***********************************************************************************************************************/
int32_t gpio_control(ARM_GPIO_PIN *pin, uint32_t control, uint32_t arg)
{
    /* Check if pin isn't null */
    if (pin == NULL)
    {
        proj_assert(false);

        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Make sure the pin given is connected */
    if (pin == GPIO_NOT_CONNECTED)
    {
        return ARM_DRIVER_OK;
    }

    switch (control)
    {
        case ARM_GPIO_CONFIGURE_STATE:
        {
            uint8_t  temp       = 0;
            uint32_t open_mods  = arg & ARM_GPIO_CONF_OPEN_MASK;
            uint32_t close_mods = (arg & ARM_GPIO_CONF_CLOSE_MASK) >> ARM_GPIO_CONF_END_POS;

            /* Set the default output level */
            pin->open_out  = (ARM_GPIO_PIN_LEVEL)((open_mods & ARM_GPIO_DOUT_MASK) == ARM_GPIO_DOUT_HIGH);
            pin->close_out = (ARM_GPIO_PIN_LEVEL)((close_mods & ARM_GPIO_DOUT_MASK) == ARM_GPIO_DOUT_HIGH);

            open_mods &= ~ARM_GPIO_DOUT_MASK;
            close_mods &= ~ARM_GPIO_DOUT_MASK;

            /* Set GPIO pin open and close pull-up states */
            pin->open_pullup  = (open_mods & ARM_GPIO_PU_PD_MASK) >> ARM_GPIO_PU_PD_POS;
            pin->close_pullup = (close_mods & ARM_GPIO_PU_PD_MASK) >> ARM_GPIO_PU_PD_POS;

            /* Set the open mode output speed */
            switch (open_mods & ARM_GPIO_SPEED_MASK)
            {
                case ARM_GPIO_SPEED_SLOW:
                {
                    pin->open_speed = GPIO_SPEED_FREQ_LOW;
                    break;
                }
                case ARM_GPIO_SPEED_NORMAL: /* Default value */
                {
                    pin->open_speed = GPIO_SPEED_FREQ_MEDIUM;
                    break;
                }
                case ARM_GPIO_SPEED_HIGH:
                {
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
                    pin->open_speed = GPIO_SPEED_FREQ_VERY_HIGH;
#else

                    pin->open_speed  = GPIO_SPEED_FREQ_HIGH;
#endif
                    break;
                }
                default:
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }
            }

            /* Set the closed mode output speed */
            switch (close_mods & ARM_GPIO_SPEED_MASK)
            {
                case ARM_GPIO_SPEED_SLOW:
                {
                    pin->close_speed = GPIO_SPEED_FREQ_LOW;
                    break;
                }
                case ARM_GPIO_SPEED_NORMAL: /* Default value */
                {
                    pin->close_speed = GPIO_SPEED_FREQ_MEDIUM;
                    break;
                }
                case ARM_GPIO_SPEED_HIGH:
                {
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
                    pin->close_speed = GPIO_SPEED_FREQ_VERY_HIGH;

#else
                    pin->close_speed = GPIO_SPEED_FREQ_HIGH;
#endif

                    break;
                }
                default:
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }
            }

            /* Set GPIO pin open mode */
            temp = (open_mods & ARM_GPIO_MOD_MASK) >> ARM_GPIO_MOD_POS;
            switch (temp)
            {
                case ARM_GPIO_DISABLED:
                {
                    /* Set analog mode (used for disabled pin) */
                    pin->open_mode = GPIO_MODE_ANALOG;
                    break;
                }
                case ARM_GPIO_INPUT:
                {
                    pin->open_mode = GPIO_MODE_INPUT;
                    break;
                }
                case ARM_GPIO_OUTPUT:
                {
                    /* In case the output is open-drain */
                    if (open_mods & ARM_GPIO_OUTPUT_TYPE_MASK)
                    {
                        pin->open_mode = GPIO_MODE_OUTPUT_OD;
                    }
                    /* In case the output is push-pull */
                    else
                    {
                        pin->open_mode = GPIO_MODE_OUTPUT_PP;
                    }

                    break;
                }
                case ARM_GPIO_AF_OUTPUT:
                {
                    /* In case the alternate function is open-drain */
                    if (open_mods & ARM_GPIO_OUTPUT_TYPE_MASK)
                    {
                        pin->open_mode = GPIO_MODE_AF_OD;
                    }
                    /* In case the alternate function is push-pull */
                    else
                    {
                        pin->open_mode = GPIO_MODE_AF_PP;
                    }

                    break;
                }
                /* In case we set the GPIO as Alternate function (the alternate function needs to be set seperately) */
                case ARM_GPIO_AF_INPUT:
                {
#if defined(STM32F1)
                    pin->open_mode = GPIO_MODE_AF_INPUT;
#else
                    /* The STM32L4 handles AF inputs very weirdly, we have to set it as an AF push-pull, no idea why,
                     * dont ask me - TR */
                    pin->open_mode = GPIO_MODE_AF_PP;
#endif
                    break;
                }
                case ARM_GPIO_ANALOG:
                {
                    /* Note: GPIO_MODE_ANALOG is just setting the pin into analog mode (basically low energy high-z),
                     * but disconnecting it from the ADC, while GPIO_MODE_ANALOG_ADC_CONTROL also configures the
                     * GPIOx->ASCR register (on supported devices) which connects the pin to the ADC. */
#if defined(STM32L4)
                    /* Set analog ADC mode */
                    pin->open_mode = GPIO_MODE_ANALOG_ADC_CONTROL;
#else
                    pin->open_mode = GPIO_MODE_ANALOG;
#endif
                    break;
                }
                /* Invalid mode parameter */
                default:
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }
            }

            /* Reinitialize temporary variable */
            temp = 0;

            /* Set GPIO pin close mode */
            temp = (close_mods & ARM_GPIO_MOD_MASK);

            switch (temp)
            {
                case ARM_GPIO_DISABLED:
                {
                    /* Set analog mode (used for disabled pin) */
                    pin->close_mode = GPIO_MODE_ANALOG;
                    break;
                }
                case ARM_GPIO_INPUT:
                {
                    pin->close_mode = GPIO_MODE_INPUT;
                    break;
                }
                case ARM_GPIO_OUTPUT:
                {
                    /* In case the output is open-drain */
                    if (close_mods & ARM_GPIO_OUTPUT_TYPE_MASK)
                    {
                        pin->close_mode = GPIO_MODE_OUTPUT_OD;
                    }
                    /* In case the output is push-pull */
                    else
                    {
                        pin->close_mode = GPIO_MODE_OUTPUT_PP;
                    }

                    break;
                }
                /* In case we set the GPIO as Alternate function (the alternate function needs to be set seperately) */
                case ARM_GPIO_AF_OUTPUT:
                {
                    /* In case the alternate function is open-drain */
                    if (close_mods & ARM_GPIO_OUTPUT_TYPE_MASK)
                    {
                        pin->close_mode = GPIO_MODE_AF_OD;
                    }
                    /* In case the alternate function is push-pull */
                    else
                    {
                        pin->close_mode = GPIO_MODE_AF_PP;
                    }

                    break;
                }
                /* In case we set the GPIO as Alternate function (the alternate function needs to be set seperately) */
                case ARM_GPIO_AF_INPUT:
                {
#if defined(STM32F1)
                    pin->close_mode = GPIO_MODE_AF_INPUT;
#else
                    /* The STM32L4 handles AF inputs very weirdly, we have to set it as an AF push-pull, no idea why,
                     * dont ask me - TR */
                    pin->close_mode = GPIO_MODE_AF_PP;
#endif
                    break;
                }
                case ARM_GPIO_ANALOG:
                {
                    /* Note: GPIO_MODE_ANALOG is just setting the pin into analog mode (basically low energy high-z),
                     * but disconnecting it from the ADC, while GPIO_MODE_ANALOG_ADC_CONTROL also configures the
                     * GPIOx->ASCR register (on supported devices) which connects the pin to the ADC. */
#if defined(STM32L4)
                    /* Set analog ADC mode */
                    pin->close_mode = GPIO_MODE_ANALOG_ADC_CONTROL;
#else
                    pin->close_mode     = GPIO_MODE_ANALOG;
#endif
                    break;
                }
                /* Invalid mode parameter */
                default:
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }
            }

            break;
        }
        /* Open/Close GPIO pin */
        case ARM_GPIO_SET_STATE:
        {
            GPIO_InitTypeDef gpio_init = {0};

            /* Check if we need to open or close the GPIO pin*/
            switch (arg)
            {
                /* Open the GPIO pin */
                case OPEN:
                {
                    /* Check if pin needs to be opened (hasn't been opened before) */
                    if (!pin->is_open)
                    {
                        DriverStatus.pins_open++;

                        pin->is_open = true;
                    }

                    /* Configure GPIO Init struct according to the pin configurations */
                    gpio_init.Pin   = pin->pin_num;
                    gpio_init.Mode  = pin->open_mode;
                    gpio_init.Pull  = pin->open_pullup;
                    gpio_init.Speed = pin->open_speed;
#if defined(STM32F1)
                    /* Configure the remap if needed. */
                    CLEAR_BIT(AFIO->MAPR, pin->open_alternate_function);
                    SET_BIT(AFIO->MAPR, pin->open_alternate_function);
#else
                    gpio_init.Alternate = pin->open_alternate_function;
#endif
                    /* Open GPIO pin according to its configurations */
                    HAL_GPIO_Init(pin->gpio_port, &gpio_init);

                    /* Write default output */
                    gpio_write(pin, pin->open_out);

                    break;
                }
                /* Close the GPIO pin */
                case CLOSE:
                {
                    /* Check if pin needs to be closed (hasn't been close before) */
                    if (pin->is_open)
                    {
                        DriverStatus.pins_open--;

                        pin->is_open = false;
                    }
                    /* Configure GPIO Init struct according to the pin configurations */
                    gpio_init.Pin   = pin->pin_num;
                    gpio_init.Mode  = pin->close_mode;
                    gpio_init.Pull  = pin->close_pullup;
                    gpio_init.Speed = pin->close_speed;
#if defined(STM32F1)
                    /* Configure the remap if needed. */
                    CLEAR_BIT(AFIO->MAPR, pin->close_alternate_function);
                    SET_BIT(AFIO->MAPR, pin->close_alternate_function);
#else
                    gpio_init.Alternate = pin->close_alternate_function;
#endif

                    /* Open GPIO pin according to its configurations */
                    HAL_GPIO_Init(pin->gpio_port, &gpio_init);

                    /* Write default output */
                    gpio_write(pin, pin->close_out);
                    break;
                }
                /* Invalid state given */
                default:
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }
            }

            break;
        }
        /* Set GPIO pin interrupt trigger */
        case ARM_GPIO_SET_INT_TRIGGER:
        {
            switch (arg)
            {
                /* Disable interrupts */
                case ARM_GPIO_INT_TRIGGER_NONE:
                {
                    /* Clear previous interrupt triggers */
                    pin->open_mode &= ~(GPIO_MODE_IT_RISING_FALLING);
                    LL_EXTI_DisableIT_0_31(pin->pin_num);
                    break;
                }
                /* Enable interrupt to be triggered on rising edge */
                case ARM_GPIO_INT_TRIGGER_RISING:
                {
                    /* Clear previous triggers */
                    pin->open_mode &= ~(GPIO_MODE_IT_RISING_FALLING);

                    /* Enable Interrupts on rising edge */
                    pin->open_mode |= GPIO_MODE_IT_RISING;

                    break;
                }
                /* Enable interrupts to be triggered on falling edge*/
                case ARM_GPIO_INT_TRIGGER_FALLING:
                {
                    /* Clear previous triggers */
                    pin->open_mode &= ~(GPIO_MODE_IT_RISING_FALLING);

                    /* Enable Interrupts on falling edge */
                    pin->open_mode |= GPIO_MODE_IT_FALLING;

                    break;
                }
                /* Enable interrupts on both rising and falling edge */
                case ARM_GPIO_INT_TRIGGER_BOTH:
                {
                    /* Enable interrupt trigger */
                    pin->open_mode |= GPIO_MODE_IT_RISING_FALLING;

                    break;
                }
                /* Invalid interrupt trigger parameter */
                default:
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }
            }

            if (pin->is_open)
            {
                gpio_control(pin, ARM_GPIO_SET_STATE, OPEN);
            }

            break;
        }
        /* Set GPIO pin interrupt handler */
        case ARM_GPIO_SET_INT_HANDLER:
        {
            /* Save handler in GPIO handlers struct */
            gpio_handlers[get_gpio_pos(pin->pin_num)] = (ARM_GPIO_Interrupt_t)arg;

            break;
        }
        /* Set GPIO alternate function */
        case ARM_GPIO_SET_AF:
        {
            /* Set open mode alternate function */
            pin->open_alternate_function = (arg & ARM_GPIO_AF_OPEN_MASK);

            /* Set close mode alternate function*/
            pin->close_alternate_function = (arg & ARM_GPIO_AF_CLOSE_MASK) >> ARM_GPIO_AF_END_POS;

            break;
        }
        /* Set or Clear pending IRQ */
        case ARM_GPIO_SET_IRQ:
        {
            return gpio_set_irq(pin, (bool)arg);
        }
#if defined(CONFIG_DISABLE_SWD_PINS)
        case ARM_GPIO_SET_SWD:
        {
            return gpio_set_swd(arg);
        }
#endif
        /* Invalid control parameter */
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description    : Get GPIO module status
 * Return         : GPIO module status
 **********************************************************************************************************************/
ARM_GPIO_STATUS gpio_get_status(void)
{
    return DriverStatus;
}

/**********************************************************************************************************************/
/* Private Function Implementations                                                                                   */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description    : Convert 1-hot gpio index to its position.
 * Input          : pin_one_hot - 1-hot pin index.
 * Return         : The one position. For example 0x1->0, 0x2->1, 0x4->2 and so on.
 **********************************************************************************************************************/
__STATIC_INLINE uint8_t get_gpio_pos(uint16_t pin_one_hot)
{
    uint8_t pos = 0;

    while (pin_one_hot > 1)
    {
        pos++;
        pin_one_hot = pin_one_hot >> 1;
    }

    return pos;
}

/***********************************************************************************************************************
 * Description    : GPIO pin set or clear pending IRQ
 * Input          : pin    - the GPIO pin
 *                  enable - set or clear
 * Return         : Execution status
 **********************************************************************************************************************/
__STATIC_INLINE int32_t gpio_set_irq(ARM_GPIO_PIN *pin, bool enable)
{
    /* Set the GPIO pin IRQ */
    if (enable)
    {
        /* Generates a Software interrupt on selected EXTI line */
        __HAL_GPIO_EXTI_GENERATE_SWIT(pin->pin_num);
    }
    /* Clear the GPIO pin IRQ */
    else
    {
        /* Clears the EXTI's line pending bits */
        __HAL_GPIO_EXTI_CLEAR_IT(pin->pin_num);
    }

    return ARM_DRIVER_OK;
}

#if defined(CONFIG_DISABLE_SWD_PINS)
/***********************************************************************************************************************
 * Description: Enable/Disable SWD pins.
 * Input      : enable - enable or disable flag.
 * Return     : Execution status
 * Note       : Currently only disabling SWD is supported.
 **********************************************************************************************************************/
static int32_t gpio_set_swd(bool enable)
{
    if (enable)
    {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    gpio_control(&GPIO_SWD_IO, ARM_GPIO_CONFIGURE_STATE, ARM_GPIO_CONF_MAKE(ARM_GPIO_DISABLED, ARM_GPIO_DISABLED));
    gpio_control(&GPIO_SWD_IO, ARM_GPIO_SET_STATE, CLOSE);

    gpio_control(&GPIO_SWD_CLK, ARM_GPIO_CONFIGURE_STATE, ARM_GPIO_CONF_MAKE(ARM_GPIO_DISABLED, ARM_GPIO_DISABLED));
    gpio_control(&GPIO_SWD_CLK, ARM_GPIO_SET_STATE, CLOSE);

    return ARM_DRIVER_OK;
}
#endif

/***********************************************************************************************************************
 * Description : Clear EXTI IRQ pending interrupts.
 **********************************************************************************************************************/
static void gpio_nvic_clear_irq(void)
{
    nvic_clear_irq(EXTI15_10_IRQn);
    nvic_clear_irq(EXTI9_5_IRQn);
    nvic_clear_irq(EXTI4_IRQn);
    nvic_clear_irq(EXTI3_IRQn);
    nvic_clear_irq(EXTI2_IRQn);
    nvic_clear_irq(EXTI1_IRQn);
    nvic_clear_irq(EXTI0_IRQn);
}

/***********************************************************************************************************************
 * Description : Enable/Disable EXTI IRQ lines that are configured 'enabled' in the user configuration struct.
 * Input       : enable - bool, enable/disable IRQ's.
 **********************************************************************************************************************/
static void gpio_nvic_enable_irq(bool enable)
{
    nvic_enable_irq(EXTI15_10_IRQn, gpio_conf.exti15_10_irq_enable && enable);
    nvic_enable_irq(EXTI9_5_IRQn, gpio_conf.exti9_5_irq_enable && enable);
    nvic_enable_irq(EXTI4_IRQn, gpio_conf.exti4_irq_enable && enable);
    nvic_enable_irq(EXTI3_IRQn, gpio_conf.exti3_irq_enable && enable);
    nvic_enable_irq(EXTI2_IRQn, gpio_conf.exti2_irq_enable && enable);
    nvic_enable_irq(EXTI1_IRQn, gpio_conf.exti1_irq_enable && enable);
    nvic_enable_irq(EXTI0_IRQn, gpio_conf.exti0_irq_enable && enable);
}

/***********************************************************************************************************************
 * Internal use (IRQ)
 ***********************************************************************************************************************

 **********************************************************************************************************************
 * Description : GPIO IRQHandler for the EXTI line that caused the interrupt (called from HAL_GPIO_EXTI_IRQHandler)
 * Input       : pin - GPIO pin number
 **********************************************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    ARM_GPIO_Interrupt_t handler = gpio_handlers[get_gpio_pos(pin)];

    if (handler != NULL)
    {
        handler();
    }
}

/***********************************************************************************************************************
 * Description : handle EXTI0 IRQ
 **********************************************************************************************************************/
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/***********************************************************************************************************************
 * Description : handle EXTI1 IRQ
 **********************************************************************************************************************/
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/***********************************************************************************************************************
 * Description : handle EXTI2 IRQ
 **********************************************************************************************************************/
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/***********************************************************************************************************************
 * Description : handle EXTI3 IRQ
 **********************************************************************************************************************/
void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/***********************************************************************************************************************
 * Description : handle EXTI4 IRQ
 **********************************************************************************************************************/
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/***********************************************************************************************************************
 * Description : handle EXTI5-9 IRQ
 **********************************************************************************************************************/
void EXTI9_5_IRQHandler(void)
{
    for (uint16_t index = 5; index <= 9; index++)
    {
        uint16_t line_index = (1 << index);
        HAL_GPIO_EXTI_IRQHandler(line_index);
    }
}

/***********************************************************************************************************************
 * Description : handle EXTI010-15 IRQ
 **********************************************************************************************************************/
void EXTI15_10_IRQHandler(void)
{
    for (uint16_t index = 10; index <= 15; index++)
    {
        uint16_t line_index = (1 << index);
        HAL_GPIO_EXTI_IRQHandler(line_index);
    }
}