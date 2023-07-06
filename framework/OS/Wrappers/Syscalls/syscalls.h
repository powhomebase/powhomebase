/**********************************************************************************************************************/
/* Description      : System calls header                                                                             */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Library headers */
#if !defined(TEST)
#include CMSIS_device_header
#endif
#include <cmsis_compiler.h>
#include <stdbool.h>
#include <stdint.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#if defined(TEST)
#define IRQn_Type uint32_t
#endif

/**********************************************************************************************************************/
/* Interface Functions                                                                                                */
/**********************************************************************************************************************/

/* Run callback in privileged mode */
void run_in_priv(void (*callback)(void*), void* input);

/* Soft reset the MCU */
void soft_reset(void);

/* Set interrupt priority */
void nvic_set_priority(IRQn_Type irq, uint32_t priority);

/* Clear pending IRQ */
void nvic_clear_irq(IRQn_Type irq);

/* Enable/disable interrupt */
void nvic_enable_irq(IRQn_Type irq, bool enable);

/* Disables all interrupts */
__STATIC_FORCEINLINE void nvic_disable_irq_all(void)
{
    __disable_irq();
}

/* Enables all interrupts */
__STATIC_FORCEINLINE void nvic_enable_irq_all(void)
{
    __enable_irq();
}

/* Check if interrupts are enabled */
__STATIC_FORCEINLINE bool nvic_are_irqs_enabled(void)
{
    return __get_PRIMASK() == 0;
}
