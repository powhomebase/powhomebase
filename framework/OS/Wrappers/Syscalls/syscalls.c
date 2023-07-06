/***********************************************************************************************************************
 * Description      : System calls implementation.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "syscalls.h"

/* Library headers */
#include <stdint.h>
#include <string.h>

/***********************************************************************************************************************
 * Description: Run callback in privileged mode.
 * Input      : callback - void (void *), function pointer.
 *              input    - void *, callback input.
 * Return     : void - None
 **********************************************************************************************************************/
void run_in_priv(void (*callback)(void*), void* input)
{
    callback(input);
}

/***********************************************************************************************************************
 * Description: Soft reset the MCU.
 * Input      : void - None
 * Return     : void - None
 **********************************************************************************************************************/
void soft_reset(void)
{
    NVIC_SystemReset();
}

/***********************************************************************************************************************
 * Description: Set interrupt priority.
 * Input      : irq      - IRQn_Type, IRQ object.
 *              priority - uint32_t, IRQ priority.
 * Return     : void - None
 **********************************************************************************************************************/
void nvic_set_priority(IRQn_Type irq, uint32_t priority)
{
    NVIC_SetPriority(irq, priority);
}

/***********************************************************************************************************************
 * Description: Clear pending IRQ.
 * Input      : irq - IRQn_Type, IRQ to clear.
 * Return     : void - None
 **********************************************************************************************************************/
void nvic_clear_irq(IRQn_Type irq)
{
    NVIC_ClearPendingIRQ(irq);
}

/***********************************************************************************************************************
 * Description: Enable/disable interrupt.
 * Input      : irq    - IRQn_Type, IRQ object.
 * Input      : enable - bool, 'true' to enable IRQ, 'false' to disable IRQ.
 * Return     : void - None
 **********************************************************************************************************************/
void nvic_enable_irq(IRQn_Type irq, bool enable)
{
    if (enable)
    {
        NVIC_EnableIRQ(irq);
    }
    else
    {
        NVIC_DisableIRQ(irq);
    }
}
