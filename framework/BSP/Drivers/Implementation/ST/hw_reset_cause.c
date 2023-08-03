/**********************************************************************************************************************/
/* Description : HW reset cause implementation for STM32 devices.                                                     */
/**********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "hw_reset_cause.h"

/* Related project headers */
#include <logging.h>
#include <proj_assert.h>
#include <proj_exception.h>

/* Library headers */
#include CMSIS_device_header
#if defined(STM32L4)
#include <stm32l4xx_ll_rcc.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_rcc.h>
#elif defined(STM32F1)
#include <stm32f1xx_ll_rcc.h>
#elif defined(STM32F4)
#include <stm32f4xx_ll_rcc.h>
#else
#error "Unsupported device"
#endif

/**********************************************************************************************************************/
/* Interface Function Implementations                                                                                 */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Get the hw reset cause from RCC.
 * Return     : The HW reset cause.
 **********************************************************************************************************************/
hw_reset_cause_e hw_reset_cause_get(void)
{
#if defined(RCC_FLAG_BORRST)
    if (LL_RCC_IsActiveFlag_BORRST())
    {
        return HW_RESET_CAUSE_BOR;
    }
#endif /* RCC_FLAG_BORRST */

#if defined(RCC_FLAG_OBLRST)
    if (LL_RCC_IsActiveFlag_OBLRST())
    {
        return HW_RESET_CAUSE_OBL;
    }
#endif /* RCC_FLAG_OBLRST */

#if defined(RCC_FLAG_PORRST)
    if (LL_RCC_IsActiveFlag_PORRST())
    {
        return HW_RESET_CAUSE_POR;
    }
#endif /* RCC_FLAG_PORRST */

    if (LL_RCC_IsActiveFlag_IWDGRST())
    {
        return HW_RESET_CAUSE_WDG;
    }

    if (LL_RCC_IsActiveFlag_WWDGRST())
    {
        return HW_RESET_CAUSE_WDG;
    }

    if (LL_RCC_IsActiveFlag_LPWRRST())
    {
        return HW_RESET_CAUSE_LP;
    }

    if (LL_RCC_IsActiveFlag_PINRST())
    {
        return HW_RESET_CAUSE_PIN;
    }

#if defined(RCC_FLAG_FWRST)
    if (LL_RCC_IsActiveFlag_FWRST())
    {
        return HW_RESET_CAUSE_FW;
    }
#endif /* RCC_FLAG_FWRST */

    return HW_RESET_CAUSE_NONE;
}

/***********************************************************************************************************************
 * Description: Clear the HW reset cause flags.
 **********************************************************************************************************************/
void hw_reset_cause_clear(void)
{
    LL_RCC_ClearResetFlags();
}