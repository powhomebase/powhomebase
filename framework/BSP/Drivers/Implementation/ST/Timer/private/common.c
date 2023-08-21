/***********************************************************************************************************************
 * Description: Implementation file of common functionality shared by the timer module implemenations.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own Header */
#include "common.h"

/* Libraries */
#if defined(STM32F1)
#include <stm32f1xx_ll_tim.h>
#elif defined(STM32F4)
#include <stm32f4xx_ll_tim.h>
#elif defined(STM32L4)
#include <stm32l4xx_ll_tim.h>
#elif defined(STM32U5)
#include <stm32u5xx_ll_tim.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_tim.h>
#else
#error "Unsupported ST device! Please implement it (should be very simple)."
#endif

/* Project Related */
#include <proj_exception.h>

/**********************************************************************************************************************/
/* Public Function Definitions                                                                                        */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Converts channel number to device specific enum/macro of Low Level API
 * Input      : channel - channel of HW Timer
 * Return     : channel enum in api form
 **********************************************************************************************************************/
uint32_t ll_tim_convert_channel_num(uint32_t channel)
{
    switch (channel)
    {
#if defined(LL_TIM_CHANNEL_CH1)
        case 1:
        {
            return LL_TIM_CHANNEL_CH1;
        }
#endif /* defined(LL_TIM_CHANNEL_CH1) */
#if defined(LL_TIM_CHANNEL_CH2)
        case 2:
        {
            return LL_TIM_CHANNEL_CH2;
        }
#endif /* defined(LL_TIM_CHANNEL_CH2) */
#if defined(LL_TIM_CHANNEL_CH3)
        case 3:
        {
            return LL_TIM_CHANNEL_CH3;
        }
#endif /* defined(LL_TIM_CHANNEL_CH3) */
#if defined(LL_TIM_CHANNEL_CH4)
        case 4:
        {
            return LL_TIM_CHANNEL_CH4;
        }
#endif /* defined(LL_TIM_CHANNEL_CH4) */
#if defined(LL_TIM_CHANNEL_CH5)
        case 5:
        {
            return LL_TIM_CHANNEL_CH5;
        }
#endif /* defined(LL_TIM_CHANNEL_CH5) */
#if defined(LL_TIM_CHANNEL_CH6)
        case 6:
        {
            return LL_TIM_CHANNEL_CH6;
        }
#endif /* defined(LL_TIM_CHANNEL_CH6) */
        default:
        {
            proj_exception();
            return 0;
        }
    }
}

/***********************************************************************************************************************
 * Description: Sets value of channel's capture compare register through Low Lever API
 * Input      : p_tim   - TIM object
 *              channel - channel of the timer (Low Level API enum)
 *              value   - new capture comapre value
 * Return     : None
 * Notes      : Depending on the channel's configuration, this might pre-set the value, meaning that the change will
 *              take effect only after an update event (overflow).
 **********************************************************************************************************************/
void ll_tim_set_channel_value(TIM_TypeDef *p_tim, uint32_t channel, uint32_t value)
{
    switch (channel)
    {
#if defined(LL_TIM_CHANNEL_CH1)
        case LL_TIM_CHANNEL_CH1:
        {
            LL_TIM_OC_SetCompareCH1(p_tim, value);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH1) */
#if defined(LL_TIM_CHANNEL_CH2)
        case LL_TIM_CHANNEL_CH2:
        {
            LL_TIM_OC_SetCompareCH2(p_tim, value);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH2) */
#if defined(LL_TIM_CHANNEL_CH3)
        case LL_TIM_CHANNEL_CH3:
        {
            LL_TIM_OC_SetCompareCH3(p_tim, value);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH3) */
#if defined(LL_TIM_CHANNEL_CH4)
        case LL_TIM_CHANNEL_CH4:
        {
            LL_TIM_OC_SetCompareCH4(p_tim, value);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH4) */
#if defined(LL_TIM_CHANNEL_CH5)
        case LL_TIM_CHANNEL_CH5:
        {
            LL_TIM_OC_SetCompareCH5(p_tim, value);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH5) */
#if defined(LL_TIM_CHANNEL_CH6)
        case LL_TIM_CHANNEL_CH6:
        {
            LL_TIM_OC_SetCompareCH6(p_tim, value);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH6) */
        default:
        {
            proj_exception();
        }
    }
}

/***********************************************************************************************************************
 * Description: Enables Capture/Compare interrupt on the selected channel
 * Input      : p_tim   - TIM object
 *              channel - channel of the timer (Low Level API enum)
 * Return     : None
 **********************************************************************************************************************/
void ll_tim_cc_irq_enable(TIM_TypeDef *p_tim, uint32_t channel)
{
    switch (channel)
    {
#if defined(LL_TIM_CHANNEL_CH1)
        case LL_TIM_CHANNEL_CH1:
        {
            LL_TIM_EnableIT_CC1(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH1) */
#if defined(LL_TIM_CHANNEL_CH2)
        case LL_TIM_CHANNEL_CH2:
        {
            LL_TIM_EnableIT_CC2(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH2) */
#if defined(LL_TIM_CHANNEL_CH3)
        case LL_TIM_CHANNEL_CH3:
        {
            LL_TIM_EnableIT_CC3(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH3) */
#if defined(LL_TIM_CHANNEL_CH4)
        case LL_TIM_CHANNEL_CH4:
        {
            LL_TIM_EnableIT_CC4(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH4) */
        default:
        {
            proj_exception();
        }
    }
}

/***********************************************************************************************************************
 * Description: Disables Capture/Compare interrupt on the selected channel
 * Input      : p_tim   - TIM object
 *              channel - channel of the timer (Low Level API enum)
 * Return     : None
 **********************************************************************************************************************/
void ll_tim_cc_irq_disable(TIM_TypeDef *p_tim, uint32_t channel)
{
    switch (channel)
    {
#if defined(LL_TIM_CHANNEL_CH1)
        case LL_TIM_CHANNEL_CH1:
        {
            LL_TIM_DisableIT_CC1(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH1) */
#if defined(LL_TIM_CHANNEL_CH2)
        case LL_TIM_CHANNEL_CH2:
        {
            LL_TIM_DisableIT_CC2(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH2) */
#if defined(LL_TIM_CHANNEL_CH3)
        case LL_TIM_CHANNEL_CH3:
        {
            LL_TIM_DisableIT_CC3(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH3) */
#if defined(LL_TIM_CHANNEL_CH4)
        case LL_TIM_CHANNEL_CH4:
        {
            LL_TIM_DisableIT_CC4(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH4) */
        default:
        {
            proj_exception();
        }
    }
}
/***********************************************************************************************************************
 * Description: Clears Capture/Compare interrupt on the selected channel
 * Input      : p_tim   - TIM object
 *              channel - channel of the timer (Low Level API enum)
 * Return     : None
 **********************************************************************************************************************/
void ll_tim_cc_irq_clear(TIM_TypeDef *p_tim, uint32_t channel)
{
    switch (channel)
    {
#if defined(LL_TIM_CHANNEL_CH1)
        case LL_TIM_CHANNEL_CH1:
        {
            LL_TIM_ClearFlag_CC1(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH1) */
#if defined(LL_TIM_CHANNEL_CH2)
        case LL_TIM_CHANNEL_CH2:
        {
            LL_TIM_ClearFlag_CC2(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH2) */
#if defined(LL_TIM_CHANNEL_CH3)
        case LL_TIM_CHANNEL_CH3:
        {
            LL_TIM_ClearFlag_CC3(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH3) */
#if defined(LL_TIM_CHANNEL_CH4)
        case LL_TIM_CHANNEL_CH4:
        {
            LL_TIM_ClearFlag_CC4(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH4) */
#if defined(LL_TIM_CHANNEL_CH5)
        case LL_TIM_CHANNEL_CH5:
        {
            LL_TIM_ClearFlag_CC5(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH5) */
#if defined(LL_TIM_CHANNEL_CH6)
        case LL_TIM_CHANNEL_CH6:
        {
            LL_TIM_ClearFlag_CC6(p_tim);
            break;
        }
#endif /* defined(LL_TIM_CHANNEL_CH6) */
        default:
        {
            proj_exception();
        }
    }
}

/***********************************************************************************************************************
 * Description: Returns value from capture register of the selected channel
 * Input      : p_tim   - TIM object
 *              channel - channel of the timer (Low Level API enum)
 * Return     : None
 **********************************************************************************************************************/
uint32_t ll_tim_get_capture(TIM_TypeDef *p_tim, uint32_t channel)
{
    switch (channel)
    {
#if defined(LL_TIM_CHANNEL_CH1)
        case LL_TIM_CHANNEL_CH1:
        {
            return LL_TIM_IC_GetCaptureCH1(p_tim);
        }
#endif /* defined(LL_TIM_CHANNEL_CH1) */
#if defined(LL_TIM_CHANNEL_CH2)
        case LL_TIM_CHANNEL_CH2:
        {
            return LL_TIM_IC_GetCaptureCH2(p_tim);
        }
#endif /* defined(LL_TIM_CHANNEL_CH2) */
#if defined(LL_TIM_CHANNEL_CH3)
        case LL_TIM_CHANNEL_CH3:
        {
            return LL_TIM_IC_GetCaptureCH3(p_tim);
        }
#endif /* defined(LL_TIM_CHANNEL_CH3) */
#if defined(LL_TIM_CHANNEL_CH4)
        case LL_TIM_CHANNEL_CH4:
        {
            return LL_TIM_IC_GetCaptureCH4(p_tim);
        }
#endif /* defined(LL_TIM_CHANNEL_CH4) */
        default:
        {
            proj_exception();
            return 0;
        }
    }
}
