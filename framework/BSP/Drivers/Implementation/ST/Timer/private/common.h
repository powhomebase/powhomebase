/***********************************************************************************************************************
 * Description: Header file of common functionality shared by the timer module implemenations.
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Libraries */
#include <stdint.h>
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Public Function Declarations                                                                                       */
/**********************************************************************************************************************/

/* Converts channel number to device specific enum */
uint32_t ll_tim_convert_channel_num(uint32_t channel);

/* Sets channel's Capture Compare register to value */
void ll_tim_set_channel_value(TIM_TypeDef *p_tim, uint32_t channel, uint32_t value);

/* Enables Capture/Compare interrupt */
void ll_tim_cc_irq_enable(TIM_TypeDef *p_tim, uint32_t channel);

/* Disables Capture/Compare interrupt */
void ll_tim_cc_irq_disable(TIM_TypeDef *p_tim, uint32_t channel);

/* Clears Capture/Compare interrupt */
void ll_tim_cc_irq_clear(TIM_TypeDef *p_tim, uint32_t channel);

/* Returns value from capture register of the selected channel */
uint32_t ll_tim_get_capture(TIM_TypeDef *p_tim, uint32_t channel);
