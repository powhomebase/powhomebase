/***********************************************************************************************************************
 * Description: Implementation file of HW Ticker for STM32 devices.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own Header */
#include "hw_ticker_impl.h"

/* Private headers */
#include "private/common.h"

/* Libraries */
#if defined(STM32F1)
#include <stm32f1xx_ll_tim.h>
#elif defined(STM32F4)
#include <stm32f4xx_ll_tim.h>
#elif defined(STM32L4)
#include <stm32l4xx_ll_tim.h>
#elif defined(STM32WL)
#include <stm32wlxx_ll_tim.h>
#else
#error "Unsupported ST device! Please implement it (should be very simple)."
#endif

/**********************************************************************************************************************/
/* Interface Virtual Method Declarations                                                                              */
/**********************************************************************************************************************/

static int32_t start(hw_ticker_t *p_base, uint32_t value);
static int32_t stop(hw_ticker_t *p_base);
static int32_t set(hw_ticker_t *p_base, uint32_t value);

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

/* Virtual Methods */
hw_ticker_methods_t const hw_ticker_impl_m = {
    .Start = start,
    .Stop  = stop,
    .Set   = set,
};

/**********************************************************************************************************************/
/* Interface Virtual Method Definitions                                                                               */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Starts periodic tick generation.
 * Input      : p_this  - pointer to self
 *              value   - capture compare value
 * Return     : ARM_DRIVER return code
 **********************************************************************************************************************/
static int32_t start(hw_ticker_t *p_base, uint32_t value)
{
    hw_ticker_impl_t *p_this       = (hw_ticker_impl_t *)p_base;
    uint32_t          channel_enum = ll_tim_convert_channel_num(p_this->base.p_conf->channel);

    LL_TIM_OC_InitTypeDef channel_init;
    LL_TIM_OC_StructInit(&channel_init);

    channel_init.CompareValue = value;

    LL_TIM_OC_Init(p_this->p_instance, channel_enum, &channel_init);
    LL_TIM_OC_DisablePreload(p_this->p_instance, channel_enum);
    ll_tim_cc_irq_clear(p_this->p_instance, channel_enum);
    ll_tim_cc_irq_enable(p_this->p_instance, channel_enum);

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Stops periodic tick generation.
 * Input      : p_this  - pointer to self
 * Return     : ARM_DRIVER return code
 **********************************************************************************************************************/
static int32_t stop(hw_ticker_t *p_base)
{
    hw_ticker_impl_t *p_this       = (hw_ticker_impl_t *)p_base;
    uint32_t          channel_enum = ll_tim_convert_channel_num(p_this->base.p_conf->channel);

    ll_tim_cc_irq_disable(p_this->p_instance, channel_enum);

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Sets the capture compare value immediately.
 * Input      : p_this  - pointer to self
 *              value   - new capture compare value
 * Return     : ARM_DRIVER return code
 **********************************************************************************************************************/
static int32_t set(hw_ticker_t *p_base, uint32_t value)
{
    hw_ticker_impl_t *p_this       = (hw_ticker_impl_t *)p_base;
    uint32_t          channel_enum = ll_tim_convert_channel_num(p_this->base.p_conf->channel);

    ll_tim_set_channel_value(p_this->p_instance, channel_enum, value);

    return ARM_DRIVER_OK;
}
