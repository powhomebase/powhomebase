ifneq ($(findstring STM32U5A5,$(CONFIG_DEVICE)),)
PREPROCESSOR_MACROS += STM32U5A5xx
else ifneq ($(findstring STM32U585,$(CONFIG_DEVICE)),)
PREPROCESSOR_MACROS += STM32U585xx
else
$(error Unsupported device)
endif

CONFIG_CPU_CORTEX_M33 := y

DEVICE_HEADER := \"stm32u5xx.h\"

INCLUDE_DIRS += \
$(ST_HAL_DIR)/devices/STM32U5/Device/Include \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Inc \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Inc/Legacy

C_SRC += \
$(ST_HAL_DIR)/devices/STM32U5/Device/Source/Templates/system_stm32u5xx.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_adc_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_adc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_comp.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_cortex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_crc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_dac_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_dac.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_dma.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_exti.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_flash_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_flash.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_gpio.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_i2c.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_irda.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_iwdg.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_pwr.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_pwr_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_rcc_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_rcc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_rtc_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_rtc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_sai.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_smartcard.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_spi.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_spi_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_tim_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_tim.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_uart.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_uart_ex.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_usart.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal_wwdg.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_hal.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_adc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_crc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_dac.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_dma.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_exti.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_gpio.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_i2c.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_pwr.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_rcc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_rtc.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_spi.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_lptim.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_tim.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_usart.c \
$(ST_HAL_DIR)/devices/STM32U5/HAL/Src/stm32u5xx_ll_utils.c
