ifneq ($(findstring STM32L476,$(CONFIG_DEVICE)),)
PREPROCESSOR_MACROS += STM32L476xx
else ifneq ($(findstring STM32L452,$(CONFIG_DEVICE)),)
PREPROCESSOR_MACROS += STM32L452xx
else ifneq ($(findstring STM32L486,$(CONFIG_DEVICE)),)
PREPROCESSOR_MACROS += STM32L486xx
else
$(error Unsupported device)
endif

CONFIG_CPU_CORTEX_M4 := y

DEVICE_HEADER := \"stm32l4xx.h\"

INCLUDE_DIRS += \
$(ST_HAL_DIR)/devices/STM32L4/Device/Include \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Inc \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Inc/Legacy

C_SRC += \
$(ST_HAL_DIR)/devices/STM32L4/Device/Source/Templates/system_stm32l4xx.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_adc_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_adc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_can.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_comp.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_cortex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_crc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_cryp.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_cryp_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_dac_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_dac.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_dma.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_exti.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_flash_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_flash.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_gpio.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_hcd.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_i2c.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_irda.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_iwdg.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_mmc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_nand.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_nor.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_pcd_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_pcd.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_pwr.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_pwr_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_qspi.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_rcc_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_rcc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_rtc_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_rtc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_sd.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_smartcard.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_spi.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_spi_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_sram.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_tim_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_tim.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_uart.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_uart_ex.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_usart.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal_wwdg.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_hal.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_adc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_crc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_dac.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_dma.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_exti.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_gpio.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_i2c.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_pwr.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_rcc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_rtc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_sdmmc.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_spi.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_tim.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_usart.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_usb.c \
$(ST_HAL_DIR)/devices/STM32L4/HAL/Src/stm32l4xx_ll_utils.c
