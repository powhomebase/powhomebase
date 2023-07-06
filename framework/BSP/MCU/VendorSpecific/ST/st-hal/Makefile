ifndef ST_HAL_DIR
$(error ST_HAL_DIR not defined)
endif

PREPROCESSOR_MACROS += USE_HAL_DRIVER USE_FULL_LL_DRIVER

ifneq ($(findstring STM32F1,$(CONFIG_DEVICE)),)
CONFIG_PART_FAMILY := "STM32F1"

include $(ST_HAL_DIR)/devices/STM32F1/Makefile

else ifneq ($(findstring STM32WL,$(CONFIG_DEVICE)),)

CONFIG_PART_FAMILY := "STM32WL"

include $(ST_HAL_DIR)/devices/STM32WL/Makefile

else ifneq ($(findstring STM32F4,$(CONFIG_DEVICE)),)

CONFIG_PART_FAMILY := "STM32F4"

include $(ST_HAL_DIR)/devices/STM32F4/Makefile

else ifneq ($(findstring STM32L4,$(CONFIG_DEVICE)),)

CONFIG_PART_FAMILY := "STM32L4"

include $(ST_HAL_DIR)/devices/STM32L4/Makefile

else
$(error Unsupported device)
endif

ifeq ($(BUILD_TEST),y)
INCLUDE_DIRS += \
$(ST_HAL_DIR)/devices/$(CONFIG_PART_FAMILY)/HAL/Config
endif
