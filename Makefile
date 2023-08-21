##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [PROJECTGEN_VERSION] date: [GEN_DATE]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = pow


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SRC := \
Core/Src/main.c \
Core/Src/stm32u5xx_it.c \
framework/BSP/Drivers/Implementation/ST/gpio.c \
framework/OS/Wrappers/Syscalls/syscalls.c \
framework/BSP/Drivers/API/Timer/hw_timer_manager.c \
framework/BSP/Drivers/API/Timer/hw_ticker.c \
framework/BSP/Drivers/Implementation/ST/dma.c \
framework/BSP/Drivers/Implementation/ST/spi_dma.c \
framework/BSP/Drivers/Implementation/ST/i2c.c \
framework/BSP/Drivers/Implementation/ST/i2s_dma.c \
framework/BSP/Drivers/Implementation/ST/ospi_dma.c \
framework/BSP/Drivers/Implementation/ST/uart.c \
framework/BSP/Drivers/Implementation/ST/uart_interrupt.c \
framework/BSP/Drivers/Implementation/ST/Timer/lp_hw_timer_manager_impl.c \
framework/BSP/Drivers/Implementation/ST/Timer/hw_timer_manager_impl.c \
framework/BSP/Drivers/Implementation/ST/Timer/hw_ticker_impl.c \
framework/BSP/Drivers/Implementation/ST/Timer/private/common.c \
objects/bsp_objs.c \
mocks/cmsis_os2.c

# ASM sources
ASM_SOURCES = framework/BSP/MCU/VendorSpecific/ST/st-hal/devices/STM32U5/Device/Source/Templates/gcc/startup_stm32u585xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
# CPU_VALUE

# fpu
# FPU_VALUE

# float-abi
# FLOAT-ABI_VALUE

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -mcpu=cortex-m33

# macros for gcc
# AS defines
# AS_DEFS = AS_DEFS_VALUE

# C defines
PREPROCESSOR_MACROS =

# AS includes
AS_INCLUDES =

# C includes
INCLUDE_DIRS := \
Drivers/CMSIS/Include \
Core/Inc \
framework/BSP/Drivers \
framework/BSP/Drivers/API \
framework/BSP/Drivers/API/Timer \
framework/BSP/Drivers/Implementation/ST \
framework/BSP/MCU/CMSIS/Core \
framework/OS/Wrappers/Syscalls \
framework/Utils \
framework/Utils/zephyr \
objects \
mocks

CONFIG_DEVICE:= STM32U585
ST_HAL_DIR := framework/BSP/MCU/VendorSpecific/ST/st-hal
include $(ST_HAL_DIR)/Makefile

PREPROCESSOR_MACROS += CMSIS_device_header=$(DEVICE_HEADER)

INCLUDE_PATH := $(addprefix -I,$(INCLUDE_DIRS))
CFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(INCLUDE_PATH) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -ggdb3 -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32U585_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SRC:.c=.o)))
vpath %.c $(sort $(dir $(C_SRC)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***