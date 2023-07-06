########################################################################################################################
#                                         Awesome POW Makefile                                                         #
########################################################################################################################

TARGETNAME := st_hal.a

ifeq ($(V), 1)
Q :=
vecho = @true
else
Q := @
vecho = @echo
endif

#Embedded toolchain
CC := arm-none-eabi-gcc
LD := $(CC)
AR := arm-none-eabi-ar
AS := $(CC)
OBJCOPY := arm-none-eabi-objcopy

BUILDDIR := build
BINDIR   := $(BUILDDIR)/bin
OBJDIR   := $(BUILDDIR)/obj

########################################################################################################################
#                                                   Files                                                              #
########################################################################################################################

ST_HAL_DIR := .
include Makefile

INCLUDE_DIRS += \
scripts/ci/build/Core

########################################################################################################################
#                                                   Flags                                                              #
########################################################################################################################

CFLAGS += -std=c18 -ggdb -ffunction-sections -fdata-sections -fno-strict-aliasing -O0

ifeq ($(CONFIG_CPU_CORTEX_M3),y)
COMMONFLAGS += -mcpu=cortex-m3
else ifeq ($(CONFIG_CPU_CORTEX_M4),y)
COMMONFLAGS += -mcpu=cortex-m4
endif

ifeq ($(CONFIG_FPU),y)
COMMONFLAGS += -mfloat-abi=softfp
else
COMMONFLAGS += -mfloat-abi=soft
endif

COMMONFLAGS += -mthumb -mfpu=auto -specs=nano.specs -specs=nosys.specs

CFLAGS += $(COMMONFLAGS) -Werror -Wall -Wextra -Wno-unknown-pragmas
ASFLAGS += $(COMMONFLAGS)

INCLUDE_PATH = $(addprefix -I,$(INCLUDE_DIRS))
CFLAGS += $(INCLUDE_PATH)

ifneq ($(BUILD_DEFINES),)
PREPROCESSOR_MACROS += $(BUILD_DEFINES)
endif

CFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))
ASFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))

########################################################################################################################
#                                                   Rules                                                              #
########################################################################################################################

SOURCEFILES := $(C_SRC) $(S_SRC) $(s_SRC)

C_PATHS = $(sort $(dir $(C_SRC)))
S_PATHS = $(sort $(dir $(S_SRC) $(s_SRC)))

vpath %.c $(C_PATHS)
vpath %.s $(S_PATHS)
vpath %.S $(S_PATHS)

source_obj1 := $(SOURCEFILES:.c=.o)
source_obj2 := $(source_obj1:.s=.o)
source_objs := $(source_obj2:.S=.o)

all_objs := $(addprefix $(OBJDIR)/, $(notdir $(source_objs)))

all: $(BINDIR)/$(TARGETNAME)

$(BINDIR)/$(TARGETNAME): $(all_objs) |$(BINDIR)
	$(vecho) $(BLUE)Archiving$(RESET) into $(UNDERLINE)$(GREEN)$@$(RESET)
	$(Q)$(AR) -r $@ $^

clean:
	rm -rf $(BUILDDIR)

$(BINDIR):
	mkdir -p $(BINDIR)
$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o : %.c |$(OBJDIR)
	$(vecho) $(BLUE)Compiling$(RESET) $<
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o : %.S |$(OBJDIR)
	$(vecho) $(BLUE)Compiling$(RESET) $<
	$(Q)$(AS) $(ASFLAGS) -c $< -o $@

$(OBJDIR)/%.o : %.s |$(OBJDIR)
	$(vecho) $(BLUE)Compiling$(RESET) $<
	$(Q)$(AS) $(ASFLAGS) -c $< -o $@

########################################################################################################################
#                                               Pretty Print Helper tools                                              #
########################################################################################################################
ECHO = @echo
ESC = 

RESET = $(ESC)[0m

UNDERLINE = $(ESC)[4m

RED = $(ESC)[31m
GREEN = $(ESC)[32m
YELLOW = $(ESC)[33m
BLUE = $(ESC)[34m
MAGENTA = $(ESC)[35m
CYAN = $(ESC)[36m

