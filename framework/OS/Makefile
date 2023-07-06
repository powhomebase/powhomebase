include framework/OS/Wrappers/Makefile

INCLUDE_DIRS += \
framework/OS/CMSIS_OS/Include \
framework/OS/RTX/Include

C_SRC += \
framework/OS/CMSIS_OS/Source/os_systick.c \
framework/OS/Override/os_override.c \
framework/OS/RTX/Source/rtx_delay.c \
framework/OS/RTX/Source/rtx_evflags.c \
framework/OS/RTX/Source/rtx_evr.c \
framework/OS/RTX/Source/rtx_kernel.c \
framework/OS/RTX/Source/rtx_lib.c \
framework/OS/RTX/Source/rtx_memory.c \
framework/OS/RTX/Source/rtx_mempool.c \
framework/OS/RTX/Source/rtx_msgqueue.c \
framework/OS/RTX/Source/rtx_mutex.c \
framework/OS/RTX/Source/rtx_semaphore.c \
framework/OS/RTX/Source/rtx_system.c \
framework/OS/RTX/Source/rtx_thread.c \
framework/OS/RTX/Source/rtx_timer.c

ifeq ($(CONFIG_RTX_EVENT_RECORDER),y)
INCLUDE_DIRS +=framework/EventRecorder
C_SRC += framework/EventRecorder/EventRecorder.c
endif
