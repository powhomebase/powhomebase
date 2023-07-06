/***********************************************************************************************************************
 * Description : STM32 UART base class header
 * Notes       : UART and UART DMA drivers are mapped to the same HAL weak functions when transfer is complete.
 *               We use this base class with abstract callbacks to support both at the same time
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project related */
#include CMSIS_device_header

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef struct _uart_resources uart_resources_t;

typedef struct {
    void (*TxComplete)(uart_resources_t *p_this);
    void (*RxComplete)(uart_resources_t *p_this);
    void (*RxHalfComplete)(uart_resources_t *p_this);
    void (*Error)(uart_resources_t *p_this);
} const uart_methods_t;

typedef struct _uart_resources {
    /* ST UART handler struct must be the struct itself (and not a pointer) so we could upcast the handle pointer to the
     * resource pointer in the implementation of the HAL functions */
    UART_HandleTypeDef uart_handle;

    /* Abstract callbacks */
    uart_methods_t *const m;
} uart_resources_t;