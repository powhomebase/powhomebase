/***********************************************************************************************************************
 * Description : STM32 UART base class implementation
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header file */
#include "uart.h"

/* Project Related */
#include <proj_assert.h>

/**********************************************************************************************************************/
/* HAL weak functions implementation                                                                                  */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: HAL Tx transfer complete override - calls `TxComplete` implementation.
 * Input      : huart - pointer to a UART handle.
 * Return     : None
 **********************************************************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    uart_resources_t *uart_resources = (uart_resources_t *)huart;

    proj_assert(uart_resources->m->TxComplete != NULL);
    uart_resources->m->TxComplete(uart_resources);
}

/***********************************************************************************************************************
 * Description: HAL Rx transfer complete override - calls `RxComplete` implementation.
 * Input      : huart - pointer to a UART handle.
 * Return     : None
 **********************************************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    uart_resources_t *uart_resources = (uart_resources_t *)huart;

    proj_assert(uart_resources->m->RxComplete != NULL);
    uart_resources->m->RxComplete(uart_resources);
}

/***********************************************************************************************************************
 * Description: HAL Rx half transfer complete override - calls `RxHalfComplete` implementation.
 * Input      : huart - pointer to a UART handle.
 * Return     : None
 **********************************************************************************************************************/
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    uart_resources_t *uart_resources = (uart_resources_t *)huart;

    proj_assert(uart_resources->m->RxHalfComplete != NULL);
    uart_resources->m->RxHalfComplete(uart_resources);
}

/***********************************************************************************************************************
 * Description: HAL error transfer override - calls `Error` implementation.
 * Input      : huart - pointer to a UART handle.
 * Return     : None
 **********************************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Upcast ST HAL's handle pointer to CMSIS resources pointer. In order for this to happen correctly
       the handle MUST be the first element in the resources and MUST be the struct
       (and not a pointer to this struct) */
    uart_resources_t *uart_resources = (uart_resources_t *)huart;

    proj_assert(uart_resources->m->Error != NULL);
    uart_resources->m->Error(uart_resources);
}