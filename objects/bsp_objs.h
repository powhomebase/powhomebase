/**********************************************************************************************************************/
/* Description : BSP Objects.                                                                                         */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Include                                                                                                            */
/**********************************************************************************************************************/

/* Libraries */
#include <Driver_GPIO.h>
#include <Driver_USART.h>
#include <Driver_SPI.h>
#include <Driver_QSPI.h>

/**********************************************************************************************************************/
/* Objects                                                                                                            */
/**********************************************************************************************************************/

/* GPIO */
extern ARM_GPIO_PIN GPIO_PC10;
extern ARM_GPIO_PIN GPIO_PC11;

/* UART */
extern ARM_DRIVER_USART Driver_USART1;

/* QSPI DMA */
extern ARM_DRIVER_QSPI Driver_OSPI2;

/* SPI DMA */
extern ARM_DRIVER_SPI Driver_SPI2;
