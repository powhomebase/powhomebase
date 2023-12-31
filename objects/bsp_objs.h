/**********************************************************************************************************************/
/* Description : BSP Objects.                                                                                         */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Include                                                                                                            */
/**********************************************************************************************************************/

/* Libraries */
#include <Driver_Flash.h>
#include <Driver_GPIO.h>
#include <Driver_USART.h>
#include <Driver_I2C.h>
#include <Driver_SPI.h>
#include <Driver_QSPI.h>
#include <Driver_SAI.h>

/**********************************************************************************************************************/
/* Objects                                                                                                            */
/**********************************************************************************************************************/

/* GPIO */
extern ARM_GPIO_PIN GPIO_PC10;
extern ARM_GPIO_PIN GPIO_PC11;

/* UART */
extern ARM_DRIVER_USART Driver_USART1;

/* I2C */
extern ARM_DRIVER_I2C Driver_I2C1;

/* QSPI DMA */
extern ARM_DRIVER_QSPI Driver_OSPI2;

/* SPI DMA */
extern ARM_DRIVER_SPI Driver_SPI2;

/* I2S DMA */
extern ARM_DRIVER_SAI Driver_I2S1;

/* Internal Flash */
extern ARM_DRIVER_FLASH Driver_Flash;