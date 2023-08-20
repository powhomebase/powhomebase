/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include CMSIS_device_header
#include <stdint.h>
#include <Driver_DMA.h>
#include <Driver_GPIO.h>
#include <Driver_USART.h>
#include <Driver_SPI.h>
#include <Driver_QSPI.h>
#include <stm32u5xx_hal_icache.h>
#include <bsp_objs.h>

/* Private variables ---------------------------------------------------------*/

ARM_GPIO_PIN_LEVEL level;
uint32_t counter = 0;

#define UART_BUFFER_SIZE 20
uint8_t uart_data[UART_BUFFER_SIZE];

#define SPI_BUFFER_SIZE 20
uint8_t spi_data[SPI_BUFFER_SIZE];

#define I2S_BUFFER_SIZE 20
uint8_t i2s_data[I2S_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
void Error_Handler(void);

static void gpio_irq(void)
{
  level = gpio_read(&GPIO_PC11);
  counter++;
}

static void uart_event(uint32_t event)
{
  if (event == ARM_USART_EVENT_TX_COMPLETE)
  {
    Driver_USART1.Receive(uart_data,UART_BUFFER_SIZE);
  }
  else if(event == ARM_USART_EVENT_RECEIVE_COMPLETE)
  {
    uint32_t rx_count = Driver_USART1.GetRxCount();
    if (rx_count <= UART_BUFFER_SIZE)
    {
      Driver_USART1.Send(uart_data,rx_count);
    }
  }
  else if(event == ARM_USART_EVENT_RX_TIMEOUT)
  {
    uint32_t rx_count = Driver_USART1.GetRxCount();
    if (rx_count <= UART_BUFFER_SIZE)
    {
      Driver_USART1.Send(uart_data,rx_count);
    }
  }
  else{
    /* Some error */
  }
}

static void spi_event(uint32_t event)
{
  if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
  {
    uint32_t count = Driver_SPI2.GetDataCount();
    Driver_USART1.Send(spi_data,count);
  }
  else{
    /* Some error */
  }
}

static void qspi_event(uint32_t event)
{
  if (event == ARM_QSPI_EVENT_TRANSFER_COMPLETE)
  {
    uint8_t msg[] = "ARM QSPI EVENT";
    Driver_USART1.Send(msg, sizeof(msg));
  }
  else{
    /* Some error */
  }
}

static void i2s_event(uint32_t event)
{
  if (event == ARM_SAI_EVENT_SEND_COMPLETE || event == ARM_SAI_EVENT_RECEIVE_COMPLETE)
  {
    uint8_t msg[] = "ARM I2S EVENT";
    Driver_USART1.Send(msg, sizeof(msg));
  }
  else{
    uint8_t msg[] = "ARM I2S EVENT ERROR";
    Driver_USART1.Send(msg, sizeof(msg));
  }
}



static void gpio_games(void)
{
  gpio_control(&GPIO_PC10, ARM_GPIO_CONFIGURE_STATE, ARM_GPIO_CONF_MAKE(ARM_GPIO_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL, ARM_GPIO_DISABLED));
  gpio_control(&GPIO_PC11, ARM_GPIO_CONFIGURE_STATE, ARM_GPIO_CONF_MAKE(ARM_GPIO_INPUT, ARM_GPIO_DISABLED));

  gpio_control(&GPIO_PC11, ARM_GPIO_SET_INT_HANDLER, (uint32_t)gpio_irq);
  gpio_control(&GPIO_PC11, ARM_GPIO_SET_INT_TRIGGER, ARM_GPIO_INT_TRIGGER_RISING);

  gpio_control(&GPIO_PC10, ARM_GPIO_SET_STATE, OPEN);
  gpio_control(&GPIO_PC11, ARM_GPIO_SET_STATE, OPEN);

  gpio_write(&GPIO_PC10, HIGH);
  level = gpio_read(&GPIO_PC11);
  gpio_write(&GPIO_PC10, LOW);
  level = gpio_read(&GPIO_PC11);

  gpio_control(&GPIO_PC11, ARM_GPIO_SET_INT_TRIGGER, ARM_GPIO_INT_TRIGGER_FALLING);

  gpio_write(&GPIO_PC10, HIGH);
  level = gpio_read(&GPIO_PC11);
  gpio_write(&GPIO_PC10, LOW);
  level = gpio_read(&GPIO_PC11);

  gpio_control(&GPIO_PC11, ARM_GPIO_SET_INT_TRIGGER, ARM_GPIO_INT_TRIGGER_BOTH);

  gpio_write(&GPIO_PC10, HIGH);
  level = gpio_read(&GPIO_PC11);
  gpio_write(&GPIO_PC10, LOW);
  level = gpio_read(&GPIO_PC11);
}
  
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
/* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_DeInit();
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* Initialize all configured peripherals */
  gpio_initialize();
  gpio_power_control(ARM_POWER_FULL);
  
  gpio_games();

  Driver_USART1.Initialize(uart_event);
  Driver_USART1.PowerControl(ARM_POWER_FULL);
  Driver_USART1.Receive(uart_data,UART_BUFFER_SIZE);

  Driver_SPI2.Initialize(spi_event);
  Driver_SPI2.PowerControl(ARM_POWER_FULL);

  Driver_OSPI2.Initialize(qspi_event);
  Driver_OSPI2.PowerControl(ARM_POWER_FULL);

  Driver_I2S1.Initialize(i2s_event);
  Driver_I2S1.PowerControl(ARM_POWER_FULL);
  Driver_I2S1.Receive(i2s_data, I2S_BUFFER_SIZE);

  while (1)
  {
  
  }
}

/**
  * @brief System Clock Configuration

  * @retval None
  */
void SystemClock_Config(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
