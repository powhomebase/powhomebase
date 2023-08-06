/**********************************************************************************************************************/
/* Description : BSP Objects.                                                                                         */
/**********************************************************************************************************************/

/**********************************************************************************************************************/
/* Include                                                                                                            */
/**********************************************************************************************************************/

/* Own Header */
#include "bsp_objs.h"

/* Libraries */
#include <spi_dma.h>
#include <ospi_dma.h>
#include <gpio.h>
#include <uart_interrupt.h>
#include <dma.h>

/**********************************************************************************************************************/
/* Objects                                                                                                            */
/**********************************************************************************************************************/

/* GPIO */
gpio_user_conf_t gpio_conf = {
  .exti11_irq_enable = true,
};

GPIO_GENERATE_OBJECT(PC10, GPIOC, GPIO_PIN_10);
GPIO_GENERATE_OBJECT(PC11, GPIOC, GPIO_PIN_11);

/* UART */
uart_int_user_conf_t usart1_user_conf = {
    .instance = USART1, /* UART instance*/

    .baudrate = 115200,
    .word_length =  UART_WORDLENGTH_8B,  
    .stop_bits =  UART_STOPBITS_1,    
    .parity =  UART_PARITY_NONE,     
    .mode =  UART_MODE_TX_RX,        
    .hw_control =  UART_HWCONTROL_NONE,
    .oversampling =  UART_OVERSAMPLING_16, 

    .irq = USART1_IRQn,

    .one_bit_sampling = UART_ONE_BIT_SAMPLE_DISABLE,
    .advanced_features =&(UART_AdvFeatureInitTypeDef){UART_ADVFEATURE_NO_INIT},
} ;

GPIO_GENERATE_OBJECT(PA9, GPIOA, GPIO_PIN_9);
GPIO_GENERATE_OBJECT(PA10, GPIOA, GPIO_PIN_10);

static uart_int_gpios_t usart1_gpios_conf = {
    .rx = &GPIO_PA10,
    .tx = &GPIO_PA9,
    .cts = GPIO_NOT_CONNECTED,
    .rts = GPIO_NOT_CONNECTED,
    .tx_io_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL, ARM_GPIO_DISABLED),
    .rx_io_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_INPUT, ARM_GPIO_DISABLED),
    .alternate_f = GPIO_AF7_USART1,
};

USART_INTERRUPT_GENERATE_OBJECT(USART, 1, usart1_user_conf, usart1_gpios_conf);

/* QSPI DMA */

static dma_user_conf_t dma_user_conf_1_12 ={
  .dma_instance = GPDMA1_Channel12,
  .irq = GPDMA1_Channel12_IRQn,
  .dma_init = &(DMA_InitTypeDef){
    .Request = GPDMA1_REQUEST_OCTOSPI2,
    .BlkHWRequest = DMA_BREQ_SINGLE_BURST,
    .Direction = DMA_PERIPH_TO_MEMORY,
    .SrcInc = DMA_SINC_FIXED,
    .DestInc = DMA_DINC_INCREMENTED,
    .SrcDataWidth = DMA_SRC_DATAWIDTH_WORD,
    .DestDataWidth = DMA_DEST_DATAWIDTH_WORD,
    .Priority = DMA_HIGH_PRIORITY,
    .SrcBurstLength = 4,
    .DestBurstLength = 4,
    .TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0,
    .TransferEventMode = DMA_TCEM_BLOCK_TRANSFER,
    .Mode = DMA_NORMAL,
  },
  .Parent = OCTOSPI2,
};

DMA_GENERATE_OBJECT(1, 12, dma_user_conf_1_12);

GPIO_GENERATE_OBJECT(PF0, GPIOF, GPIO_PIN_0);
GPIO_GENERATE_OBJECT(PF1, GPIOF, GPIO_PIN_1);
GPIO_GENERATE_OBJECT(PF2, GPIOF, GPIO_PIN_2);
GPIO_GENERATE_OBJECT(PF3, GPIOF, GPIO_PIN_3);
GPIO_GENERATE_OBJECT(PF4, GPIOF, GPIO_PIN_4);
GPIO_GENERATE_OBJECT(PI5, GPIOI, GPIO_PIN_5);

static ospi_dma_gpios_t ospi2_dma_gpios= {
  .io0 = &GPIO_PF0,
  .io0_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_SPEED_HIGH, ARM_GPIO_DISABLED),
  .io1 = &GPIO_PF1,
  .io1_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_SPEED_HIGH, ARM_GPIO_DISABLED),
  .io2 = &GPIO_PF2,
  .io2_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_SPEED_HIGH, ARM_GPIO_DISABLED),
  .io3 = &GPIO_PF3,
  .io3_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_SPEED_HIGH, ARM_GPIO_DISABLED),
  .cs = &GPIO_PI5,
  .cs_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_SPEED_HIGH, ARM_GPIO_DISABLED),
  .clk = &GPIO_PF4,
  .clk_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL | ARM_GPIO_SPEED_HIGH, ARM_GPIO_DISABLED),
  .alternate_f = GPIO_AF5_OCTOSPI2,
};

ospi_dma_user_conf_t ospi2_dma_user_conf = {
    .instance = OCTOSPI2,
    .init = &(OSPI_InitTypeDef){
      .FifoThreshold = 16,
      .DualQuad = HAL_OSPI_DUALQUAD_DISABLE,
      .MemoryType = HAL_OSPI_MEMTYPE_MACRONIX,
      .DeviceSize = 26,
      .ChipSelectHighTime = 2,
      .FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE,
      .ClockMode = HAL_OSPI_CLOCK_MODE_0,
      .WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED,
      .ClockPrescaler = 4,
      .SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE,
      .DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE,
      .ChipSelectBoundary = 0,
      .DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED,
      .MaxTran = 0,
      .Refresh = 0,
    },
    .irq = OCTOSPI2_IRQn,
};

OSPI_DMA_GENERATE_OBJECT(2, ospi2_dma_user_conf, ospi2_dma_gpios, Driver_DMA1_12, DMA1_12_handle)

void GPDMA1_Channel12_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DMA1_12_handle);
}

/* SPI DMA */

/* RX */
static dma_user_conf_t dma_user_conf_1_1 ={
  .dma_instance = GPDMA1_Channel1,
  .irq = GPDMA1_Channel1_IRQn,
  .dma_init = &(DMA_InitTypeDef){
    .Request = GPDMA1_REQUEST_SPI2_RX,
    .BlkHWRequest = DMA_BREQ_SINGLE_BURST,
    .Direction = DMA_PERIPH_TO_MEMORY,
    .SrcInc = DMA_SINC_FIXED,
    .DestInc = DMA_DINC_FIXED,
    .SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE,
    .DestDataWidth = DMA_DEST_DATAWIDTH_BYTE,
    .Priority = DMA_HIGH_PRIORITY,
    .SrcBurstLength = 1,
    .DestBurstLength = 1,
    .TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0,
    .TransferEventMode = DMA_TCEM_BLOCK_TRANSFER,
    .Mode = DMA_NORMAL,
  },
  .Parent = SPI2,
};

DMA_GENERATE_OBJECT(1, 1, dma_user_conf_1_1);

/* TX */
static dma_user_conf_t dma_user_conf_1_0 ={
  .dma_instance = GPDMA1_Channel0,
  .irq = GPDMA1_Channel0_IRQn,
  .dma_init = &(DMA_InitTypeDef){
    .Request = GPDMA1_REQUEST_SPI2_TX,
    .BlkHWRequest = DMA_BREQ_SINGLE_BURST,
    .Direction = DMA_MEMORY_TO_PERIPH,
    .SrcInc = DMA_SINC_FIXED,
    .DestInc = DMA_DINC_FIXED,
    .SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE,
    .DestDataWidth = DMA_DEST_DATAWIDTH_BYTE,
    .Priority = DMA_HIGH_PRIORITY,
    .SrcBurstLength = 1,
    .DestBurstLength = 1,
    .TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0,
    .TransferEventMode = DMA_TCEM_BLOCK_TRANSFER,
    .Mode = DMA_NORMAL,
  },
  .Parent = SPI2,
};

DMA_GENERATE_OBJECT(1, 0, dma_user_conf_1_0);

GPIO_GENERATE_OBJECT(PD4, GPIOD, GPIO_PIN_4);
GPIO_GENERATE_OBJECT(PD3, GPIOD, GPIO_PIN_3);
GPIO_GENERATE_OBJECT(PD1, GPIOD, GPIO_PIN_1);
GPIO_GENERATE_OBJECT(PB12, GPIOB, GPIO_PIN_12);

static spi_dma_gpios_t spi2_gpios = {
    .mosi = &GPIO_PD4,
    .mosi_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL, ARM_GPIO_DISABLED),
    .miso = &GPIO_PD3,
    .miso_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_INPUT, ARM_GPIO_DISABLED),
    .cs = &GPIO_PB12,
    .cs_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL, ARM_GPIO_DISABLED),
    .clk = &GPIO_PD1,
    .clk_conf = ARM_GPIO_CONF_MAKE(ARM_GPIO_AF_OUTPUT | ARM_GPIO_OUTPUT_PUSH_PULL, ARM_GPIO_DISABLED),
    .alternate_f = GPIO_AF5_SPI2,
};

static spi_dma_user_conf_t spi2_user_conf = {
    .instance = SPI2,                /* SPI registers base address */
    .init = &(SPI_InitTypeDef){
      .Mode = SPI_MODE_MASTER,
      .Direction = SPI_DIRECTION_2LINES,
      .DataSize = SPI_DATASIZE_4BIT,
      .CLKPolarity = SPI_POLARITY_LOW,
      .CLKPhase = SPI_PHASE_1EDGE,
      .NSS = SPI_NSS_HARD_OUTPUT,
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
      .CRCPolynomial = 0x7,
      .NSSPMode = SPI_NSS_PULSE_ENABLE,
      .NSSPolarity = SPI_NSS_POLARITY_LOW,
      .FifoThreshold = SPI_FIFO_THRESHOLD_01DATA,
      .MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE,
      .MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE,
      .MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE,
      .MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE,
      .IOSwap = SPI_IO_SWAP_DISABLE,
      .ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY,
      .ReadyPolarity = SPI_RDY_POLARITY_HIGH,
    },                    /* SPI communication parameters */
    .max_allowed_baudrate_hz = 10000000, /* Maximun allowed SPI baud rate */
    .enforce_full_transfers = true,  /* Allow only complete transfers */
};

SPI_DMA_GENERATE_OBJECT(                                                                                       \
    2, spi2_user_conf, spi2_gpios, DMA1_0_handle, Driver_DMA1_0, DMA1_1_handle, Driver_DMA1_1);

void GPDMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DMA1_1_handle);
}

void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DMA1_0_handle);
}
