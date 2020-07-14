#include "uart.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"

#ifndef UART1_IO_CONFIG
#define UART1_IO_CONFIG UART_IO_CONFIG_INIT(1, NULL, 0, 0, NULL, 0, 0)
#endif
#ifndef UART2_IO_CONFIG
#define UART2_IO_CONFIG UART_IO_CONFIG_INIT(2, NULL, 0, 0, NULL, 0, 0)
#endif
#ifdef USART3
#ifndef UART3_IO_CONFIG
#define UART3_IO_CONFIG UART_IO_CONFIG_INIT(3, NULL, 0, 0, NULL, 0, 0)
#endif
#endif
#ifdef UART4
#ifndef UART4_IO_CONFIG
#define UART4_IO_CONFIG UART_IO_CONFIG_INIT(4, NULL, 0, 0, NULL, 0, 0)
#endif
#endif

//Define clock enabling macros to empty strings if they are not defined
#ifndef UART1_RX_PIN_CLK_ENABLE
#define UART1_RX_PIN_CLK_ENABLE()
#endif
#ifndef UART1_TX_PIN_CLK_ENABLE
#define UART1_TX_PIN_CLK_ENABLE()
#endif
#ifndef UART2_RX_PIN_CLK_ENABLE
#define UART2_RX_PIN_CLK_ENABLE()
#endif
#ifndef UART2_TX_PIN_CLK_ENABLE
#define UART2_TX_PIN_CLK_ENABLE()
#endif
#ifdef USART3
#ifndef UART3_RX_PIN_CLK_ENABLE
#define UART3_RX_PIN_CLK_ENABLE()
#endif
#ifndef UART3_TX_PIN_CLK_ENABLE
#define UART3_TX_PIN_CLK_ENABLE()
#endif
#endif
#ifdef UART4
#ifndef UART4_RX_PIN_CLK_ENABLE
#define UART4_RX_PIN_CLK_ENABLE()
#endif
#ifndef UART4_TX_PIN_CLK_ENABLE
#define UART4_TX_PIN_CLK_ENABLE()
#endif
#endif

#ifndef UART1_RX_DMA
#define UART1_RX_DMA                    0
#define UART1_RX_DMA_CHANNEL            0
#define UART1_RX_DMA_REQUEST            0
uint8_t * const uart1_rx_buffer = NULL;
#else
uint8_t uart1_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
#endif
#ifndef UART2_RX_DMA
#define UART2_RX_DMA                    0
#define UART2_RX_DMA_CHANNEL            0
#define UART2_RX_DMA_REQUEST            0
uint8_t * const uart2_rx_buffer = NULL;
#else
uint8_t uart2_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
#endif
#ifndef UART3_RX_DMA
#define UART3_RX_DMA                    0
#define UART3_RX_DMA_CHANNEL            0
#define UART3_RX_DMA_REQUEST            0
uint8_t * const uart3_rx_buffer = NULL;
#else
uint8_t uart3_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
#endif
#ifndef UART4_RX_DMA
#define UART4_RX_DMA                    0
#define UART4_RX_DMA_CHANNEL            0
#define UART4_RX_DMA_REQUEST            0
uint8_t * const uart4_rx_buffer = NULL;
#else
uint8_t uart4_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
#endif

//We have to define this non existing DMA instance when the DMA is unused
#define DMA0                            NULL
//We need two indirection levels for macro expansion to work
#define DMA_INSTANCE_ind(dma)           DMA##dma
#define DMA_INSTANCE(dma)               DMA_INSTANCE_ind(dma)

//We have to define this non existing DMA IRQn when the DMA is unused
#define DMA0_Channel0_IRQn              0
//We need two indirection levels for macro expansion to work
#define DMA_IRQ_ind(dma, channel)       DMA##dma##_Channel##channel##_IRQn
#define DMA_IRQ(dma, channel)           DMA_IRQ_ind(dma, channel)

#define DMA_IRQ_HANDLER_ind(dma, channel)   DMA##dma##_Channel##channel##_IRQHandler
#define DMA_IRQ_HANDLER(dma, channel)       DMA_IRQ_HANDLER_ind(dma, channel)

#define UART1_DMA_IRQ_HANDLER           DMA_IRQ_HANDLER(UART1_RX_DMA, UART1_RX_DMA_CHANNEL)
#define UART2_DMA_IRQ_HANDLER           DMA_IRQ_HANDLER(UART2_RX_DMA, UART2_RX_DMA_CHANNEL)
#define UART3_DMA_IRQ_HANDLER           DMA_IRQ_HANDLER(UART3_RX_DMA, UART3_RX_DMA_CHANNEL)
#define UART4_DMA_IRQ_HANDLER           DMA_IRQ_HANDLER(UART4_RX_DMA, UART4_RX_DMA_CHANNEL)

#define UART_DMA_CONFIG_INIT(n) \
{\
  DMA_INSTANCE(UART##n##_RX_DMA),\
  UART##n##_RX_DMA_CHANNEL - 1,\
  UART##n##_RX_DMA_REQUEST,\
  DMA_IRQ(UART##n##_RX_DMA, UART##n##_RX_DMA_CHANNEL),\
  uart##n##_rx_buffer,\
  UART_DMA_RX_BUFFER_SIZE\
}

typedef struct
{
  GPIO_TypeDef *rx_port;
  uint32_t rx_pin;
  uint32_t rx_alternate;
  GPIO_TypeDef *tx_port;
  uint32_t tx_pin;
  uint32_t tx_alternate;
} Uart_IO_Config;

typedef struct
{
  DMA_TypeDef *rx_dma;
  uint32_t rx_dma_channel;
  uint32_t rx_dma_request;
  IRQn_Type rx_irq;
  uint8_t *rx_buffer;
  size_t rx_buffer_len;
} Uart_DMA_Config;

typedef struct
{
  USART_TypeDef *instance;
  IRQn_Type irq;
  Uart_IO_Config io_config;
  Uart_DMA_Config dma_config;
} Uart_Config;

typedef struct
{
  size_t rx_buffer_old_pos;
} Uart_Context;

const Uart_Config uart_configs[UART_ID_NB] = {
  [UART_ID_UART1] = {
    .instance = USART1,
    .irq = USART1_IRQn,
    .io_config = UART1_IO_CONFIG,
    .dma_config = UART_DMA_CONFIG_INIT(1)
  },
  [UART_ID_UART2] = {
    .instance = USART2,
    .irq = USART2_IRQn,
    .io_config = UART2_IO_CONFIG,
    .dma_config = UART_DMA_CONFIG_INIT(2)
  },
#ifdef USART3
  [UART_ID_UART3] = {
    .instance = USART3,
    .irq = USART3_IRQn,
    .io_config = UART3_IO_CONFIG,
    .dma_config = UART_DMA_CONFIG_INIT(3)
  },
#endif
#ifdef UART4
  [UART_ID_UART4] = {
    .instance = UART4,
    .irq = UART4_IRQn,
    .io_config = UART4_IO_CONFIG,
    .dma_config = UART_DMA_CONFIG_INIT(4)
  },
#endif
};

static Uart_Context uart_contexts[UART_ID_NB] = {
  [UART_ID_UART1] = {
    .rx_buffer_old_pos = 0
  },
  [UART_ID_UART2] = {
    .rx_buffer_old_pos = 0
  },
#ifdef UART3
  [UART_ID_UART3] = {
    .rx_buffer_old_pos = 0
  },
#endif
#ifdef UART4
  [UART_ID_UART4] = {
    .rx_buffer_old_pos = 0
  },
#endif
};

static int (* input_handlers[UART_ID_NB])(unsigned char c) = {NULL};

static void uart_gpio_init(Uart_Id id)
{
  GPIO_InitTypeDef GPIO_InitStruct = {
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH
  };

  Uart_IO_Config const * const io_config = &(uart_configs[id].io_config);

  switch( id )
  {
  case UART_ID_UART1:
    __HAL_RCC_USART1_CLK_ENABLE();
    UART1_RX_PIN_CLK_ENABLE();
    UART1_TX_PIN_CLK_ENABLE();
    break;
  case UART_ID_UART2:
    __HAL_RCC_USART2_CLK_ENABLE();
    UART2_RX_PIN_CLK_ENABLE();
    UART2_TX_PIN_CLK_ENABLE();
    break;
#ifdef USART3
  case UART_ID_UART3:
    __HAL_RCC_USART3_CLK_ENABLE();
    UART3_RX_PIN_CLK_ENABLE();
    UART3_TX_PIN_CLK_ENABLE();
    break;
#endif
#ifdef UART4
  case UART_ID_UART4:
    __HAL_RCC_UART4_CLK_ENABLE();
    UART4_RX_PIN_CLK_ENABLE();
    UART4_TX_PIN_CLK_ENABLE();
    break;
#endif
  default:
    return;
  }

  GPIO_InitStruct.Pin = io_config->rx_pin;
  GPIO_InitStruct.Alternate = io_config->rx_alternate;
  HAL_GPIO_Init(io_config->rx_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = io_config->tx_pin;
  GPIO_InitStruct.Alternate = io_config->tx_alternate;
  HAL_GPIO_Init(io_config->tx_port, &GPIO_InitStruct);
}

static void uart_dma_init(const Uart_Config *config)
{
  DMA_TypeDef *dma = config->dma_config.rx_dma;
  uint32_t channel_idx = config->dma_config.rx_dma_channel;
  uint32_t request = config->dma_config.rx_dma_request;
  IRQn_Type irq = config->dma_config.rx_irq;

  if( dma == DMA1 ) {
    __HAL_RCC_DMA1_CLK_ENABLE();
  } else if ( dma == DMA2 ) {
    __HAL_RCC_DMA2_CLK_ENABLE();
  }

  LL_DMA_SetPeriphRequest(dma, channel_idx, request);
  LL_DMA_SetDataTransferDirection(dma, channel_idx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(dma, channel_idx, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(dma, channel_idx, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(dma, channel_idx, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(dma, channel_idx, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(dma, channel_idx, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(dma, channel_idx, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(dma, channel_idx, (uint32_t)&(config->instance->RDR));
  LL_DMA_SetMemoryAddress(dma, channel_idx, (uint32_t)config->dma_config.rx_buffer);
  LL_DMA_SetDataLength(dma, channel_idx, UART_DMA_RX_BUFFER_SIZE);

  /* Enable HT & TC interrupts */
  LL_DMA_EnableIT_HT(dma, channel_idx);
  LL_DMA_EnableIT_TC(dma, channel_idx);

  /* DMA interrupt init */
  NVIC_SetPriority(irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(irq);

  LL_DMA_EnableChannel(dma, channel_idx);
}


int uart_init(Uart_Id id, uint32_t baudrate)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  USART_TypeDef *uart;
  int use_dma = 0;

  if(id >= UART_ID_NB) {
    //Unknown uart device
    return -1;
  }

  uart = uart_configs[id].instance;

  uart_gpio_init(id);

  //If this uart is configured to use a DMA
  if( uart_configs[id].dma_config.rx_dma != NULL ) {
    uart_dma_init(&uart_configs[id]);
    use_dma = 1;
  }

  /* USART configuration */
  USART_InitStruct.BaudRate = baudrate;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(uart, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(uart);

  if( use_dma ) {
    LL_USART_EnableDMAReq_RX(uart);
    LL_USART_EnableIT_IDLE(uart);
  } else {
    LL_USART_EnableIT_RXNE(uart);
  }

  /* USART interrupt */
  NVIC_SetPriority(uart_configs[id].irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(uart_configs[id].irq);

  LL_USART_Enable(uart);

  return 0;
}

int uart_write(Uart_Id id, const void *data, size_t len)
{
  USART_TypeDef *instance;

  if( id >= UART_ID_NB ) {
    return -1;
  }

  instance = uart_configs[id].instance;
  const uint8_t* d = data;

  for (size_t i=0; i<len; i++) {
      LL_USART_TransmitData8(instance, d[i]);
      while (!LL_USART_IsActiveFlag_TXE(instance)) {}
  }
  while (!LL_USART_IsActiveFlag_TC(instance)) {}

  return len;
}

int uart_putchar(Uart_Id id, int c)
{
  USART_TypeDef *instance;
  char ch;

  if( id >= UART_ID_NB ) {
    return -1;
  }

  instance = uart_configs[id].instance;
  ch = c;

  LL_USART_TransmitData8(instance, ch);
  while (!LL_USART_IsActiveFlag_TXE(instance)) {}

  return ch;
}

void uart_set_input(Uart_Id id, int (* input)(unsigned char c))
{
  if(id >= UART_ID_NB) {
    return;
  }

  input_handlers[id] = input;
}

static void uart_received_data(Uart_Id id, uint8_t *ptr, size_t len)
{
  int (* input_handler)(unsigned char c);

  if( input_handlers[id] == NULL ){
    return;
  }

  input_handler = input_handlers[id];

  for( size_t i=0; i<len; i++ ) {
    input_handler(ptr[i]);
  }
}

/**
 * \brief           Check for new data received with DMA
 */
static void uart_rx_check(Uart_Id id)
{
  const size_t rx_buffer_len = uart_configs[id].dma_config.rx_buffer_len;
  uint8_t *uart_rx_dma_buffer = uart_configs[id].dma_config.rx_buffer;
  size_t *old_pos = &(uart_contexts[id].rx_buffer_old_pos);
  size_t pos;

  /* Calculate current position in buffer */
  pos = rx_buffer_len - LL_DMA_GetDataLength(
      uart_configs[id].dma_config.rx_dma,
      uart_configs[id].dma_config.rx_dma_channel);
  if (pos != *old_pos)
  { /* Check change in received data */
    if (pos > *old_pos)
    { /* Current position is over previous one */
      /* We are in "linear" mode */
      /* Process data directly by subtracting "pointers" */
      uart_received_data(id, &uart_rx_dma_buffer[*old_pos], pos - *old_pos);
    }
    else
    {
      /* We are in "overflow" mode */
      /* First process data to the end of buffer */
      uart_received_data(id, &uart_rx_dma_buffer[*old_pos], rx_buffer_len - *old_pos);
      /* Check and continue with beginning of buffer */
      if (pos > 0)
      {
        uart_received_data(id, &uart_rx_dma_buffer[0], pos);
      }
    }
  }
  *old_pos = pos; /* Save current position as old */

  /* Check and manually update if we reached end of buffer */
  if (*old_pos == rx_buffer_len)
  {
    *old_pos = 0;
  }
}

//This function may be unused if no uart is configured to use the DMA
#if UART1_RX_DMA != 0 || UART2_RX_DMA != 0 || UART3_RX_DMA != 0 || UART4_RX_DMA != 0
static void uart_dma_irq_handler(Uart_Id id)
{
  DMA_TypeDef *dma_instance;
  uint32_t channel_idx;

  NVIC_ClearPendingIRQ(uart_configs[id].irq);

  dma_instance = uart_configs[id].dma_config.rx_dma;
  channel_idx = uart_configs[id].dma_config.rx_dma_channel;
  
  //Compute the position of the half transfer complete flag for the current channel
  const uint32_t hti = 1 << ((channel_idx) * 4 + DMA_ISR_HTIF1_Pos);
  //Compute the position of the transfer complete flag for the current channel
  const uint32_t tci = 1 << ((channel_idx) * 4 + DMA_ISR_TCIF1_Pos);


  /* Check half-transfer complete interrupt */
  if (LL_DMA_IsEnabledIT_HT(dma_instance, channel_idx) && (LL_DMA_ReadReg(dma_instance, ISR) & hti)) {
    LL_DMA_WriteReg(dma_instance, IFCR, hti);     /* Clear half-transfer complete flag */
    uart_rx_check(id);                            /* Check for data to process */
  }

  /* Check transfer-complete interrupt */
  if (LL_DMA_IsEnabledIT_TC(dma_instance, channel_idx) && (LL_DMA_ReadReg(dma_instance, ISR) & tci)) {
    LL_DMA_WriteReg(dma_instance, IFCR, tci);     /* Clear transfer complete flag */
    uart_rx_check(id);                            /* Check for data to process */
  }
}
#endif

static void uart_isr(Uart_Id id)
{
  USART_TypeDef *uart_instance;

  NVIC_ClearPendingIRQ(uart_configs[id].dma_config.rx_irq);

  uart_instance = uart_configs[id].instance;

  /* Check for IDLE line interrupt */
  if( LL_USART_IsEnabledIT_IDLE(uart_instance) && LL_USART_IsActiveFlag_IDLE(uart_instance) ) {
    LL_USART_ClearFlag_IDLE(uart_instance);       /* Clear IDLE line flag */
    uart_rx_check(id);                            /* Check for data to process */
  }

  /* Check for RX buffer not empty */
  if( LL_USART_IsEnabledIT_RXNE( uart_instance ) && LL_USART_IsActiveFlag_RXNE( uart_instance )) {
    uint8_t c = LL_USART_ReceiveData8(uart_instance);
    input_handlers[id](c);
  }
}

#ifdef UART4
void UART4_IRQHandler(void)
{
  uart_isr(UART_ID_UART4);
}
#endif

#define USART_ISR(n)  void USART##n##_IRQHandler(void) { uart_isr(n-1); }
USART_ISR(1);
USART_ISR(2);
#ifdef USART3
USART_ISR(3);
#endif

#if UART1_RX_DMA != 0
void UART1_DMA_IRQ_HANDLER (void)
{
  uart_dma_irq_handler(UART_ID_UART1);
}
#endif

#if UART2_RX_DMA != 0
void UART2_DMA_IRQ_HANDLER(void)
{
  uart_dma_irq_handler(UART_ID_UART2);
}
#endif

#if UART3_RX_DMA != 0
void UART3_DMA_IRQ_HANDLER(void)
{
  uart_dma_irq_handler(UART_ID_UART3);
}
#endif

#if UART4_RX_DMA != 0
void UART4_DMA_IRQ_HANDLER(void)
{
  uart_dma_irq_handler(UART_ID_UART4);
}
#endif
