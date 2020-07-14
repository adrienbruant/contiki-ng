#include "contiki.h"
#include "stm32l4xx_hal.h"
#include "dbg-handlers.h"

#include "dev/uart.h"
#include "dev/serial-line.h"

#include <unistd.h>

#define STDOUT_BUF_SIZE 128

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);
}

void platform_init_stage_one(void)
{
  HAL_Init();
}

void platform_init_stage_two(void)
{
  MX_GPIO_Init();

  uart_init(SERIAL_LINE_CONF_UART, SERIAL_LINE_CONF_BAUDRATE);
  uart_set_input(SERIAL_LINE_CONF_UART, serial_line_input_byte);

  serial_line_init();
}

void platform_init_stage_three(void)
{

}

void platform_idle(void)
{

}

int __stdout_putchar(int ch)
{
  return uart_putchar(SERIAL_LINE_CONF_UART, ch);
}

int _write(int file, char *ptr, int len)
{
  int wlen;
  if( (file == STDOUT_FILENO) || (file == STDERR_FILENO) ) {
    wlen = len;
    while( wlen > 0 ) {
      if( *ptr == '\n' ) {
        if( __stdout_putchar('\r') != '\r' ) {
          return -1;
        }
      }

      if( __stdout_putchar(*ptr) != *ptr ) {
        return -1;
      } else {
        ptr++;
        wlen--;
      }
    }
  }

  return len;
}
