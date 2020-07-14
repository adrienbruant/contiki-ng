#ifndef __UART_H
#define __UART_H

#include "contiki.h"

typedef enum
{
  UART_ID_UART1,
  UART_ID_UART2,
#ifdef USART3
  UART_ID_UART3,
#endif
#ifdef UART4
  UART_ID_UART4,
#endif

  //Must remain the last element
  UART_ID_NB
} Uart_Id;

int uart_init(Uart_Id id, uint32_t baudrate);

int uart_write(Uart_Id id, const void *data, size_t len);

int uart_putchar(Uart_Id id, int c);

void uart_set_input(Uart_Id id, int (* input)(unsigned char c));

#endif //__UART_H