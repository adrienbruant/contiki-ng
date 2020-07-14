#ifndef __NUCLEO_L432_DEF_H
#define __NUCLEO_L432_DEF_H

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

#define UART2_IO_CONFIG                 UART_IO_CONFIG_INIT(2, \
        VCP_RX_GPIO_Port, VCP_RX_Pin, GPIO_AF3_USART2, \
        VCP_TX_GPIO_Port, VCP_TX_Pin, GPIO_AF7_USART2)

#define UART2_RX_DMA                    1
#define UART2_RX_DMA_CHANNEL            6
#define UART2_RX_DMA_REQUEST            2

#define UART2_TX_PIN_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART2_RX_PIN_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()

#define SERIAL_LINE_CONF_UART           UART_ID_UART2
#define SERIAL_LINE_CONF_BAUDRATE       115200


#endif //__NUCLEO_L432_DEF_H