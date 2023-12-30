#include <main.h>


#define UART_PC_PORT	GPIOA
#define RX_PIN			GPIO2
#define TX_PIN			GPIO3

#define UART_PC			USART2
#define UART_PC_RCC		RCC_USART2

#define UART_DMA 		DMA1
#define RX_DMA_ST		DMA_STREAM5
#define TX_DMA_ST		DMA_STREAM6




void setup_PC_usart();
void start_uart_rx_reception(uint32_t n_bytes);