#include <main.h>


#define UART_PC_PORT	GPIOA
#define RX_PIN			GPIO12
#define TX_PIN			GPIO11

#define UART_PC			USART6
#define UART_PC_RCC		RCC_USART6

#define UART_DMA 		DMA2
#define DMA_RCC         RCC_DMA2
#define RX_DMA_ST		DMA_STREAM1
#define TX_DMA_ST		DMA_STREAM6

#define NVIC_DMA_TX     NVIC_DMA2_STREAM6_IRQ
#define NVIC_DMA_RX     NVIC_DMA2_STREAM1_IRQ



void setup_PC_usart();
void start_uart_rx_reception(uint32_t n_bytes);