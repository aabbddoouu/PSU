#include <main.h>


#define UART_STM_PORT	GPIOA
#define STM_RX_PIN			GPIO3
#define STM_TX_PIN			GPIO2

#define UART_STM		USART2
#define UART_STM_RCC		RCC_USART2

// #define UART_DMA 		DMA2
// #define DMA_RCC         RCC_DMA2
// #define RX_DMA_ST		DMA_STREAM1
// #define TX_DMA_ST		DMA_STREAM6

// #define NVIC_DMA_TX     NVIC_DMA2_STREAM6_IRQ
// #define NVIC_DMA_RX     NVIC_DMA2_STREAM1_IRQ



void setup_STM_usart();
