#include <main.h>


#define UART_STM_PORT	    GPIOA
#define STM_RX_PIN			GPIO3
#define STM_TX_PIN			GPIO2

#define UART_STM		    USART2
#define UART_STM_RCC		RCC_USART2

// #define UART_DMA 		DMA2
// #define DMA_RCC         RCC_DMA2
// #define RX_DMA_ST		DMA_STREAM1
// #define TX_DMA_ST		DMA_STREAM6

// #define NVIC_DMA_TX     NVIC_DMA2_STREAM6_IRQ
// #define NVIC_DMA_RX     NVIC_DMA2_STREAM1_IRQ

#define UART_BT_PORT	    GPIOB
#define BT_RX_PIN			GPIO7
#define BT_TX_PIN			GPIO6

#define UART_BT		        USART1
#define UART_BT_RCC 		RCC_USART1

#define UART_DMA 		    DMA2
#define DMA_RCC             RCC_DMA2
#define RX_DMA_ST		    DMA_STREAM5
#define TX_DMA_ST		    DMA_STREAM7
#define UART_DMA_CH		    DMA_SxCR_CHSEL_4


#define NVIC_DMA_TX         NVIC_DMA2_STREAM7_IRQ
#define NVIC_DMA_RX         NVIC_DMA2_STREAM5_IRQ

void start_uart_rx_reception(uint32_t n_bytes);

void setup_STM_usart();
void setup_BT_usart();
