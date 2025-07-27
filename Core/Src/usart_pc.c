#include <usart_pc.h>
#include <usart.h>
#include <dma.h>



/**
 * @brief Setup PC usart (USART2) with DMA
 * 
 */
void setup_PC_usart(void) {

	gpio_mode_setup(UART_PC_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN|TX_PIN);
	gpio_set_output_options(UART_PC_PORT,GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,TX_PIN);
	gpio_set_af(UART_PC_PORT, GPIO_AF7, RX_PIN|TX_PIN); 

	/* Setup USART2 parameters. */
	usart_set_baudrate(UART_PC, 56000);
	usart_set_databits(UART_PC, 8);
	usart_set_stopbits(UART_PC, USART_STOPBITS_1);
	usart_set_mode(UART_PC, USART_MODE_TX_RX);
	usart_set_parity(UART_PC, USART_PARITY_NONE);
	usart_set_flow_control(UART_PC, USART_FLOWCONTROL_NONE);


    //usart_enable_tx_dma(UART_PC);
    usart_enable_rx_dma(UART_PC);

    
    /*
    usart_set_rx_timeout_value(UART_PC, 1000000);
    usart_enable_rx_timeout_interrupt(UART_PC);
    nvic_enable_irq(NVIC_UART7_IRQ);
    usart_enable_rx_timeout(UART_PC);
    //usart_enable_rx_interrupt(UART_PC);
    

    USART_CR1(UART_PC) |= USART_CR1_IDLEIE;
    usart_enable_rx_interrupt(UART_PC);

    nvic_set_priority(NVIC_UART7_IRQ, 0b10000001);
    nvic_enable_irq(NVIC_UART7_IRQ);
*/
	usart_enable(UART_PC);

    //init_uart_dma();
    
    
}


