#include <usart_STM2.h>
#include <usart.h>

void setup_STM_usart(void) {


	gpio_mode_setup(UART_STM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, STM_RX_PIN|STM_TX_PIN);
	gpio_set_output_options(UART_STM_PORT,GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,STM_TX_PIN);
	gpio_set_af(UART_STM_PORT, GPIO_AF7, STM_RX_PIN|STM_TX_PIN); 

	/* Setup USART2 parameters. */
	usart_set_baudrate(UART_STM, 115200);
	usart_set_databits(UART_STM, 8);
	usart_set_stopbits(UART_STM, USART_STOPBITS_1);
	usart_set_mode(UART_STM, USART_MODE_TX_RX);
	usart_set_parity(UART_STM, USART_PARITY_NONE);
	usart_set_flow_control(UART_STM, USART_FLOWCONTROL_NONE);

    usart_enable(UART_STM);

}
