#include <usart_STM2.h>
#include <usart.h>

extern volatile uint8_t Tx_buffer[BUFF_LEN];
extern volatile uint8_t Rx_buffer[BUFF_LEN];
extern uint32_t UART_RCV_count;
extern volatile int32_t Main_State;

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

void setup_BT_usart(void) {


	gpio_mode_setup(UART_BT_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BT_RX_PIN|BT_TX_PIN);
	gpio_set_output_options(UART_BT_PORT,GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,BT_TX_PIN);
	gpio_set_af(UART_BT_PORT, GPIO_AF7, BT_RX_PIN|BT_TX_PIN); 

	/* Setup USART2 parameters. */
	usart_set_baudrate(UART_BT, 115200);
	usart_set_databits(UART_BT, 8);
	usart_set_stopbits(UART_BT, USART_STOPBITS_1);
	usart_set_mode(UART_BT, USART_MODE_TX_RX);
	usart_set_parity(UART_BT, USART_PARITY_NONE);
	usart_set_flow_control(UART_BT, USART_FLOWCONTROL_NONE);

	usart_enable_rx_dma(UART_BT);

    usart_enable(UART_BT);

	init_uart_dma();

}

/////////////////////////////////////////////////////////////
//DMA STUFF
//
/////////////////////////////////////////////////////////////

/**
 * @brief 
 * 
 */
void init_uart_dma() {
    rcc_periph_clock_enable(DMA_RCC);
	/////////////////////////////////////////////////////////
    // //UART1_TX : DMA 2 Stream 7 Ch 4
    // dma_stream_reset(UART_DMA, TX_DMA_ST);
	// dma_set_priority(UART_DMA, TX_DMA_ST, DMA_SxCR_PL_VERY_HIGH);
	// //UART1 is 8bits
	// dma_set_memory_size(UART_DMA, TX_DMA_ST, DMA_SxCR_MSIZE_8BIT);
	// dma_set_peripheral_size(UART_DMA, TX_DMA_ST, DMA_SxCR_PSIZE_8BIT);
    
	// dma_enable_memory_increment_mode(UART_DMA, TX_DMA_ST);
	// dma_set_transfer_mode(UART_DMA, TX_DMA_ST, 
	// DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	
	// dma_set_peripheral_address(UART_DMA, TX_DMA_ST, (uint32_t) &USART_DR(UART_BT));
    // dma_set_memory_address(UART_DMA, TX_DMA_ST, (uint8_t*)Tx_buffer);

    // dma_enable_memory_increment_mode(UART_DMA, TX_DMA_ST);
    // dma_disable_peripheral_increment_mode(UART_DMA, TX_DMA_ST);

	// dma_channel_select(UART_DMA, TX_DMA_ST, UART_DMA_CH);	
    // dma_enable_transfer_complete_interrupt(UART_DMA, TX_DMA_ST);

    // nvic_set_priority(NVIC_DMA_TX, 0b01000001);
    // nvic_enable_irq(NVIC_DMA_TX);
    

	/////////////////////////////////////////////////////////
	//Uart1_RX DMA 2 Stream 5 Ch 4
	dma_stream_reset(UART_DMA, RX_DMA_ST);
	dma_set_priority(UART_DMA, RX_DMA_ST, DMA_SxCR_PL_HIGH);
	//UART1 is 8bits
	dma_set_memory_size(UART_DMA, RX_DMA_ST, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(UART_DMA, RX_DMA_ST, DMA_SxCR_PSIZE_8BIT);
    
	dma_enable_memory_increment_mode(UART_DMA, RX_DMA_ST);
	dma_set_transfer_mode(UART_DMA, RX_DMA_ST, 
	DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	
	dma_set_peripheral_address(UART_DMA, RX_DMA_ST, (uint32_t) &USART_DR(UART_BT));
    dma_set_memory_address(UART_DMA, RX_DMA_ST, (uint8_t*)Rx_buffer);

    dma_enable_memory_increment_mode(UART_DMA, RX_DMA_ST);
    dma_disable_peripheral_increment_mode(UART_DMA, RX_DMA_ST);

	dma_channel_select(UART_DMA, RX_DMA_ST, UART_DMA_CH);	
    dma_enable_transfer_complete_interrupt(UART_DMA, RX_DMA_ST);
    
    nvic_set_priority(NVIC_DMA_RX, 0b01000001);
    nvic_enable_irq(NVIC_DMA_RX);
    
	
}


/**
 * @brief 
 * 
 * @param n_words 
 */
void start_uart_tx_transfer(uint32_t n_words){
    
	dma_set_number_of_data(UART_DMA, TX_DMA_ST, 4*n_words);
    dma_clear_interrupt_flags(UART_DMA, TX_DMA_ST, DMA_TCIF);
	dma_enable_stream(UART_DMA, TX_DMA_ST);
}

/**
 * @brief 
 * 
 * @param n_bytes usually 5 bytes as of 1.0 ver
 */
void start_uart_rx_reception(uint32_t n_bytes){
    //if problems go out and do somehing

    dma_clear_interrupt_flags(UART_DMA, RX_DMA_ST, DMA_TCIF);
    dma_set_number_of_data(UART_DMA, RX_DMA_ST, n_bytes);
    dma_enable_stream(UART_DMA, RX_DMA_ST);
    
}

/**
 * @brief Called when Tx transfer is done
 * 
 */
void  dma2_stream7_isr(){
	if(dma_get_interrupt_flag(UART_DMA, TX_DMA_ST, DMA_TCIF)){
		dma_clear_interrupt_flags(UART_DMA, TX_DMA_ST, DMA_TCIF);
        
	}
}

/**
 * @brief Called when Rx transfer is done
 * 
 */
void  dma2_stream5_isr(){
	if(dma_get_interrupt_flag(UART_DMA, RX_DMA_ST, DMA_TCIF)){
		dma_clear_interrupt_flags(UART_DMA, RX_DMA_ST, DMA_TCIF);

        Main_State = RX_RCV_STATE;
        UART_RCV_count++;

	}
}

/////////////////////////////////////////////////////////////
/*
void uart_SendCommand_dma (uint16_t command, uint16_t nargs, bool Tx_preloaded, ...)
{   
    dma_busy=true;
    last_command=command;
    va_list args;
    va_start(args, nargs);
    uint32_t* temp_arg;

    Tx7_buffer[0]= WRITE_CMD;
    nargs++; nargs*=4;    
    Tx7_buffer[1]= nargs;
    Tx7_buffer[2]= command;

    nargs/=4;
    if(!Tx_preloaded){
        for(size_t x=1; x<nargs; x++)
        {
            temp_arg = va_arg(args, uint32_t*);
            Tx7_buffer[2+x]= *temp_arg;
        }
    }
    //last index is nargs+1 => nargs+2 words to send
    nargs+=2;
    //Receive header 1st
    receiving_header=true;
    //start the dma transfer
    start_uart_tx_transfer(nargs);
}

*/