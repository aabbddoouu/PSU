#include <usart_pc.h>
#include <usart.h>
#include <dma.h>

extern volatile uint8_t Tx_buffer[BUFF_LEN];
extern volatile uint8_t Rx_buffer[BUFF_LEN];
extern uint32_t UART_RCV_count;
extern volatile int32_t Main_State;

/**
 * @brief Setup PC usart (USART2) with DMA
 * 
 */
void setup_PC_usart(void) {

	gpio_mode_setup(UART_PC_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN|TX_PIN);
	gpio_set_output_options(UART_PC_PORT,GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,TX_PIN);
	gpio_set_af(UART_PC_PORT, GPIO_AF7, RX_PIN|TX_PIN); 

	/* Setup USART2 parameters. */
	usart_set_baudrate(UART_PC, 115200);
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
    rcc_periph_clock_enable(RCC_DMA1);
	/////////////////////////////////////////////////////////
    //UART7_TX : DMA 1 Stream 6 Ch 4
    dma_stream_reset(UART_DMA, TX_DMA_ST);
	dma_set_priority(UART_DMA, TX_DMA_ST, DMA_SxCR_PL_VERY_HIGH);
	//UART7 is 8bits
	dma_set_memory_size(UART_DMA, TX_DMA_ST, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(UART_DMA, TX_DMA_ST, DMA_SxCR_PSIZE_8BIT);
    
	dma_enable_memory_increment_mode(UART_DMA, TX_DMA_ST);
	dma_set_transfer_mode(UART_DMA, TX_DMA_ST, 
	DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	
	dma_set_peripheral_address(UART_DMA, TX_DMA_ST, (uint32_t) &USART_DR(UART_PC));
    dma_set_memory_address(UART_DMA, TX_DMA_ST, (uint8_t*)Tx_buffer);

    dma_enable_memory_increment_mode(UART_DMA, TX_DMA_ST);
    dma_disable_peripheral_increment_mode(UART_DMA, TX_DMA_ST);

	dma_channel_select(UART_DMA, TX_DMA_ST, DMA_SxCR_CHSEL_4);	
    dma_enable_transfer_complete_interrupt(UART_DMA, TX_DMA_ST);

    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0b01000001);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    

	/////////////////////////////////////////////////////////
	//Uart7_RX DMA 1 Stream 5 Ch 4
	dma_stream_reset(UART_DMA, RX_DMA_ST);
	dma_set_priority(UART_DMA, RX_DMA_ST, DMA_SxCR_PL_HIGH);
	//UART7 is 8bits
	dma_set_memory_size(UART_DMA, RX_DMA_ST, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(UART_DMA, RX_DMA_ST, DMA_SxCR_PSIZE_8BIT);
    
	dma_enable_memory_increment_mode(UART_DMA, RX_DMA_ST);
	dma_set_transfer_mode(UART_DMA, RX_DMA_ST, 
	DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	
	dma_set_peripheral_address(UART_DMA, RX_DMA_ST, (uint32_t) &USART_DR(UART_PC));
    dma_set_memory_address(UART_DMA, RX_DMA_ST, (uint8_t*)Rx_buffer);

    dma_enable_memory_increment_mode(UART_DMA, RX_DMA_ST);
    dma_disable_peripheral_increment_mode(UART_DMA, RX_DMA_ST);

	dma_channel_select(UART_DMA, RX_DMA_ST, DMA_SxCR_CHSEL_4);	
    dma_enable_transfer_complete_interrupt(UART_DMA, RX_DMA_ST);
    
    nvic_set_priority(NVIC_DMA1_STREAM5_IRQ, 0b01000001);
    nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
    
	
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
 * @param n_bytes usually 4 bytes as of 1.0 ver
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
void  dma1_stream6_isr(){
	if(dma_get_interrupt_flag(UART_DMA, TX_DMA_ST, DMA_TCIF)){
		dma_clear_interrupt_flags(UART_DMA, TX_DMA_ST, DMA_TCIF);
        
	}
}

/**
 * @brief Called when Rx transfer is done
 * 
 */
void  dma1_stream5_isr(){
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