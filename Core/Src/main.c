#include <main.h>
#define BUTTON_DELAY 300

volatile uint64_t ticks=0;
volatile uint64_t button_ticks=0;	
volatile uint64_t led_ticks=0;
volatile uint64_t comm_ticks=0;
volatile uint32_t index_comm=1;

volatile ISR_Handler Calib_ISR ={0, 0};

volatile enum Current_Source {I_in, I_1, I_2};
volatile uint16_t Current_Table[3];

gpiopin EN_OUT1 	= {GPIOB, GPIO12};
gpiopin EN_OUT2 	= {GPIOB, GPIO4};

gpiopin EN_CALIB1 	= {GPIOB, GPIO5};
gpiopin EN_CALIB2 	= {GPIOB, GPIO3};

gpiopin IN_CALIB2 	= {GPIOA, GPIO0};
gpiopin IN_CALIB1 	= {GPIOA, GPIO1};

gpiopin IN_Iin 		= {GPIOA, GPIO4};
gpiopin IN_Iout1 	= {GPIOA, GPIO5};
gpiopin IN_Iout2 	= {GPIOA, GPIO6};

gpiopin LED 		= {LED_GPIO, BLUE_LED};

PSU PSU1 			= {DAC_CH0, V_MIN_0};
PSU PSU2 			= {DAC_CH1, V_MIN_1};
PSU PSU_I2C = {0, 0};

volatile uint8_t Tx_buffer[BUFF_LEN];
volatile uint8_t Rx_buffer[BUFF_LEN];

volatile int32_t Main_State = DEFAULT_STATE;

volatile uint32_t UART_RCV_count = 0;

/////////////////////////////////////////////////////
//// UART printf SECTION
/////////////////////////////////////////////////////

// used to counter a compiling error (neded for str manipulation for debugging)
//void *_sbrk(int incr) { return (void *)-1; }

void _putchar(char character)
{
	// send char to UART
	usart_send_blocking(UART_PC,character);  //USART3 is connected to ST-link serial com
}
int uart_printf(const char *format,...) {

	va_list va;
	va_start(va, format);
	const int ret = vprintf_(format, va);
	va_end(va);
	return ret;

}


// ////////////////////////////////////////////////////////////////////////////////////////////


static void clock_setup(void)
{
	//USE HSI
	rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_96MHZ]);

	//USE HSE
	//rcc_clock_setup_pll(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_96MHZ]);

	/* Enable GPIOD clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_SYSCFG);

	rcc_periph_clock_enable(UART_PC_RCC);
	rcc_periph_clock_enable(RCC_ADC1);
}


static void gpio_setup(void)
{

	//Output Relays Controll
	gpio_mode_setup(EN_OUT1.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN_OUT1.pin|EN_OUT2.pin);
	gpio_set_output_options(EN_OUT1.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, EN_OUT1.pin|EN_OUT2.pin);
	gpio_clear(EN_OUT1.port, EN_OUT1.pin|EN_OUT2.pin);
	delay_ms(10);


/*
	//CALIB
	//OUT
	gpio_mode_setup(EN_CALIB1.port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, EN_CALIB1.pin|EN_CALIB2.pin);
	gpio_set_output_options(EN_CALIB1.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, EN_CALIB1.pin|EN_CALIB2.pin);
	
	gpio_clear(EN_CALIB1.port, EN_CALIB1.pin);
	delay_ms(10);
	gpio_clear(EN_CALIB1.port, EN_CALIB2.pin);
	delay_ms(10);
*/
	
	//LED
	gpio_mode_setup(LED.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED.pin);
	gpio_set_output_options(LED.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, LED.pin);
	gpio_clear(LED.port, LED.pin);

	//Analog Read
	gpio_mode_setup(IN_CALIB1.port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, IN_CALIB1.pin|IN_CALIB2.pin|
	IN_Iin.pin|IN_Iout1.pin|IN_Iout2.pin);



	adc_power_off(ADC1);

	adc_set_single_conversion_mode(ADC1);
	uint8_t channel_array[] = {ADC_CHANNEL4, ADC_CHANNEL5, ADC_CHANNEL6};
	adc_enable_scan_mode(ADC1);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_480CYC);
	adc_set_regular_sequence(ADC1, 3, channel_array);
	adc_enable_overrun_interrupt(ADC1);
	nvic_enable_irq(NVIC_ADC_IRQ);
	

	//DMA Config
	adc_enable_dma(ADC1);
	adc_set_dma_terminate(ADC1); //no DMA re-request after EOC

	rcc_periph_clock_enable(DMA_RCC);
	/////////////////////////////////////////////////////////
    //ADC1_DR : DMA 2 Stream 0 Ch 0
    dma_stream_reset(ADC_DMA, ADC_DMA_ST);
	dma_set_priority(ADC_DMA, ADC_DMA_ST, DMA_SxCR_PL_MEDIUM);
	//ADC_DR is 16bits
	dma_set_memory_size(ADC_DMA, ADC_DMA_ST, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(ADC_DMA, ADC_DMA_ST, DMA_SxCR_PSIZE_16BIT);
    
	dma_enable_memory_increment_mode(ADC_DMA, ADC_DMA_ST);
	dma_set_transfer_mode(ADC_DMA, ADC_DMA_ST, 
	DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	
	dma_set_peripheral_address(ADC_DMA, ADC_DMA_ST, (uint32_t) &ADC1_DR);
    dma_set_memory_address(ADC_DMA, ADC_DMA_ST, (uint16_t*)Current_Table);

    dma_enable_memory_increment_mode(ADC_DMA, ADC_DMA_ST);
    dma_disable_peripheral_increment_mode(ADC_DMA, ADC_DMA_ST);

	dma_channel_select(ADC_DMA, ADC_DMA_ST, ADC_DMA_CH);	
    dma_enable_transfer_complete_interrupt(ADC_DMA, ADC_DMA_ST);

	nvic_set_priority(NVIC_DMA_ADC, 0b01100000);
    nvic_enable_irq(NVIC_DMA_ADC);

	adc_power_on(ADC1);
	
}

void dma2_stream0_isr(){
	adc_disable_dma(ADC1);
	dma_clear_interrupt_flags(ADC_DMA, ADC_DMA_ST, DMA_TCIF);
}

void adc_isr(){ //check if it's adc or adc1
	if(ADC1_SR & ADC_SR_OVR){ // deal with an OVR problem
		// ??
	}
}

static inline void start_dma_adc_convertion(){
	adc_enable_dma(ADC1);
	
	dma_set_number_of_data(ADC_DMA, ADC_DMA_ST, 3*2);
    dma_clear_interrupt_flags(ADC_DMA, ADC_DMA_ST, DMA_TCIF);

	dma_enable_stream(UART_DMA, TX_DMA_ST);

}

void delay_setup(void)
{
	/* set up a millisecond free running timer for delay operations used during the main setup */
	rcc_periph_clock_enable(RCC_TIM2);
	/* millisecond counter */
	timer_set_prescaler(TIM2, 10800); //TIMx are clocked by 2*apbx ; increment each 500us
	timer_set_period(TIM2, 0xFFFF); //load with highest value
	timer_one_shot_mode(TIM2);
	
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM5);
	/* microsecond counter */
	timer_set_prescaler(TIM5, 96); //TIMx are clocked by 2*apbx ; increment each 1us
	timer_set_period(TIM5,1000); //OF each ms	
	timer_continuous_mode(TIM5);
	//timer_update_on_overflow(TIM5);
	nvic_set_priority(NVIC_TIM5_IRQ, 0x10);
	nvic_enable_irq(NVIC_TIM5_IRQ);
	
	//timer_enable_irq(TIM5,TIM_DIER_UIE);
	//timer_enable_counter(TIM5);
	//timer_update_on_overflow(TIM5);

	

}



void tim5_isr(){	//happens every time timer7 overflows
	timer_clear_flag(TIM5,TIM_SR_UIF);
 	//if(timer_get_flag(TIM5,TIM_SR_UIF)){ //Dont need it since it's the only src of the interrupt
	ticks++;

	//1Hz
	if(ticks%1000==0){
		if(Calib_ISR.flag){
			Calib_ISR.doTask=1;
		}

	}
		
}

/**
 * @brief 
 * Blocking delay using Timer 2 in oneshot mode.
 * Dont use delay after the main setup, unless you're debuging
 * @param ms no more than 1<<16
 */
void delay_ms(uint16_t ms)	
{
	TIM_ARR(TIM2) = 10*ms-1;
	TIM_EGR(TIM2) = TIM_EGR_UG;
	timer_enable_counter(TIM2);
	while (TIM_CR1(TIM2) & TIM_CR1_CEN);
}

/**
 * @brief 
 * Blocking delay using Timer 2 in oneshot mode.
 * Dont use delay after the main setup, unless you're debuging
 * @param ms no more than 1<<16
 */
void delay_100us(uint16_t us)	
{
	TIM_ARR(TIM2) = us;
	TIM_EGR(TIM2) = TIM_EGR_UG;
	timer_enable_counter(TIM2);
	while (TIM_CR1(TIM2) & TIM_CR1_CEN);
}


static inline void Await_RX_RCV_Async(){
	
	start_uart_rx_reception(4);
}

int main(void)
{
	int i, j;
	
	
	clock_setup();

	delay_setup();
	
	setup_PC_usart();
	uart_printf("\n******************\n");
	uart_printf("Booting Up...\n");
	uart_printf("\n******************\n");

	dac_setup();

	gpio_setup();
	
	
	gpio_clear(LED_GPIO, BLUE_LED);

	uint32_t delay = 1e7;
	
	

	start_dma_adc_convertion();

	asm("NOP");
	
	/*
	dac_set_voltage(DAC_CH0, 100000);
	delay_100us(1);
	//dac_read_reg(0xF8);	
	delay_100us(1);
	dac_set_voltage(DAC_CH1, 100000);
	delay_100us(1);

	while(1);

	delay_ms(2000);

	dac_set_voltage(DAC_CH0, 8000);
	delay_100us(1);
	dac_set_voltage(DAC_CH1, 15000);
	delay_100us(1);

	delay_ms(2000);

	dac_set_voltage(DAC_CH0, 6000);
	delay_100us(1);
	//dac_read_reg(0xF8);	
	delay_100us(1);
	dac_set_voltage(DAC_CH1, 12000);
	delay_100us(1);

	delay_ms(2000);

	dac_set_voltage(DAC_CH0, 4000);
	delay_100us(1);
	dac_set_voltage(DAC_CH1, 9000);
	delay_100us(1);
	*/

	//gpio_clear(EN_DCDC.port, EN_DCDC.pin);

	//enable timer5 after setup is done (for safety)
	timer_enable_irq(TIM5,TIM_DIER_UIE);
	timer_enable_counter(TIM5);

	
	Await_RX_RCV_Async();

	uint32_t Conv_mV=0;

	while (1) {

		switch (Main_State)
		{
		case DEFAULT_STATE:

			if(ticks-led_ticks>1000){	//used to visually check if the code is not frozen somewhere (trap loop, exception, periph failure ...)
				gpio_toggle(LED_GPIO, BLUE_LED);
				led_ticks=ticks;
				//uart_printf("Count : %d\n", UART_RCV_count);
			
			}



			break;

		case RX_RCV_STATE:
			// Enter when RX DMA has finished the read transaction
			// Process the data and go to SEND_DAC_STATE
			
			// Structure of the MSG :             
			//		1st byte : PSU 1 or 2 
            //      2nd byte : 2nd digit of voltage
            //      3rd byte : 1st digit of voltage
            //      4th byte : 1st devimal digit
			
			// HINT : '0' in ASCII is 0x30 in unit8_t/char (instead of using atoi)

			//gpio_toggle(GPIOA, GPIO5);

			PSU_I2C.Voltage_mV = (Rx_buffer[3]-0x30)*100 + (Rx_buffer[1]-0x30)*10000 + (Rx_buffer[2]-0x30)*1000;

			switch (Rx_buffer[0])
			{
			case '2':
				PSU_I2C.Channel=PSU1.Channel;
				Main_State = SEND_DAC_STATE;
				break;

			case '1':
				PSU_I2C.Channel=PSU2.Channel;
				Main_State = SEND_DAC_STATE;
				break;
			
			default:
				Main_State=TX_SEND_STATE;
				break;
			}

			
			break;

		case SEND_DAC_STATE:
			// Sends the wanted Voltage to the µA DAC
			uart_printf("RCVD : %d mV\n",PSU_I2C.Voltage_mV);
			dac_set_voltage(PSU_I2C.Channel, PSU_I2C.Voltage_mV);
			Main_State = TX_SEND_STATE;
			break;

		case TX_SEND_STATE:
			// Sends an OK (ACK) to the PC after I2C comm is OK
			Await_RX_RCV_Async();
			Main_State = DEFAULT_STATE;
			break;

		case SEND_CURRENT:
			// Sends Current values to the 2nd STM32
			// dma_method to send 3x2 bytes

		
		case ERR_I2C_STATE:
			// Check if NACK Err or Busy Err
			// Not yet implemented 
			break;

		default:
			break;
		}




	}
	

	return 0;
}

void hard_fault_handler(){
	//Request SW reset
	SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
}
