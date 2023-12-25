#include <main.h>
#define BUTTON_DELAY 300

volatile uint64_t ticks=0;
volatile uint64_t button_ticks=0;	
volatile uint64_t led_ticks=0;

volatile ISR_Handler Calib_ISR ={0, 0};

gpiopin EN_OUT1 = {GPIOB, GPIO10};
gpiopin EN_OUT2 = {GPIOB, GPIO4};

gpiopin EN_DCDC = {GPIOA, GPIO8};

gpiopin EN_CALIB1 = {GPIOB, GPIO5};
gpiopin EN_CALIB2 = {GPIOB, GPIO3};
gpiopin BUTTON_CALIB = {GPIOB, GPIO0};
gpiopin ADC_CALIB = {GPIOA, GPIO0};
gpiopin LED_CALIB = {GPIOC, GPIO7};

/////////////////////////////////////////////////////
//// UART printf SECTION
/////////////////////////////////////////////////////

// used to counter a compiling error (neded for str manipulation for debugging)
//void *_sbrk(int incr) { return (void *)-1; }

void _putchar(char character)
{
	// send char to UART
	usart_send_blocking(USART2,character);  //USART3 is connected to ST-link serial com
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

	/* Enable clocks for USART2. */
	rcc_periph_clock_enable(RCC_USART2);

	rcc_periph_clock_enable(RCC_ADC1);
}

static void usart_setup(void)
{
		// UART Section
	/* Setup GPIO pins for USART2 transmit and receive. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO2);
	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO5 on GPIO port A for LED. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	//DCDC-INTEGRATED Controll
	gpio_mode_setup(EN_DCDC.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN_DCDC.pin);
	gpio_set_output_options(EN_DCDC.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, EN_DCDC.pin);
	gpio_set(EN_DCDC.port, EN_DCDC.pin);
	delay_ms(1000);


	//Output Relays Controll
	gpio_mode_setup(EN_OUT1.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN_OUT1.pin|EN_OUT2.pin);
	gpio_set_output_options(EN_OUT1.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, EN_OUT1.pin|EN_OUT2.pin);
	gpio_clear(EN_OUT1.port, EN_OUT1.pin|EN_OUT2.pin);
	delay_ms(1000);


/*
	//CALIB
	//OUT
	gpio_mode_setup(EN_CALIB1.port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, EN_CALIB1.pin|EN_CALIB2.pin);
	gpio_set_output_options(EN_CALIB1.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, EN_CALIB1.pin|EN_CALIB2.pin);
	
	gpio_clear(EN_CALIB1.port, EN_CALIB1.pin);
	delay_ms(1000);
	gpio_clear(EN_CALIB1.port, EN_CALIB2.pin);
	delay_ms(1000);
*/
	

	//LED
	gpio_mode_setup(LED_CALIB.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_CALIB.pin);
	gpio_clear(LED_CALIB.port, LED_CALIB.pin);
	//Button
	gpio_mode_setup(BUTTON_CALIB.port, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, BUTTON_CALIB.pin);
	exti_select_source(BUTTON_CALIB.pin, BUTTON_CALIB.port);
	exti_set_trigger(BUTTON_CALIB.pin, EXTI_TRIGGER_RISING);
	nvic_enable_irq(NVIC_EXTI0_IRQ); //no need to assert the priority
	//exti_enable_request(BUTTON_CALIB.pin);
	//Analog Read
	gpio_mode_setup(ADC_CALIB.port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, ADC_CALIB.pin);

	adc_power_off(ADC1);

	adc_set_single_conversion_mode(ADC1);
	uint8_t channel_array[] = {ADC_CHANNEL0};
	adc_disable_scan_mode(ADC1);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_480CYC);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	
	adc_power_on(ADC1);
	
}

void exti0_isr(){ //Calib Button EXTI
	//TODO
	if(ticks-button_ticks>=BUTTON_DELAY){
		Calib_ISR.flag++;
	}
	button_ticks=ticks;
	exti_reset_request(BUTTON_CALIB.pin);
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

int main(void)
{
	int i, j;
	
	
	clock_setup();

	delay_setup();
	
	usart_setup();
	uart_printf("\n******************\n");
	uart_printf("\t Booting Up...\n");
	uart_printf("\n******************\n");

	dac_setup();

	gpio_setup();
	
	
	gpio_clear(GPIOA, GPIO5);

	uint32_t delay = 1e7;
	
	
	
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t adc_out = adc_read_regular(ADC1);
	


	//dac_setup();

	asm("NOP");
	
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


	//gpio_clear(EN_DCDC.port, EN_DCDC.pin);

	//enable timer5 after setup is done (for safety)
	timer_enable_irq(TIM5,TIM_DIER_UIE);
	timer_enable_counter(TIM5);
	
	gpio_toggle(GPIOA, GPIO5);
	delay_ms(1000);
	gpio_toggle(GPIOA, GPIO5);

	while (1) {

		
		if(ticks-led_ticks>1000){	//used to visually check if the code is not frozen somewhere (trap loop, exception, periph failure ...)
			gpio_toggle(GPIOA, GPIO5);
			gpio_toggle(LED_CALIB.port, LED_CALIB.pin);
			led_ticks=ticks;
			//uart_printf("Blinking with delay \n");

			if(Calib_ISR.doTask){
				if(gpio_get(BUTTON_CALIB.port, BUTTON_CALIB.pin)){
					gpio_toggle(LED_CALIB.port, LED_CALIB.pin);
					Calib_ISR.doTask=0;Calib_ISR.flag=0;
				}
			}

			if(usart_recv(USART2)){
				uart_printf("Ok\n");
			}
		}

	}
	

	return 0;
}

void hard_fault_handler(){
	//Request SW reset
	SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
}
