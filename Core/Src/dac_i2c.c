#include <dac_i2c.h>

extern volatile uint32_t I2C_XFER;

gpiopin DAC_SCL = {GPIOB, GPIO8};
gpiopin DAC_SDA = {GPIOB, GPIO9};

uint8_t* Voltage_regs[3] = {(uint8_t)0, (uint8_t)0, (uint8_t)0xFF};


int i2c_read7_v2(uint32_t i2c, int addr, uint8_t *res, size_t n)
{

	int status=0;

	i2c_send_start(i2c);
	i2c_enable_ack(i2c);

	/* Wait for the end of the start condition, master mode selected, and BUSY bit set */
	while ( !( (I2C_SR1(i2c) & I2C_SR1_SB)
		&& (I2C_SR2(i2c) & I2C_SR2_MSL)
		&& (I2C_SR2(i2c) & I2C_SR2_BUSY) )){
			if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
		}

	i2c_send_7bit_address(i2c, addr, I2C_READ);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)){
		if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
	}
	/* Clearing ADDR condition sequence. */
	(void)I2C_SR2(i2c);

	for (size_t i = 0; i < n; ++i) {
		if (i == n - 1) {
			i2c_disable_ack(i2c);
			//i2c_nack_next(i2c);
			//i2c_nack_current(i2c);
		}
		while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)){
			if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
		}
		res[i] = i2c_get_data(i2c);
	}
	//i2c_nack_current(i2c);
	

	return;
}




int i2c_transfer2(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn) {

	int status=0;
	I2C_XFER=1;
	timer_one_shot_mode(TIM2);
	TIM_ARR(TIM2) = TIMEOUT_I2C;
	TIM_EGR(TIM2) = TIM_EGR_UG;
	TIM_CR1(TIM2) |= TIM_CR1_CEN;


	if (wn) {
		status=i2c_write7_v2(i2c, addr, w, wn);
		if(status==-1)	{TIM_CR1(TIM2) &= ~TIM_CR1_CEN; I2C_XFER=0; return -1;}
	}
	if (rn) {
		status=i2c_read7_v2(i2c, addr, r, rn);
		if(status==-1)	{TIM_CR1(TIM2) &= ~TIM_CR1_CEN; I2C_XFER=0; return -1;}
	} else {
		i2c_send_stop(i2c);
	}

	I2C_XFER=0;
	TIM_ARR(TIM2) = 0;
	TIM_CR1(TIM2) &= ~TIM_CR1_CEN;

}

int i2c_write7_v2(uint32_t i2c, int addr, const uint8_t *data, size_t n)
{

	while ((I2C_SR2(i2c) & I2C_SR2_BUSY)) {
		if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
	}

	i2c_send_start(i2c);

	/* Wait for the end of the start condition, master mode selected, and BUSY bit set */
	while ( !( (I2C_SR1(i2c) & I2C_SR1_SB)
		&& (I2C_SR2(i2c) & I2C_SR2_MSL)
		&& (I2C_SR2(i2c) & I2C_SR2_BUSY) )){
			if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
		}

	i2c_send_7bit_address(i2c, addr, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)){
		if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
	}

	/* Clearing ADDR condition sequence. */
	(void)I2C_SR2(i2c);

	for (size_t i = 0; i < 100; i++){asm("NOP");}
	

	for (size_t i = 0; i < n; i++) {
		i2c_send_data(i2c, data[4*i]);
		while (!(I2C_SR1(i2c) & (I2C_SR1_BTF))){
			if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
		}
		for (size_t i = 0; i < 100; i++){asm("NOP");}

	}
}

/**
 * @brief Setup DAC comm peripheral
 * 
 */
void dac_setup(){
	//RCC_DCKCFGR2|= (RCC_DCKCFGR2_UARTxSEL_HSI<<RCC_DCKCFGR2_I2C4SEL_SHIFT); //set i2c1 clock to HSI = 16MHz
	


SWRST:
	rcc_periph_clock_enable(RCC_I2C1);
	i2c_set_clock_frequency(DAC_I2C, 48);
	
	//SW reset the I2C bus
	rcc_periph_reset_pulse(RST_I2C1);
	I2C_CR1(DAC_I2C) |= I2C_CR1_SWRST; 
	delay_ms(1);
	I2C_CR1(DAC_I2C) &= (~I2C_CR1_SWRST); 
	//

	delay_ms(2000);

	gpio_mode_setup(
		DAC_SCL.port,
      	GPIO_MODE_AF,
      	GPIO_PUPD_PULLUP,
      	DAC_SCL.pin|DAC_SDA.pin		//  SCL|SDA
	);

	gpio_set_output_options(
		DAC_SCL.port,
		GPIO_OTYPE_OD,
		GPIO_OSPEED_25MHZ,
		DAC_SCL.pin|DAC_SDA.pin
	);

	delay_ms(2000);


	gpio_set_af(DAC_SCL.port,GPIO_AF4,DAC_SCL.pin|DAC_SDA.pin); //set to AF4 => i2c
	
	i2c_peripheral_disable(DAC_I2C);
	i2c_set_speed(DAC_I2C,i2c_speed_fm_400k,42);
	
	delay_ms(1000);
	
	i2c_peripheral_enable(DAC_I2C);

	

	if((I2C1_SR2 & I2C_SR2_BUSY)){
		//RESET MCU
		uart_printf("\nBusy Bug on I2C bus. Restarting...\n");
		gpio_toggle(GPIOA, GPIO5);
		delay_ms(100); 
		gpio_toggle(GPIOA, GPIO5);
		delay_ms(100);
		//hard_fault_handler();
		goto SWRST;
	}

/*
	i2c_send_stop(DAC_I2C);
	delay_ms(10);
	i2c_send_start(DAC_I2C);
	delay_ms(10);
	i2c_send_7bit_address(DAC_I2C, 0x00, I2C_WRITE);
	delay_ms(10);
*/

	i2c_peripheral_enable(DAC_I2C);

	uart_printf("\nBusy Bug didn't occur PagMan !\n");
}

/**
 * @brief Sets the desired voltage on channel 0 or 1
 * 
 * @param channel 		OUT 0 or 1
 * @param Voltage_mV 	Voltage in mV
 */
int dac_set_voltage(uint32_t channel, uint32_t Voltage_mV){
	
	int status=0;

	uint32_t 	I_uA=0;
	uint32_t 	buff32=0;
	uint8_t 	I_7bits=0;

	//Convert Voltage to µA

	switch (channel)
	{
	//Channel 0 : Vout = 12V + 0.1*Is0
	case 0:
		if (Voltage_mV<V_MIN_0)
		{
			uart_printf("Voltage Out of Bound\n");
			return 1;
		}
		I_uA=(Voltage_mV-V_MIN_0)*10;
		buff32=I_uA*0x7F; // scaling error
		buff32/=IFS0;


		Voltage_regs[0]=0xF8;

		break;

	//Channel 1 : Vout = 12V + 0.1*Is1
	case 1:
		if (Voltage_mV<V_MIN_1)
		{
			uart_printf("Voltage Out of Bound\n");
			return 1;
		}
		I_uA=(Voltage_mV-V_MIN_1)*10;
		buff32=I_uA*0x7F;
		buff32/=IFS1;

		Voltage_regs[0]=0xF9;

		break;
	default:
		uart_printf("Channel ID KO\n");
		return 2;

	}
	
	//Convert µA to DAC bits
	if (buff32>0x7F)
	{
		buff32=0x7F;
	}
	
	I_7bits=(uint8_t) buff32 & (0x7F);

	Voltage_regs[1]=I_7bits;


	status=i2c_transfer2(DAC_I2C, DAC_ADDR, Voltage_regs, 2, NULL, 0);
	i2c_send_stop(DAC_I2C); //not sure if needed

	if(status==-1)	{
		uart_printf("ERROR : I2C Timeout !\n");
		return -1;
	}



	uart_printf("Current is %d uA - Sending %x to Reg: %x\n", I_uA/1000, Voltage_regs[1], Voltage_regs[0]);

	return 0;
}



uint8_t dac_read_reg(uint8_t reg){
	uint8_t* reg_data[2]; 	
	uint8_t* reg_t[1];

	reg_data[0]=0xFF;
	reg_t[0]=reg;

	i2c_transfer2(DAC_I2C, DAC_ADDR, reg_t, 1, reg_data, 1);
	//i2c_transfer2(DAC_I2C, DAC_ADDR, reg_t, 1, NULL, 0);
	//delay_ms(1);
	//i2c_read7_v2(DAC_I2C, DAC_ADDR, reg_data, 1);
	
	i2c_send_stop(DAC_I2C);

	//i2c_send_start(DAC_I2C);

	//i2c_send_7bit_address(DAC_I2C, 0x00, I2C_WRITE);

	uart_printf("Reg %x => %x\n", reg, reg_data[0]);

	return reg_data[0];

}