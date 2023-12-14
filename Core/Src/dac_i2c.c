#include <dac_i2c.h>

gpiopin DAC_SCL = {GPIOB, GPIO8};
gpiopin DAC_SDA = {GPIOB, GPIO9};

static uint8_t* Voltage_regs[2] = {0, 0};

/**
 * @brief Setup DAC comm peripheral
 * 
 */
void dac_setup(){
	//RCC_DCKCFGR2|= (RCC_DCKCFGR2_UARTxSEL_HSI<<RCC_DCKCFGR2_I2C4SEL_SHIFT); //set i2c1 clock to HSI = 16MHz

	rcc_periph_clock_enable(RCC_I2C1);
	i2c_set_clock_frequency(DAC_I2C, 16);
	rcc_periph_reset_pulse(RST_I2C1);


	gpio_mode_setup(
		DAC_SCL.port,
      	GPIO_MODE_AF,
      	GPIO_PUPD_NONE,
      	DAC_SCL.pin|DAC_SDA.pin		//  SCL|SDA
	);
	gpio_set_output_options(
		DAC_SCL.port,
		GPIO_OTYPE_OD,
		GPIO_OSPEED_25MHZ,
		DAC_SCL.pin|DAC_SDA.pin
	);
	gpio_set_af(DAC_SCL.port,GPIO_AF4,DAC_SCL.pin|DAC_SDA.pin); //set to AF4 => i2c
	i2c_peripheral_disable(DAC_I2C);

	i2c_set_speed(DAC_I2C,i2c_speed_sm_100k,rcc_get_i2c_clk_freq(DAC_I2C));
	delay_ms(1000);
	i2c_peripheral_enable(DAC_I2C);

}

/**
 * @brief Sets the desired voltage on channel 0 or 1
 * 
 * @param channel 		OUT 0 or 1
 * @param Voltage_mV 	Voltage in mV
 */
uint8_t dac_set_voltage(uint32_t channel, uint32_t Voltage_mV){
	
	uint32_t 	I_uA=0;
	uint32_t 	buff32=0x7F;
	uint8_t 	I_7bits=0;

	//Convert Voltage to µA

	switch (channel)
	{
	//Channel 0 : Vout = 4V + 0.1*Is0
	case 0:
		if (Voltage_mV<V_MIN_0)
		{
			return 1;
		}
		I_uA=(Voltage_mV-V_MIN_0)*10;
		buff32*=I_uA/IFS0;

		Voltage_regs[0]=0xF8;

		break;

	//Channel 1 : Vout = 9V + 0.1*Is1
	case 1:
		if (Voltage_mV<V_MIN_1)
		{
			return 1;
		}
		I_uA=(Voltage_mV-V_MIN_1)*10;
		buff32*=I_uA/IFS1;

		Voltage_regs[0]=0xF9;

		break;
	default:
		return 2;

	}
	
	//Convert µA to DAC bits
	if (buff32>0x7F)
	{
		buff32=0x7F;
	}
	
	I_7bits=(uint8_t) buff32 & (0x7F);

	Voltage_regs[1]=I_7bits;

	//i2c_transfer7(DAC_I2C, DAC_ADDR, Voltage_regs, 2, (void*)0x0, 0);

	uart_printf("Current is %d uA \n", I_uA/1000);

	return 0;
}
