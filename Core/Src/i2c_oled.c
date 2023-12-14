#include <i2c_oled.h>

gpiopin OLED_SCL = {GPIOD, GPIO12};
gpiopin OLED_SDA = {GPIOD, GPIO13};

#define I2C_AF	GPIO_AF4

/**
 * @brief Setup oled comm peripheral
 * 
 */
void oled_setup(){
	RCC_DCKCFGR2|= (RCC_DCKCFGR2_UARTxSEL_HSI<<RCC_DCKCFGR2_I2C4SEL_SHIFT); //set i2c1 clock to HSI = 16MHz
	rcc_periph_clock_enable(RCC_I2C4);
	
	i2c_reset(I2C_OLED);
	gpio_mode_setup(
		OLED_SCL.port,
      	GPIO_MODE_AF,
      	GPIO_PUPD_NONE,
      	OLED_SCL.pin|OLED_SDA.pin		//  SCL|SDA
	);
	gpio_set_output_options(
		OLED_SCL.port,
		GPIO_OTYPE_OD,
		GPIO_OSPEED_25MHZ,
		OLED_SCL.pin|OLED_SDA.pin
	);
	gpio_set_af(OLED_SCL.port,I2C_AF,OLED_SCL.pin|OLED_SDA.pin); //set to AF4 => i2c
	i2c_peripheral_disable(I2C_OLED);
	i2c_set_7bit_addr_mode(I2C_OLED);
	i2c_enable_analog_filter(I2C_OLED);
	i2c_set_digital_filter(I2C_OLED, 0);
	i2c_set_speed(I2C_OLED,i2c_speed_fmp_1m,rcc_get_i2c_clk_freq(I2C_OLED));
	delay_ms(1000);
	i2c_peripheral_enable(I2C_OLED);
}


/**
 * Run a write/read transaction to a given 7bit i2c address
 * If both write & read are provided, the read will use repeated start.
 * Both write and read are optional
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param w buffer of data to write
 * @param wn length of w
 * @param autoend choose whether to end i2c link after xfer or not
 */
void i2c_xfer7(uint32_t i2c, uint8_t addr, uint8_t command, uint8_t *w, size_t wn, bool autoend)
{	
	I2C_ICR(i2c)|=I2C_ICR_NACKCF; //clear NACKF
	i2c_set_7bit_address(i2c, addr);
	i2c_set_write_transfer_dir(i2c);
	i2c_set_bytes_to_transfer(i2c, wn+1);
	if (autoend)
		i2c_enable_autoend(i2c);
	else
		i2c_disable_autoend(i2c);
	
	i2c_send_start(i2c);

	bool wait = true;
		while (wait) {
			if (i2c_transmit_int_status(i2c)) {
				wait = false;
			}
			while (i2c_nack(i2c)); /* Some error */
		}
		i2c_send_data(i2c, command);
	
	while (wn--) {
		wait = true;
		while (wait) {
			if (i2c_transmit_int_status(i2c)) {
				wait = false;
			}
			while (i2c_nack(i2c)); /* Some error */
		}
		i2c_send_data(i2c, *w++);
	}
}


//Write line to oled (primitive)
void update_time_oled(char * str){
		for (size_t i = 0; i < 19; i++)
		{
			ssd1306_Line(0,i,SSD1306_WIDTH-1,i, Black);
		}
		
		ssd1306_SetCursor(0,5);
		ssd1306_WriteString(str, Font_7x10, White);

		
}


void update_status_oled(char * str){
		for (size_t i = 20; i < 34; i++)
		{
			ssd1306_Line(0,i,SSD1306_WIDTH-1,i, Black);
		}

		ssd1306_SetCursor(0,20);
		ssd1306_WriteString(str, Font_7x10, White);

		
}


void update_current_oled(char * str){
		for (size_t i = 35; i < 49; i++)
		{
			ssd1306_Line(0,i,SSD1306_WIDTH-1,i, Black);
		}

		ssd1306_SetCursor(0,35);
		ssd1306_WriteString(str, Font_7x10, White);

		
}


void update_test_oled(char * str){
		for (size_t i = 50; i < 63; i++)
		{
			ssd1306_Line(0,i,SSD1306_WIDTH-1,i, Black);
		}

		ssd1306_SetCursor(0,50);
		ssd1306_WriteString(str, Font_7x10, White);

}


