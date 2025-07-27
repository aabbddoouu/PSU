#include <i2c_oled.h>

extern volatile uint32_t I2C_XFER;



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

	int status=0;
	I2C_XFER=1;
	timer_one_shot_mode(TIM2);
	TIM_ARR(TIM2) = TIMEOUT_I2C;
	TIM_EGR(TIM2) = TIM_EGR_UG;
	TIM_CR1(TIM2) |= TIM_CR1_CEN;


	i2c_peripheral_disable(i2c);
	i2c_peripheral_enable(i2c);

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
	
	i2c_send_data(i2c, command);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF))){
		if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
	}
	
	while (wn--) {

		i2c_send_data(i2c, *w++);
		while (!(I2C_SR1(i2c) & (I2C_SR1_BTF))){
			if((TIM_CR1(TIM2) & TIM_CR1_CEN) ==0)	{return -1;}
		}
	}

	i2c_send_stop(i2c);

	I2C_XFER=0;
	TIM_ARR(TIM2) = 0;
	TIM_CR1(TIM2) &= ~TIM_CR1_CEN;
}

int i2c_read7_oled(uint32_t i2c, int addr, uint8_t *res, size_t n)
{


	return;
}




int i2c_transfer_oled(uint32_t i2c, uint8_t addr, uint8_t command, uint8_t *w, size_t wn, uint8_t *r, size_t rn) {

	int status=0;
	I2C_XFER=1;
	timer_one_shot_mode(TIM2);
	TIM_ARR(TIM2) = TIMEOUT_I2C;
	TIM_EGR(TIM2) = TIM_EGR_UG;
	TIM_CR1(TIM2) |= TIM_CR1_CEN;

	status=i2c_write7_oled(i2c, addr, w, wn);

	if (wn) {
		status=i2c_write7_oled(i2c, addr, w, wn);
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

int i2c_write7_oled(uint32_t i2c, int addr, const uint8_t *data, size_t n)
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


