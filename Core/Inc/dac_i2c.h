#include <main.h>

#define DAC_I2C     I2C1
#define DAC_ADDR    0x20>>1

#define IFS0        180160
#define IFS1        77470

#define V_MIN_0     4000
#define V_MIN_1     9000

#define DAC_CH0     0
#define DAC_CH1     1

void dac_setup();
uint8_t dac_set_voltage(uint32_t , uint32_t );
uint8_t dac_read_reg(uint8_t);