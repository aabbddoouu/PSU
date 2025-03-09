#include <main.h>

#define DAC_I2C     I2C1
#define DAC_ADDR    0x20>>1

#define IFS0        127000 //in nA
#define IFS1        127000



#define V_MIN_0     12000   //might wanna change the lower FB to get Vout_min = 5
#define V_MIN_1     12000

#define DAC_CH0     0
#define DAC_CH1     1

void dac_setup();
int dac_set_voltage(uint32_t , uint32_t );
uint8_t dac_read_reg(uint8_t);