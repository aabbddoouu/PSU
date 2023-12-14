#include <main.h>

#define DAC_I2C     I2C1
#define DAC_ADDR    0x20

#define IFS0        18016
#define IFS1        7747

#define V_MIN_0     4000
#define V_MIN_1     9000

void dac_setup();
uint8_t dac_set_voltage(uint32_t , uint32_t );