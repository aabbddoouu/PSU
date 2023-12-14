#include <main.h>

#define AUTOEND true    //true = stop i2c link at the end of i2c_xfer7()
#define I2C_OLED I2C4


void update_time_oled(char*);
void update_status_oled(char*);
void update_current_oled(char*);
void update_test_oled(char*);
void i2c_xfer7(uint32_t i2c, uint8_t addr, uint8_t command, uint8_t *w, size_t wn, bool autoend);
void oled_setup();

