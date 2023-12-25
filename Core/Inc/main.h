#ifndef MAIN_H
#define MAIN_H


#define STM32F4



#include "printf.h"
#include <stdio.h>	
#include <string.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencmsis/core_cm3.h>
#include <math.h>
#include <limits.h>
#include <libopencm3/cm3/systick.h>
#include <defines.h>
#include <errno.h>
#include <flash.h>
#include <i2c_oled.h>
#include <exti.h>
#include <adc.h>
#include <dac_i2c.h>

#define LED_GPIO    GPIOB
#define RED_LED     GPIO14
#define GREEN_LED   GPIO0
#define BLUE_LED    GPIO7

typedef struct
{
    uint32_t    flag;
    uint32_t    doTask
}ISR_Handler;

typedef struct
{
    uint32_t    port;
    uint32_t    pin
}gpiopin;

int uart_printf(const char *format,...) __attribute((format(printf,1,2)));

typedef struct
{
    uint32_t    Days;
    uint8_t     Hours;
    uint8_t     Minutes;
    uint8_t     Seconds
}RTC;


#define FLOAT_ERROR 1e-6f   //can be used for float comparison ? if it's not used by the FPU


void delay_ms(uint16_t ms);;
void delay_100us(uint16_t us);	
void stop_us_count(uint32_t *c);
void start_us_count(uint32_t *c);
void hard_fault_handler();
#endif