#ifndef _BATTERY_h_
#define _BATTERY_h_
#include "stm32f4xx_hal.h"
void battery_init();
uint32_t get_val();
extern uint32_t ADC_Value;
void wireless_printf_battery(const char* format, ...);
void printArray(uint8_t* arr,int len);

#endif