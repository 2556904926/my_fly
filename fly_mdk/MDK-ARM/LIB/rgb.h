#ifndef __RBG_H
#define __RBG_H
#include "stm32f4xx.h"
#include "gpio.h"
#include <time.h>


#define RGB1 0x01
#define RGB2 0x02
#define RGB3 0X03

#define RGB1_GPIO1 GPIOD
#define RGB1_GPIO2 GPIOD
#define RGB1_GPIO3 GPIOD

#define RGB1_GPIO1_PIN GPIO_PIN_0
#define RGB1_GPIO2_PIN GPIO_PIN_1
#define RGB1_GPIO3_PIN GPIO_PIN_2


#define RGB2_GPIO1 GPIOC
#define RGB2_GPIO2 GPIOC
#define RGB2_GPIO3 GPIOB

#define RGB2_GPIO1_PIN GPIO_PIN_4
#define RGB2_GPIO2_PIN GPIO_PIN_5
#define RGB2_GPIO3_PIN GPIO_PIN_0

#define RGB3_GPIO1 GPIOD
#define RGB3_GPIO2 GPIOD
#define RGB3_GPIO3 GPIOD

#define RGB3_GPIO1_PIN GPIO_PIN_7
#define RGB3_GPIO2_PIN GPIO_PIN_6
#define RGB3_GPIO3_PIN GPIO_PIN_5

#define off   0x00 //Ö±½Ó¹Ø±ÕrgbµÆ
#define red   0x01
#define green 0x02
#define blue  0x03
#define tellow 0x04
#define cyan 0x05
#define purple 0x06
#define white 0x07
void rgb_choose(uint8_t RGBX,uint8_t res);
void RGB_flashing(uint8_t RGBX);
#endif
