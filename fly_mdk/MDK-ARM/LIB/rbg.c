#include "rgb.h"
typedef struct {
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
}rgb;
rgb color[8]= {
{1, 1, 1},
{1, 1, 0},  // Red
{1, 0, 1},  // Green
{0, 1, 1},  // Blue
{1, 0, 0},  // Yellow
{0, 0, 1},  // Cyan
{0, 1, 0},  // purple
{0, 0, 0},  // White
};

/**
 * @brief RGB灯选择
 *
 * 该函数控制3个RGB灯
 * @param RGBX RGB灯号
 * @param res 颜色
 * @return 无
 */
void rgb_choose(uint8_t RGBX,uint8_t res){
	switch(RGBX){
		case RGB1:
		HAL_GPIO_WritePin(RGB1_GPIO1,RGB1_GPIO1_PIN,(GPIO_PinState)color[res].Red);
		HAL_GPIO_WritePin(RGB1_GPIO2,RGB1_GPIO2_PIN,(GPIO_PinState)color[res].Green);
		HAL_GPIO_WritePin(RGB1_GPIO3,RGB1_GPIO3_PIN,(GPIO_PinState)color[res].Blue);
		break;
		case RGB2:
		HAL_GPIO_WritePin(RGB2_GPIO1,RGB2_GPIO1_PIN,(GPIO_PinState)color[res].Red);
		HAL_GPIO_WritePin(RGB2_GPIO2,RGB2_GPIO2_PIN,(GPIO_PinState)color[res].Green);
		HAL_GPIO_WritePin(RGB2_GPIO3,RGB2_GPIO3_PIN,(GPIO_PinState)color[res].Blue);
		break;
		case RGB3:
		HAL_GPIO_WritePin(RGB3_GPIO1,RGB3_GPIO1_PIN,(GPIO_PinState)color[res].Red);
		HAL_GPIO_WritePin(RGB3_GPIO2,RGB3_GPIO2_PIN,(GPIO_PinState)color[res].Green);
		HAL_GPIO_WritePin(RGB3_GPIO3,RGB3_GPIO3_PIN,(GPIO_PinState)color[res].Blue);
		break;
		default:
		break;
	}
}
/**
 * @brief 
 *
 * 该函数控制RGB灯闪烁
 *
 * @param RGBX RGB灯号
 * @param res 颜色
 * @param res 开关写0，则为开
 * @return 无
 */
void RGB_flashing(uint8_t RGBX){
    static uint8_t res0=0,res1=0,res2=0,res;
    if(RGBX==RGB1) {res0++,res=res0;}
    else if(RGBX==RGB2) {res1++,res=res1;}
    else {res2++,res=res2;}
	res=res%8;
	rgb_choose(RGBX,res);
	res++;
}
