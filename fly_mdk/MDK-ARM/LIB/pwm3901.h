#ifndef __PWM3901_H
#define __PWM3901_H
//光流传感器
#include "usart.h"
#define angle_to_rad 0.017453292519943295769236907684886f //角度转弧度
#define Optical_time 20      //光流数据处理时间  20ms
#define cpi          0.1063f //数据转化cm
typedef struct Optical_flow{
    int16_t x;
    int16_t y;
    int16_t h;
    uint8_t soual;
    int min_x_i;
    int min_y_i;//原始积分
    int pixel_flow_x_i;
    int pixel_flow_y_i;//滤波积分
    float ang_x;
    float ang_y;//角度补偿
    float out_x_i;
    float out_y_i;//滤波输出
    float out_x_i_last;
    float out_y_i_last;//上一次滤波输出
    float x_v;
    float y_v;//速度值
    float x_v_out;
    float y_v_out;//速度值
    
    float x_v_cm;
    float y_v_cm;
    float x_i_out_cm;
    float y_i_out_cm;
    float h_cm;
}Optical_flow;
extern uint8_t uart3_rx_buff[11];
extern Optical_flow Optical_flow1;
extern uint8_t uart3_rx_data;
void optocal_flow_data(uint8_t *uart3_rx_buff);
void mergeArrays(uint8_t* arr1, int m, uint8_t* arr2, int n, uint8_t* arr3) ;

void pmw3901_init();
#endif