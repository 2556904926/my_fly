#ifndef __PWM3901_H
#define __PWM3901_H
//����������
#include "usart.h"
#define angle_to_rad 0.017453292519943295769236907684886f //�Ƕ�ת����
#define Optical_time 20      //�������ݴ���ʱ��  20ms
#define cpi          0.1063f //����ת��cm
typedef struct Optical_flow{
    int16_t x;
    int16_t y;
    int16_t h;
    uint8_t soual;
    int min_x_i;
    int min_y_i;//ԭʼ����
    int pixel_flow_x_i;
    int pixel_flow_y_i;//�˲�����
    float ang_x;
    float ang_y;//�ǶȲ���
    float out_x_i;
    float out_y_i;//�˲����
    float out_x_i_last;
    float out_y_i_last;//��һ���˲����
    float x_v;
    float y_v;//�ٶ�ֵ
    float x_v_out;
    float y_v_out;//�ٶ�ֵ
    
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