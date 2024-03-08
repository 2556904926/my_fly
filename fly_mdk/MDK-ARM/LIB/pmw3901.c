#include "pwm3901.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "rgb.h"
#include "nrf24l01.h"
#include "battery.h"
#include "motor.h"
uint8_t uart3_rx_buff[11]={0};
uint8_t uart3_rx_data=0;
Optical_flow Optical_flow1 = {
    .x = 0,
    .y = 0,
    .h = 0,
    .soual = 0,
    .min_x_i = 0,
    .min_y_i = 0,
    .pixel_flow_x_i = 0,
    .pixel_flow_y_i = 0,
    .ang_x = 0.0f,
    .ang_y = 0.0f,
    .out_x_i = 0.0f,
    .out_y_i = 0.0f,
    .out_x_i_last = 0.0f,
    .out_y_i_last = 0.0f,
    .x_v = 0.0f,
    .x_v_out = 0.0f,
    .y_v = 0.0f,
    .y_v_out = 0.0f,
    .h_cm = 0.0f,
    .x_v_cm = 0.0f,
    .y_v_cm = 0.0f,
    .x_i_out_cm = 0.0f,
    .y_i_out_cm = 0.0f
};

void pmw3901_init(){
    HAL_UART_Receive_DMA(&huart3,uart3_rx_buff,11);
}

uint8_t* Look_up_data(uint8_t* data,int data_len,uint8_t look_data){
    for(int i=0;i<data_len;++i){
        if(data[i]==look_data){
            return (&data[i]);
        }
    }
    return NULL;
}
void Optical_flow_data_analysis(uint8_t* uart3_rx_buff){
    static int count = 0;
    if((0xFE!=uart3_rx_buff[0])||(0x04!=uart3_rx_buff[1])||(0xAA!=uart3_rx_buff[10])) return;
    uint8_t data=0;
    for(int i=0;i<6;i++){
        data+=uart3_rx_buff[2+i];
    }
    if(data!=uart3_rx_buff[8]) return;
    Optical_flow1.x=((int16_t)uart3_rx_buff[3]<<8)|uart3_rx_buff[2];
    Optical_flow1.y=((int16_t)uart3_rx_buff[5]<<8)|uart3_rx_buff[4];
    Optical_flow1.h=((int16_t)uart3_rx_buff[7]<<8)|uart3_rx_buff[6];
    Optical_flow1.soual=uart3_rx_buff[9];
    
    //积分
    Optical_flow1.min_x_i+=Optical_flow1.x;
    Optical_flow1.min_y_i+=Optical_flow1.y;
    //低通滤波
    Optical_flow1.pixel_flow_x_i+=0.2*(Optical_flow1.min_x_i-Optical_flow1.pixel_flow_x_i);
    Optical_flow1.pixel_flow_y_i+=0.2*(Optical_flow1.min_y_i-Optical_flow1.pixel_flow_y_i);
    //姿态角去补偿积分位移
    Optical_flow1.ang_x+=(600.0f * tan(att_now.rol*angle_to_rad)-Optical_flow1.ang_x) * 0.2;
    Optical_flow1.ang_y+=(600.0f * tan(att_now.pit*angle_to_rad)-Optical_flow1.ang_y) * 0.2;
    Optical_flow1.out_x_i=Optical_flow1.pixel_flow_x_i - Optical_flow1.ang_x;
    Optical_flow1.out_y_i=Optical_flow1.pixel_flow_y_i - Optical_flow1.ang_y;
    
    //求微分速度
    Optical_flow1.x_v=(Optical_flow1.out_x_i-Optical_flow1.out_x_i_last)/(Optical_time*0.001f);
    Optical_flow1.y_v=(Optical_flow1.out_y_i-Optical_flow1.out_y_i_last)/(Optical_time*0.001f);
    Optical_flow1.out_x_i_last=Optical_flow1.out_x_i; //更新上一次的位移
    Optical_flow1.out_y_i_last=Optical_flow1.out_y_i;
    //低通滤波
    Optical_flow1.x_v_out+=(Optical_flow1.x_v - Optical_flow1.x_v_out) *0.1f;
    Optical_flow1.y_v_out+=(Optical_flow1.y_v - Optical_flow1.y_v_out) *0.1f;
    //cm转化
    Optical_flow1.h_cm=Optical_flow1.h * cpi;
    Optical_flow1.x_v_cm=Optical_flow1.x_v_out * cpi;
    Optical_flow1.y_v_cm=Optical_flow1.y_v_out * cpi;
    Optical_flow1.x_i_out_cm=Optical_flow1.out_x_i * cpi;
    Optical_flow1.y_i_out_cm=Optical_flow1.out_y_i * cpi;
    //数据处理结束
    
    if(25 > uart3_rx_buff[9]) count++;
    else count =0;
    if(count>20)  wireless_printf_battery("环境质量差",0);
}
////式中 HIGH 为实际高度，单位：米
//cpi = ((50*0.01f) / 11.914f) *2.54f ;
//pixel_flow.fix_High=cpi;
////积分位移值单位转换为：厘米
//pixel_flow.loc_x = pixel_flow.out_x_i * cpi;
//pixel_flow.loc_y = pixel_flow.out_y_i * cpi;
////微分速度值单位转换为：厘米/秒
//pixel_flow.loc_xs = pixel_flow.fix_x * cpi;
//pixel_flow.loc_ys = pixel_flow.fix_y * cpi;
void optocal_flow_data(uint8_t *uart3_rx_buff){
    static uint8_t count=0,count_copy;
    uint8_t copy[11];
    if(0==count){
        uint8_t *ptr = Look_up_data(uart3_rx_buff,11,0xAA);
        if(NULL!=ptr) {
        count_copy = ptr - uart3_rx_buff+1;
        ++count;
        }
        else{
            return;
        }
    }
    for(int i=0;i<11;++i){
        int c=i+count_copy;
        c = (i + count_copy) % 11;
        copy[i]=uart3_rx_buff[c];
    }
    Optical_flow_data_analysis(copy);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart3)
    {
//		HAL_UART_Transmit(&huart1,uart3_rx_buff,sizeof(uart3_rx_buff),200);
//        HAL_UART_Receive_IT(&huart3,&uart3_rx_data,1);
//        RGB_flashing(RGB1);
    }
    
}
void mergeArrays(uint8_t* arr1, int m, uint8_t* arr2, int n, uint8_t* arr3) {
    memcpy(arr3, arr1, m * sizeof(uint8_t));
    memcpy(arr3 + m, arr2, n * sizeof(uint8_t));
}