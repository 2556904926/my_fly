#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "usart.h"
#include "tim.h"
#include "main.h"
typedef struct 
{
    float ax;
    float ay;
    float az;//加速度
    
    float gx;
    float gy;//陀螺仪
    float gz;
    
    float pit;
    float rol;//姿态角
    float yaw;
}_Attitude;
typedef enum{
    Off = 0,
    Height_determination = 1,
    hover = 2,
    Flight_control = 3,
}falg;

#define PWM_PERIOD_COUNT     (5000)  
#define PWM_MIN               (0)

/* 最大比较值 */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 100)    //如果PWM弄成了满的，一些驱动板就会出现问题（硬件上的原因）

/****************电机引脚初始化**************/
/* 设置（占空比） */
#define motor1(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ChannelPulse)    // 设置比较寄存器的值   //左上  
#define motor2(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,ChannelPulse)    // 设置比较寄存器的值   //右上  
#define motor3(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,ChannelPulse)    // 设置比较寄存器的值   //坐下    
#define motor4(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,ChannelPulse)    // 设置比较寄存器的值   //右下 

/* 使能输出 */
#define motor1_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);   
#define motor2_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  
#define motor3_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);  
#define motor4_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);  
/* 禁用输出 */
#define motor1_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
#define motor2_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
#define motor3_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
#define motor4_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);

#define SPEED_PID_PERIOD  20    //这个要看定时器7的中断周期
#define WaitTime_ms   400   //等待时间
void motor_pwm(uint32_t moto1pwm,uint32_t motor2pwm,uint32_t motor3pwm,uint32_t motor4pwm);
void AngleController(void);//角度外环
void GyroController(falg dd);//角速度内环
uint32_t fbs(float dd);//限幅函数
extern _Attitude att_now;  //姿态传感器数据
extern int rgb_f_count;    //rgb灯闪烁频率
#endif
