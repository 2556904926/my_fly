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
    float az;//���ٶ�
    
    float gx;
    float gy;//������
    float gz;
    
    float pit;
    float rol;//��̬��
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

/* ���Ƚ�ֵ */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 100)    //���PWMŪ�������ģ�һЩ������ͻ�������⣨Ӳ���ϵ�ԭ��

/****************������ų�ʼ��**************/
/* ���ã�ռ�ձȣ� */
#define motor1(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ChannelPulse)    // ���ñȽϼĴ�����ֵ   //����  
#define motor2(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,ChannelPulse)    // ���ñȽϼĴ�����ֵ   //����  
#define motor3(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,ChannelPulse)    // ���ñȽϼĴ�����ֵ   //����    
#define motor4(ChannelPulse)     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,ChannelPulse)    // ���ñȽϼĴ�����ֵ   //���� 

/* ʹ����� */
#define motor1_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);   
#define motor2_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  
#define motor3_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);  
#define motor4_ENABLE()      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);  
/* ������� */
#define motor1_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
#define motor2_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
#define motor3_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
#define motor4_DISABLE()     HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);

#define SPEED_PID_PERIOD  20    //���Ҫ����ʱ��7���ж�����
#define WaitTime_ms   400   //�ȴ�ʱ��
void motor_pwm(uint32_t moto1pwm,uint32_t motor2pwm,uint32_t motor3pwm,uint32_t motor4pwm);
void AngleController(void);//�Ƕ��⻷
void GyroController(falg dd);//���ٶ��ڻ�
uint32_t fbs(float dd);//�޷�����
extern _Attitude att_now;  //��̬����������
extern int rgb_f_count;    //rgb����˸Ƶ��
#endif
