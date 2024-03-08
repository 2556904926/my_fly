#include "motor.h"
#include "stdio.h"
#include "pid.h"
#include "mpu6050.h"
#include "battery.h"
#include "tim.h"
#include "Kalman.h"
#include "Attitude_solution.h"
#include "rgb.h"
_Attitude att_now,att_target;
int rgb_f_count = 10;
//点击四个pwm输出
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    static int tim3_count = 0,tim2_count = 0;
    if(htim==&htim2){//100ms
        tim2_count++;
        if(tim2_count>=rgb_f_count) {
//            RGB_flashing(RGB1); 
//            RGB_flashing(RGB2); 
//            RGB_flashing(RGB3);
        }
    }
    else if(htim==&htim3){//10ms
//        get_mpu6050data(&mpu6050,&Raw_data);
//        GetAngle(&Raw_data,0.01f);
//        wireless_printf_battery("pitch:%.5f\r\n",Raw_data.Angle.pitch);
//        wireless_printf_battery("gx:%.5f\r\n",Raw_data.deg_s.x);
////        wireless_printf_battery("roll:%.5f\r\n",Raw_data.Angle.roll);
////        wireless_printf_battery("yaw:%.5f\r\n",Raw_data.Angle.yaw);
//            float x[3]={Raw_data.Angle.pitch,Raw_data.deg_s.x,0};
//            if(!kalman_filter(&mpu6050_data_poll,x)){
//            att_now.pit = mpu6050_data_poll.x[0];
//            att_now.gx = mpu6050_data_poll.x[1];
//            }
//            wireless_printf_battery("pit:%.5f\r\n",att_now.pit);
//            wireless_printf_battery("pit:%.5f\r\n",att_now.gx);
    }
}
void motor_pwm(uint32_t moto1pwm,uint32_t motor2pwm,uint32_t motor3pwm,uint32_t motor4pwm)
{
	motor1(moto1pwm);
	motor2(motor2pwm);
	motor3(motor3pwm);
	motor4(motor4pwm);
}
/*
MOTOR1 += + pidRateX.out - pidRateY.out -pidRateZ.out;
 
MOTOR2 += - pidRateX.out - pidRateY.out + pidRateZ.out;
 
MOTOR3 += - pidRateX.out + pidRateY.out - pidRateZ.out;
 
MOTOR4 += + pidRateX.out + pidRateY.out + pidRateZ.out;
                        
实现左右横滚运动的时候：（Roll）

需要M1，M4的转速增大。M2，M3电机的转速减少。
实现前后俯仰运动的时候：（Pitch）

需要M3，M4的转速增大。M1，M2电机的转速减少。
实现前后偏航运动的时候：（Yaw）
所以如果我们要飞机向逆时针方向偏航对应到图中需要M2，M4的转速增大。M1，M3电机的转速减少。
飞行姿态控制算法
*/
void AngleController(void)
{
	static uint16_t yaw_init_cnt = 0;               
	pid_compute(Position_type,&allpid.rolAngle,att_now.rol); //姿态解算的roll角作为反馈量  （位移，角度）全用位置用位置式pid           
    pid_compute(Position_type,&allpid.pitAngle,att_now.pit);  //姿态解算的pitch角作为反馈量 //外环pid控制器    
    pid_compute(Position_type,&allpid.yawAngle,att_now.rol);   //姿态解算的pitch角作为反馈量 //外环pid控制器       
}
//pid控制器
void GyroController(falg dd)
{
	int pwm_Base_value=0;
	int pid_out_x=0,pid_out_y=0,pid_out_z=0;
    
    allpid.pitGyro.target_val = allpid.pitAngle.actual_val;   //外环输出作为内环期望值
	allpid.rolGyro.target_val = allpid.rolAngle.actual_val;         //外环输出作为内环期望值
    allpid.rolGyro.target_val = allpid.rolAngle.actual_val;         //外环输出作为内环期望
	pid_out_x=(float)pid_compute(Incremental,&allpid.pitGyro,att_now.gx);
	pid_out_y=(float)pid_compute(Incremental,&allpid.rolGyro,att_now.gy);
	pid_out_z=(float)pid_compute(Incremental,&allpid.yawGyro,att_now.gz);
    
	int motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm;
	motor1_pwm=(pid_out_x-pid_out_y-pid_out_z)+pwm_Base_value;
	motor2_pwm=(-pid_out_x-pid_out_y+pid_out_z)+pwm_Base_value;
	motor3_pwm=(-pid_out_x+pid_out_y-pid_out_z)+pwm_Base_value;
	motor4_pwm=(pid_out_x-pid_out_y-pid_out_z)+pwm_Base_value;
    
	motor_pwm(fbs(motor1_pwm),fbs(motor2_pwm),fbs(motor3_pwm),fbs(motor4_pwm));
}
uint32_t fbs(float dd){
    dd=dd>PWM_MAX_PERIOD_COUNT?PWM_MAX_PERIOD_COUNT:dd;
    dd=dd<PWM_MIN?PWM_MIN:dd;
    return (uint32_t)dd;
}
