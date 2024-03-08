#ifndef __PID_H
#define	__PID_H
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
typedef struct
{
    float target_val;           //目标值
    float actual_val;        		//目标值
		float out_max;
    float err;             			//定义偏差值
	  float Ignore_err;         //可以忽略的误差，在范围内则err为0
    float err_last;          		//定义上一个偏差值
	  float err_last_last;        //定义上次俩次的误差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
	  float integral_max;         //最大积分值
}_pid;
typedef struct
{
    //角度环
    _pid pitAngle;
    _pid rolAngle;
    _pid yawAngle;
	
    //角速度环
    _pid pitGyro;      
    _pid rolGyro;
    _pid yawGyro;   
    //高度环
    _pid acc_high;
    _pid vel_high;
    _pid pos_high;
    
    //x轴向环
    _pid acc_fix_x;
    _pid vel_fix_x;
    _pid pos_fix_x;
  
    //y轴方向环
    _pid acc_fix_y;
    _pid vel_fix_y; 
    _pid pos_fix_y;
    //此处的x，y，z为光流模块的x，y，h，并不是于陀螺仪的xyz轴   	
}AllPid;
enum pid_way{
	Incremental=0,//增量式pid
	Position_type,//位置式pid
	Fuzzy//模糊pid
};//pid计算方式
 void PID_param_init(void);//读取pid参数
 void set_pid_target(_pid *pid, float temp_val);//
 float get_pid_target(_pid *pid);
 void set_p_i_d(_pid *pid, float p, float i, float d);

void allpidinit();
float Incremental_pid(_pid* pid,float now);
float Position_type_pid(_pid* pid,float now);
float Fuzzy_pid(_pid* pid,float now);
float pid_compute(enum pid_way res,_pid* pid,float now);//挑选pid计算方式
extern AllPid allpid;
#endif