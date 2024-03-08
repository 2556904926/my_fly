#ifndef __PID_H
#define	__PID_H
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
typedef struct
{
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//Ŀ��ֵ
		float out_max;
    float err;             			//����ƫ��ֵ
	  float Ignore_err;         //���Ժ��Ե����ڷ�Χ����errΪ0
    float err_last;          		//������һ��ƫ��ֵ
	  float err_last_last;        //�����ϴ����ε����ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
	  float integral_max;         //������ֵ
}_pid;
typedef struct
{
    //�ǶȻ�
    _pid pitAngle;
    _pid rolAngle;
    _pid yawAngle;
	
    //���ٶȻ�
    _pid pitGyro;      
    _pid rolGyro;
    _pid yawGyro;   
    //�߶Ȼ�
    _pid acc_high;
    _pid vel_high;
    _pid pos_high;
    
    //x����
    _pid acc_fix_x;
    _pid vel_fix_x;
    _pid pos_fix_x;
  
    //y�᷽��
    _pid acc_fix_y;
    _pid vel_fix_y; 
    _pid pos_fix_y;
    //�˴���x��y��zΪ����ģ���x��y��h���������������ǵ�xyz��   	
}AllPid;
enum pid_way{
	Incremental=0,//����ʽpid
	Position_type,//λ��ʽpid
	Fuzzy//ģ��pid
};//pid���㷽ʽ
 void PID_param_init(void);//��ȡpid����
 void set_pid_target(_pid *pid, float temp_val);//
 float get_pid_target(_pid *pid);
 void set_p_i_d(_pid *pid, float p, float i, float d);

void allpidinit();
float Incremental_pid(_pid* pid,float now);
float Position_type_pid(_pid* pid,float now);
float Fuzzy_pid(_pid* pid,float now);
float pid_compute(enum pid_way res,_pid* pid,float now);//��ѡpid���㷽ʽ
extern AllPid allpid;
#endif