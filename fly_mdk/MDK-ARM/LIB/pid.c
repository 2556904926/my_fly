//����ģ��pid�㷨
#include "pid.h"
#include "motor.h"
#include "math.h"
#include "stdio.h"
AllPid allpid={0};
//���pid�ĳ�ʼ����
float pid_parameters[15][5]={
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},//˳��Ϊkp��ki��kd���������������
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0}
};
//pid������ʼ������
void PidInit(_pid *controller,uint8_t label)
{
    controller->Kp  = pid_parameters[label][0];
    controller->Ki = pid_parameters[label][1];
    controller->Kd = pid_parameters[label][2];
    controller->integral = pid_parameters[label][3];
    controller->integral_max = pid_parameters[label][4];      
}
void allpidinit(){
	PidInit(&allpid.pitAngle,0);
	PidInit(&allpid.rolAngle,1);
	PidInit(&allpid.yawAngle,2);
	PidInit(&allpid.pitGyro,3);
	PidInit(&allpid.rolGyro,4);
	PidInit(&allpid.yawGyro,5);
	PidInit(&allpid.acc_high,6);
	PidInit(&allpid.vel_high,7);
	PidInit(&allpid.pos_high,8);
	PidInit(&allpid.acc_fix_x,9);
	PidInit(&allpid.vel_fix_x,10);
	PidInit(&allpid.pos_fix_x,11);
	PidInit(&allpid.acc_fix_y,12);//��ɳ�ʼ��
	PidInit(&allpid.vel_fix_y,13);
	PidInit(&allpid.pos_fix_y,14);
}
/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */

/**
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // ���õ�ǰ��Ŀ��ֵ
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // ���ñ���ϵ�� P
		pid->Ki = i;    // ���û���ϵ�� I
		pid->Kd = d;    // ����΢��ϵ�� D
}
float abs_pid(float x){
	if(x>=0) return x;
	else return -x;
}
float Incremental_pid(_pid* pid,float now){//Output(n) = Output(n-1) + Kp * (Error(n) - Error(n-1)) + Ki * Error(n) + Kd * (Error(n) - 2 * Error(n-1) + Error(n-2))
	pid->err_last_last=pid->err_last;
	pid->err_last=pid->err;//�����ϴ��������ϴ����
	pid->err=pid->target_val-now;
	pid->err=pid->Ignore_err>abs_pid(pid->err)?0:pid->err;
	pid->actual_val+=pid->Kp*(pid->err-pid->err_last)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_last+pid->err_last_last);
	pid->actual_val=pid->actual_val>pid->out_max?pid->out_max:pid->actual_val;
	pid->actual_val=pid->actual_val>0?pid->actual_val:0;
	return pid->actual_val;//����ʽ
}

float Position_type_pid(_pid* pid,float now){//Output(t) = Kp * e(t) + Ki * ��e(t)dt + Kd * de(t)/dt
	pid->err_last=pid->err_last;
	pid->err=pid->target_val-now;
	pid->err=pid->Ignore_err>abs_pid(pid->err)?0:pid->err;//������
	pid->integral+=pid->err;
	pid->integral=pid->integral_max>pid->integral?pid->integral:pid->integral_max;
	pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
	pid->actual_val=pid->actual_val>pid->out_max?pid->out_max:pid->actual_val;
	pid->actual_val=pid->actual_val>0?pid->actual_val:0;
	return pid->actual_val;//λ��ʽ
}
float Fuzzy_pid(_pid* pid,float now){
	//������ʵֻ�ô�ͳpidҲ���ˣ���Ͳ�д��
    return 0;
	
}

//ģ��
//	Incremental,//����ʽpid
//	Position_type,//λ��ʽpid
//	Fuzzy//ģ��pid
float pid_compute(enum pid_way res,_pid* pid,float now){
	float (*way)(_pid* pid,float x);
	switch(res){
		case Incremental:
			way=Incremental_pid;
		 break;
		case Position_type:
			way=Position_type_pid;
		break;
		default:
			way=Fuzzy_pid;
		break;
	}
	return (*way)(pid,now);
}
