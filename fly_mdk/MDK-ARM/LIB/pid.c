//采用模糊pid算法
#include "pid.h"
#include "motor.h"
#include "math.h"
#include "stdio.h"
AllPid allpid={0};
//存放pid的初始参数
float pid_parameters[15][5]={
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},//顺序为kp，ki，kd，积分最大，输出最大
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0, 0.0}
};
//pid参数初始化配置
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
	PidInit(&allpid.acc_fix_y,12);//完成初始化
	PidInit(&allpid.vel_fix_y,13);
	PidInit(&allpid.pos_fix_y,14);
}
/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */

/**
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // 设置当前的目标值
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}
float abs_pid(float x){
	if(x>=0) return x;
	else return -x;
}
float Incremental_pid(_pid* pid,float now){//Output(n) = Output(n-1) + Kp * (Error(n) - Error(n-1)) + Ki * Error(n) + Kd * (Error(n) - 2 * Error(n-1) + Error(n-2))
	pid->err_last_last=pid->err_last;
	pid->err_last=pid->err;//更新上次误差和上上次误差
	pid->err=pid->target_val-now;
	pid->err=pid->Ignore_err>abs_pid(pid->err)?0:pid->err;
	pid->actual_val+=pid->Kp*(pid->err-pid->err_last)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_last+pid->err_last_last);
	pid->actual_val=pid->actual_val>pid->out_max?pid->out_max:pid->actual_val;
	pid->actual_val=pid->actual_val>0?pid->actual_val:0;
	return pid->actual_val;//增量式
}

float Position_type_pid(_pid* pid,float now){//Output(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
	pid->err_last=pid->err_last;
	pid->err=pid->target_val-now;
	pid->err=pid->Ignore_err>abs_pid(pid->err)?0:pid->err;//误差忽略
	pid->integral+=pid->err;
	pid->integral=pid->integral_max>pid->integral?pid->integral:pid->integral_max;
	pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
	pid->actual_val=pid->actual_val>pid->out_max?pid->out_max:pid->actual_val;
	pid->actual_val=pid->actual_val>0?pid->actual_val:0;
	return pid->actual_val;//位置式
}
float Fuzzy_pid(_pid* pid,float now){
	//好像其实只用传统pid也够了？这就不写了
    return 0;
	
}

//模糊
//	Incremental,//增量式pid
//	Position_type,//位置式pid
//	Fuzzy//模糊pid
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
