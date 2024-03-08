#include "Attitude_solution.h"
#include "mpu6050.h"
#include "nrf24l01.h"
#include "battery.h"
#include "Kalman.h"
#include <math.h>
void kalmanfiter(KalmanFilter *EKF,float input)
{
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
}
 
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f*2;	  	//陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2		
const float Gyro_Gr = 0.0005326f*2;     //面计算度每秒,转换弧度每秒则 2*0.03051756	 * 0.0174533f = 0.0005326*2
 
static float NormAcc;
 
/* 四元数系数 */
typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;
Quaternion NumQ = {1, 0, 0, 0};
 
/* 陀螺仪积分误差 */
struct V{
	float x;
	float y;
	float z;
};	
volatile struct V GyroIntegError = {0};
 
/* 四元数解法初始化 */
void imu_rest(void)
{
	NumQ.q0 =1;
	NumQ.q1 = 0;
	NumQ.q2 = 0;
	NumQ.q3 = 0;	
	GyroIntegError.x = 0;
	GyroIntegError.y = 0;
	GyroIntegError.z = 0;
}
 
void GetAngle(Mpu6050_Data* Raw_data, float dt) 
{		
	volatile struct V Gravity,Acc,Gyro,AccGravity;
 
	static  float KpDef = 0.5f ;
	static  float KiDef = 0.0001f;
//static  float KiDef = 0.00001f;
	
	float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;
 
	//提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// 加速度归一化
	//printf("accX:%d\r\n",MPU6050.accX);
	NormAcc = 1/sqrt(squa(Raw_data->acc_g.x)+ squa(Raw_data->acc_g.y) +squa(Raw_data->acc_g.z));
  Acc.x = Raw_data->acc_g.x * NormAcc;
  Acc.y = Raw_data->acc_g.y * NormAcc;
  Acc.z = Raw_data->acc_g.z * NormAcc;
	
 	//向量差乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	
	//再做加速度积分补偿角速度的补偿值
  GyroIntegError.x += AccGravity.x * KiDef;
  GyroIntegError.y += AccGravity.y * KiDef;
  GyroIntegError.z += AccGravity.z * KiDef;
	
	//角速度融合加速度积分补偿值
  Gyro.x = Raw_data->deg_s.x * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
  Gyro.y = Raw_data->deg_s.y * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
  Gyro.z = Raw_data->deg_s.z * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;
	
	// 一阶龙格库塔法, 更新四元数
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = 1/sqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;
	
	// 四元数转欧拉角
	{
		 
			#ifdef	YAW_GYRO
			*(
		float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				float yaw_G = Raw_data->deg_s.z * Gyro_G;
				if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //数据太小可以认为是干扰，不是偏航动作
				{
					Raw_data->Angle.yaw  += yaw_G * dt;
//					wireless_printf_battery("Yaw:%f\r\n",Raw_data->Angle.yaw );
				}
			#endif
			Raw_data->Angle.pitch   =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;						
		
			Raw_data->Angle.roll	= atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;	//PITCH 			
//			wireless_printf_battery("Pitch:%f;\r\n",Raw_data->Angle.pitch);
//			wireless_printf_battery("Roll:%f;\r\n",Raw_data->Angle.roll);
	}
}