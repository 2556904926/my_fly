#ifndef __SOLUTION_H
#define __SOLUTION_H
#include "main.h"
#include "mpu6050.h"
//Ò»Î¬¿¨¶ûÂüÂË²¨
typedef struct{
    float NewP;
    float LastP;
    float Kg;
    float Out;
    float Q;
    float R;
}KalmanFilter;
void kalmanfiter(KalmanFilter *EKF,float input);
#define squa( Sq )        (((float)Sq)*((float)Sq))
void imu_rest(void);
void GetAngle(Mpu6050_Data* Raw_data, float dt);
#endif