#ifndef  __KALMAN_H
#define  __KALMAN_H

#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#define mpu6050_dt 0.01 //50ms
#define MM2M 		(0.001f)
#define DEG2RAD 	(0.017453292519943295769236907684886f)	// 角度转弧度
#define RAD2DEG 	(57.295779513082320876798154814105f)	// 弧度转角度

#define N 3
typedef struct kalman_coefficient{
    float F[N][N];  // 状态转移矩阵          n*n
    float B[N];   //控制矩阵                 n*1
    float u;    //控制向量                  常数
    float w[N];    //噪声均值
    float H[N][N];  // 观测矩阵              n*n
    float Q[N][N];  // 状态噪声协方差矩阵    n*n
    float R;    // 观测噪声协方差矩阵       常数
    float P[N][N];  // 状态估计协方差矩阵    n*n
    float x[N];   // 状态估计值              n*1
    float Kg[N][N]; //                       n*1
} kalman_coefficient;//n代表了传感器的参数量，也就是一个状态量的长度
typedef struct{
    float *buff;
    int count;
    int count_num;
    int n;
}Sliding_window;
#define sq(a)	((a) * (a))
#define	sqrtf4(a,b,c,d)	__sqrtf(sq(a) + sq(b) + sq(c) + sq(d))
#define sqrtf3(a,b,c)	__sqrtf(sq(a) + sq(b) + sq(c))
#define	sqrtf2(a,b)	__sqrtf(sq(a) + sq(b))
unsigned char inverseMatrix(float matrix[][N]);
unsigned char kalman_filter(kalman_coefficient* data,float* z_k);

//滑动窗口滤波
u8 Sliding_window_filtering(Sliding_window* res,float data);
extern kalman_coefficient mpu6050_data_poll;
#endif
