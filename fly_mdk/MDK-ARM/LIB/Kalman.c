//--------------------实现加速度计和陀螺仪数据融合计算角度
#include "Kalman.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#define Q_angle  0.001		//角度数据置信度，角度噪声的协方差
#define Q_gyro   0.003		//角速度漂移数据置信度，角速度噪声的协方差
#define Q_deg_s   0.002      //角速度协方差
kalman_coefficient mpu6050_data_poll={
    //陀螺仪数据
    .F={{1,mpu6050_dt,-mpu6050_dt},
        {0,         1,          0},
        {0,         0,          1}},
    .x={0},
    .u=0,
    .w={0},
    .B={0},
    .P={{1,0,0},{0,1,0},{0,0,1}},
    .Q={{Q_angle,0,0},{0,Q_deg_s,0},{0,0,Q_gyro}}
};
// 求逆矩阵
unsigned char inverseMatrix(float matrix[][N]) {
    double temp;
    double** inverse=(double**)malloc(N*sizeof(double*));
    for(int i=0;i<N;++i){
        inverse[i]=(double*)malloc(N*sizeof(double));
    }
    // 创建单位矩阵作为初始的逆矩
    for(int i=0;i<N;++i){
        for(int j=0;j<N;++j){
            if(i==j) inverse[i][j] = 1;
            else inverse[i][j] = 0;
        }
    }
//得到逆矩阵
    for (int i = 0; i < N; i++) {
        if (0 == matrix[i][i]) return 1;
        temp = matrix[i][i];
        for (int j = 0; j < N; j++) {
            matrix[i][j] /= temp;
            inverse[i][j] /= temp;
        }
        for (int k = 0; k < N; k++) {
            if (k != i) {
                temp = matrix[k][i];
                for (int j = 0; j < N; j++) {
                    matrix[k][j] -= matrix[i][j] * temp;
                    inverse[k][j] -= inverse[i][j] * temp;
                }
            }
        }
    }
    //将逆矩阵保存在原矩阵矩阵中返回
    for(int i=0;i<N;++i){
        for(int j=0;j<N;++j){
            matrix[i][j]=(float)inverse[i][j];
        }
        free(inverse[i]);
    }
    
    free(inverse);
    return 0;
}
unsigned char kalman_filter(kalman_coefficient* data,float* z_k){
    int i,j,k;
    int n=N;//减少访问时间
    float x_k[N],x_k1[N],x_k2[N];
    //预测
	/*
		X(k|k-1)=FX(k-1|k-1)+BU(k)+w(k)
    */
    for(i=0;i<n;++i){
        x_k[i]=0;
        for(j=0;i<n;++i){
            x_k[i]+=data->F[i][j]*data->x[j];//data->x[j]没更新就是上一次的值
        }
        x_k[i]+=data->B[i]*data->u+data->w[i];
    }
    /*
		P(k|k-1)=FP(k-1|k-1)F'+Q 			//P(k|k-1)再次为res1
	*/
    float res[N][N],res1[N][N],res2[N][N];
    for(i=0;i<n;++i){
        for(j=0;j<n;++j){
            res[i][j]=0;
            for(k=0;k<n;++k){
                res[i][j]+=data->P[i][k]*data->F[j][k];
            }
        }
    }

    for( i = 0; i < n; ++i){
        for( j = 0; j < n; ++j){
            res1[i][j] = 0;
            for( k = 0; k < n; ++k){
                res1[i][j]+=data->F[i][k]*res[k][j];
            }
            res1[i][j]+=data->Q[i][j];
        }
    }
    //更新 
	/*
    
		Kg(k)= P(k|k-1)H'/(HP(k|k-1)H'+R) 
	*/
    for(i=0;i<n;++i){
        for(j=0;j<n;++j){
            res2[i][j]=0;
            for(k=0;k<n;++k){
                res2[i][j]+=res1[i][k]*data->H[j][k];
            }
        }
    }
    for(i=0;i<n;++i){
        for(j=0;j<n;++j){
            res[i][j]=0;
            for(k=0;k<n;++k){
                res[i][j]+=data->H[i][k]*res2[k][j]+data->R;
            }
        }
    }
    if(!inverseMatrix(res)){
        for(i=0;i<n;++i){
        for(j=0;j<n;++j){
            data->Kg[i][j]=0;
            for(k=0;k<n;++k){
                data->Kg[i][j]+=res2[i][k]*res[k][j];
            }
        }
    }

	/*
	
    	X(k|k)= X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1))
	*/
    for(i=0;i<n;++i){
        x_k1[i] = 0;
        x_k2[i] = 0;
        for(j=0;j<n;++j){
            x_k1[i]+=data->H[i][j]*x_k[j];
        }
        x_k2[i]=z_k[i]-x_k1[i];
    }
    for(i=0;i<n;++i){
        data->x[i] = 0;
        for(j=0;j<n;++j){
            data->x[i]=data->Kg[i][j]*x_k2[j];
        }
        data->x[i]+=x_k[i];
    }
	/*
		P(k|k)=(I-Kg(k)H)P(k|k-1)
			|1 0|
		I = |	| 所以根据单位矩阵的性质：P(k|k)=P(k|k-1)-Kg(k)*H*P(k|k-1)
			|0 1|
	*/
    for(i=0;i<n;++i){
        for(j=0;j<n;++j){
            res[i][j]=0;
            for(k=0;k<n;++k){
                res[i][j]+=data->Kg[i][k]*data->H[k][j];
            }
        }
    }

    for(i=0;i<n;++i){
        for(j=0;j<n;++j){
            res2[i][j]=0;
            for(k=0;k<n;++k){
                res2[i][j]+=res[i][k]*res1[k][j];
            }
            data->P[i][j]=res1[i][j]-res2[i][j];
        }
    }
    return 0;
    }
    return 1;
}
//滑动窗口滤波
#define buff_len 10
float high_buff[buff_len]={0};
Sliding_window high_data={
    .buff=high_buff,
    .count=0,
    .n=buff_len,
    .count_num = 0
};
u8 Sliding_window_filtering(Sliding_window* res,float data){
    if(res==NULL) return 1;
    if(res->count<res->n){
        res->buff[res->count] = data;
        res->count_num+=data;
        res->count++;
        return 2;
    }
    else{
        res->count_num=res->count_num-res->buff[0]+data;
        for(int i=0;i<res->n-1;++i){
            res->buff[i] =res->buff[i+1];
        }
        res->buff[res->n-1] = data;
        return 0;
    }
}



