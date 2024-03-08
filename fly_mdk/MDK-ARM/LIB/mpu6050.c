#include "mpu6050.h"
#include "i2c.h"
#include "main.h"
#include "inv_mpu.h"
#include "motor.h"
#include "battery.h"
#include "Attitude_solution.h"
#include "rgb.h"
#define g 9.80665 //�������ٶ�
#define dbs_max 2000 //���ٶȶ�ÿ�� 
mpu6050_init mpu6050;
Mpu6050_Data Raw_data={
    .Angle={0,0,0}
};//ԭʼ���ݺ��˲�����

uint8_t mpu6050_w_len(uint8_t address,uint8_t reg,uint8_t len,uint8_t* data){
    return HAL_I2C_Mem_Write(&hi2c1,(address<<1)|0,reg,1,data,len,1000);
}
uint8_t mpu6050_r_len(uint8_t address,uint8_t reg,uint8_t len,uint8_t* pdata){
    return HAL_I2C_Mem_Read(&hi2c1,(address<<1)|0,reg,1,pdata,len,1000);
}
int16_t Sensor_I2C2_Serch(void){
	for(uint8_t i = 0; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1,100) == HAL_OK)
		{
			return i;
		}
	}
	return 0xff;
}
void mpu6050_delay_ms(int ms){
    HAL_Delay(ms);
    return;
}
void mpu6050_disposition(mpu6050_init * dd,mpu6050_tx tx,mpu6050_rx rx,delay ms){
    dd->tx = tx;
    dd->rx = rx;
    dd->delay_ms = ms;
}
//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU6050_Init(mpu6050_init*dd)
{
    mpu6050_disposition(dd,mpu6050_w_len,mpu6050_r_len,mpu6050_delay_ms);
	u8 res;
	MPU6050_Write_Byte(dd,MPU6050_PWR_MGMT1_REG,0X80);	//��λMPU6050
    dd->delay_ms(200);
	MPU6050_Write_Byte(dd,MPU6050_PWR_MGMT1_REG,0X00);	//����MPU6050
    res = MPU6050_Read_Byte(dd,MPU6050_PWR_MGMT1_REG);
    wireless_printf_battery("MPU6050_PWR_MGMT1_REG:%d \r\n",res);
	MPU6050_Set_Gyro_Fsr(dd,3);					//�����Ǵ�����,��2000dps
	MPU6050_Set_Accel_Fsr(dd,0);					//���ٶȴ�����,��2g
	MPU6050_Set_Rate(dd,200);						//���ò�����50Hz
	MPU6050_Write_Byte(dd,MPU6050_INT_EN_REG,0X00);	//�ر������ж�
	MPU6050_Write_Byte(dd,MPU6050_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU6050_Write_Byte(dd,MPU6050_FIFO_EN_REG,0x00);	//�ر�FIFO
    
	MPU6050_Write_Byte(dd,MPU6050_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
    res=MPU6050_Read_Byte(dd,MPU6050_INTBP_CFG_REG);
    wireless_printf_battery("mpu6050id:%d \r\n",res);
	res=MPU6050_Read_Byte(dd,MPU6050_DEVICE_ID_REG);
    wireless_printf_battery("mpu6050id:%d \r\n",res);
	if(res==MPU6050_ADDR)//����ID��ȷ
	{
        MPU6050_Write_Byte(dd,MPU6050_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
        MPU6050_Write_Byte(dd,MPU6050_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
        MPU6050_Set_Rate(dd,200);						//���ò�����Ϊ200Hz
        wireless_printf_battery("mpu6050��ʼ���ɹ�\r\n");
 	}else return 1;
	return 0;
}
//        while((res=mpu_dmp_init())){
//            HAL_Delay(400);
//            RGB_flashing(RGB1); 
//            RGB_flashing(RGB2); 
//            RGB_flashing(RGB3);
//            wireless_printf_battery("mpu6050��ʼ��ʧ��,�������:%d\r\n",res);
//            rgb_f_count = 1;
//        }//dmp������

u8 MPU6050_Write_Byte(mpu6050_init* dd,u8 reg,u8 data){
    return dd->tx(MPU6050_ADDR,reg,1,&data);
}
u8 MPU6050_Read_Byte(mpu6050_init* dd,u8 reg){
    u8 data=0;
    dd->rx(MPU6050_ADDR,reg,1,&data);
    return data;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU6050_Set_Gyro_Fsr(mpu6050_init* dd,u8 fsr)
{
    fsr=fsr<<3;
	return dd->tx(MPU6050_ADDR,MPU6050_GYRO_CFG_REG,1,&fsr);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU6050_Set_Accel_Fsr(mpu6050_init* dd,u8 fsr)
{
    fsr=fsr<<3;
    return dd->tx(MPU6050_ADDR,MPU6050_ACCEL_CFG_REG,1,&fsr);//���ü��ٶȴ����������̷�Χ
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU6050_Set_LPF(mpu6050_init* dd,u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return dd->tx(MPU6050_ADDR,MPU6050_CFG_REG,1,&data);//�������ֵ�ͨ�˲���  MPU6050_CFG_REG,data
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU6050_Set_Rate(mpu6050_init* dd,u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=dd->tx(MPU6050_ADDR,MPU6050_SAMPLE_RATE_REG,1,&data);	//�������ֵ�ͨ�˲��� MPU6050_SAMPLE_RATE_REG,data
 	return MPU6050_Set_LPF(dd,rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}
//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU6050_Get_Temperature(mpu6050_init* dd)
{
    u8 buf[2]; 
    short raw;
	float temp;
	dd->rx(MPU6050_ADDR,MPU6050_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU6050_Get_Gyroscope(mpu6050_init* dd,float *gx,float *gy,float *gz)
{
    u8 buf[6],res;  
	res=dd->rx(MPU6050_ADDR,MPU6050_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(float)(((u16)buf[0]<<8)|buf[1]);  
		*gy=(float)(((u16)buf[2]<<8)|buf[3]);  
		*gz=(float)(((u16)buf[4]<<8)|buf[5]);
	} 
    return res;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU6050_Get_Accelerometer(mpu6050_init* dd,float *ax,float *ay,float *az)
{
    u8 buf[6],res;  
	res=dd->rx(MPU6050_ADDR,MPU6050_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=(float)(((u16)buf[0]<<8)|buf[1]);  
		*ay=(float)(((u16)buf[2]<<8)|buf[3]);  
		*az=(float)(((u16)buf[4]<<8)|buf[5]);
	}
    return res;
}

int16_t MpuOffset[6] = {0};		//MPU6050������ֵ
u8 get_mpu6050data_frist(mpu6050_init* dd){
    u8 i = 0;
    u8 buf[12];
    dd->rx(MPU6050_ADDR,MPU6050_ACCEL_XOUTH_REG,6,buf);//�Ǽ��ٶ�
    dd->rx(MPU6050_ADDR,MPU6050_GYRO_XOUTH_REG,6,&buf[6]);//���ٶ�
    for(;i<6;++i){
        MpuOffset[i] = (short)buf[i*2]<<8|buf[i*2+1];
    }
    wireless_printf_battery("MpuOffset[%d]:%f\r\n",i,MpuOffset[i]);
    return 0;
}

//��ȡ�������ݣ����ڽṹ����
u8 get_mpu6050data(mpu6050_init* dd,Mpu6050_Data* Raw_data){
    u8 buffer[12],res,i=0;
    res=dd->rx(MPU6050_ADDR,MPU6050_ACCEL_XOUTH_REG,6,buffer);
    res=dd->rx(MPU6050_ADDR,MPU6050_GYRO_XOUTH_REG,6,&buffer[6]);//���ٶ�
    if(res!=0) return 1;
    int16_t pMpu[6];
	for(i=0;i<6;i++)
	{
		pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1])-MpuOffset[i];							/* ��������Ϊ16bit������ȥˮƽУ׼ֵ */
		if(i < 3)		/* �Ǽ��ٶȿ������˲� */
		{
			{
                KalmanFilter EKF[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	//�������˲�
                kalmanfiter(&EKF[i],(float)pMpu[i]);  
                pMpu[i] = (int16_t)EKF[i].Out;
			}
		}
		if(i > 2)		/* ���ٶ�һ�׻����˲� */
		{
			uint8_t k=i-3;
			const float factor = 0.15f;  	
			static float tBuff[3];		
 
			pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;
		}
	}
    
    Raw_data->T=(float)MPU6050_Get_Temperature(dd)/100.0;
    Raw_data->deg_s.x = (float)pMpu[3];
    Raw_data->deg_s.y = (float)pMpu[4];
    Raw_data->deg_s.z = (float)pMpu[5];
    Raw_data->acc_g.x = (float)pMpu[0];
    Raw_data->acc_g.y = (float)pMpu[1];
    Raw_data->acc_g.z = (float)pMpu[2];
    return 0;
}



