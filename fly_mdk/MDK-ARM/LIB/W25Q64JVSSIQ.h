#ifndef __W25Q64JVSSIQ_H
#define __W25Q64JVSSIQ_H
#include "spi.h"
typedef struct W25Q64JVSSIQ_init W25Q64JVSSIQ_init;
typedef void (*CS_Control_Func)(uint8_t res);//csͨ�ýӿ�
typedef uint8_t (*w25_tx)(W25Q64JVSSIQ_init *init, uint8_t txData);
typedef uint8_t (*w25_rx)(W25Q64JVSSIQ_init *init);
struct W25Q64JVSSIQ_init {
    SPI_HandleTypeDef *spiHandle; // SPI���
    CS_Control_Func csControl; // ͨ�õ�CS���ƺ���
    w25_tx write_data;
    w25_rx read_data; 
};


//ָ��   
#define Write_Enable 0x06  //дʹ��
#define Write_Disable 0x04 //д����
#define Write_Enable_for_Volatile_Status_Register 0x50  //��ʧ��״̬�Ĵ�����дʹ�� 
#define Read_Status_Register_1 0x05  
#define Read_Status_Register_2 0x35 
#define Read_Status_Register_3 0x15  //��״̬�Ĵ���
#define Write_Status_Register_1 0x01
#define Write_Status_Register_2 0x31 
#define Write_Status_Register_3 0x11   //д״̬�Ĵ���
#define Read_Data 0x03   //������ָ��
#define Fast_Read 0x0B   //���ٶ�ȡ
#define Page_Program 0x02//ҳ�����  д���ݵ���˼

#define Block_Erase_4kb 0x20//ҳ����� 4k
#define Block_Erase_32kb 0x52   //32K
#define Block_Erase_64kb 0xD8//64k

#define Chip_Erase 0xC7//������������
#define Power_down 0xB9h//����
#define Release_Power_down__Device_ID  0xAB//���绽�� ���߶�ȡid
//
#define flash1_id 0x16 //falsh����id 
//��ַ  
#define block_(x) (0x000000+x*0x010000)  //64kb������ַ  128��
#define sector(x) (0x001000*x) //4k ��ַƫ����
#define address(x,y) (block_(x)+sector(y))
//
//������������������������������������������������������������������������������������
 void W25Q64_cs_out(uint8_t t);
 uint8_t w25_spi_witre(W25Q64JVSSIQ_init *init,uint8_t txData);
 uint8_t w25_spi_read (W25Q64JVSSIQ_init *init) ;
void w25_Init(W25Q64JVSSIQ_init *res, SPI_HandleTypeDef *hspi,CS_Control_Func gpio_out,w25_tx tx,w25_rx rx);
uint8_t Flash_Write(W25Q64JVSSIQ_init* res,uint8_t data);
uint8_t falsh_write_status(W25Q64JVSSIQ_init* dd,uint8_t mode,uint8_t Status_Register,uint8_t data);
uint8_t FLASH_Read(W25Q64JVSSIQ_init* dd,uint8_t mode,uint32_t address,uint8_t *res,uint8_t reslen);
void FLASH_Page_Program(W25Q64JVSSIQ_init* dd,uint32_t address,uint8_t *res,int reslen);
void FALSH_Sector_Erase(W25Q64JVSSIQ_init* dd,uint32_t address,uint8_t Block_Erase);
void FLASH_Sector(W25Q64JVSSIQ_init* dd,uint8_t data);
uint8_t FALSH_Read_id(W25Q64JVSSIQ_init* dd,uint8_t mode);
extern W25Q64JVSSIQ_init w25_1;

#endif