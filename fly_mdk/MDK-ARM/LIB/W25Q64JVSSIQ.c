#include "W25Q64JVSSIQ.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdlib.h>
W25Q64JVSSIQ_init w25_1;
 void W25Q64_cs_out(uint8_t t){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,(GPIO_PinState)t);
}
uint8_t w25_spi_witre(W25Q64JVSSIQ_init *init,uint8_t txData) {
	  
    uint8_t rxdata;
    if (HAL_SPI_Transmit(init->spiHandle,&txData,1, HAL_MAX_DELAY)!= HAL_OK) {
        return 1;
    }
    return 0;
}
uint8_t w25_spi_read (W25Q64JVSSIQ_init* init) {
	  
    uint8_t rxdata;
    HAL_SPI_Receive(init->spiHandle,&rxdata,1, HAL_MAX_DELAY);
    return rxdata;
}
void w25_Init(W25Q64JVSSIQ_init *res, SPI_HandleTypeDef *hspi,CS_Control_Func gpio_out,w25_tx tx,w25_rx rx){
    res->spiHandle = hspi;
    res->csControl = gpio_out;
    res->write_data = tx;
    res->read_data = rx;
    res->csControl(1);
}  
/**
 * @brief 写指令
 * @param res 
                  Write_Enable 0x06      写使能
                  Write_Disable 0x04     写禁用
                  Write_Enable_for_Volatile_Status_Register 0x50 

                  Read_Status_Register_1 0x05  姿态寄存器1
                  Read_Status_Register_2 0x35  状态寄存器2
                  Read_Status_Register_3 0x15  状态寄存器3
 * @return 上面有返回值，下面无返回值
 */
uint8_t Flash_Write(W25Q64JVSSIQ_init *res,uint8_t data)
{
    uint8_t rxdata;
    res->csControl(0);                                    //使能SPI传输                                                    
    rxdata=res->write_data(res,data);                                   //发送寄存器号                                        
    res->csControl(1);                 //禁止SPI传输
     return rxdata;	
}
/**
 * @brief 写状态标志位
 * @param mode   0,非易失性状态位   1易失性状态位
 * @param Status_Register 
                  Write_Status_Register_1 0x01
                  Write_Status_Register_2 0x31 
                  Write_Status_Register_3 0x11   //写状态寄存器
 * @return 0：写入成功，1：写入失败
 */
uint8_t falsh_write_status(W25Q64JVSSIQ_init* dd,uint8_t mode,uint8_t Status_Register,uint8_t data){
    uint8_t res[3]={Read_Status_Register_1,Read_Status_Register_2,Read_Status_Register_3};
    uint8_t count,while_count=0;
    switch(Status_Register){
        case Write_Status_Register_1:
            count = 0;
        break;
        case Write_Status_Register_2:
            count = 1;
        break;
        case Write_Status_Register_3:
            count = 2;
        break;
        default:
            return 1;
        break;
    }
    if(mode==1){
        Flash_Write(dd,Write_Enable);
    }
    else {
        Flash_Write(dd,Write_Enable_for_Volatile_Status_Register);
    }
    dd->csControl(0);                                    //使能SPI传输  
    dd->write_data(dd,Status_Register);
    dd->write_data(dd,data);
    dd->csControl(1);                                    //使能SPI传输  
    if(mode==0){
        while(Flash_Write(dd,res[count])<<7){
             while_count++;
           if(while_count>=255) return 1;
        }
    }
    return 0;
}
/**
 * @brief 读取数据
 * @brief mode 0,正常读取，1快速读取
 * @param address  24位地址，按照32位传进去，分解
 * @param res 存贮数组
 * @return 0：写入成功，1：写入失败
 */
uint8_t FLASH_Read(W25Q64JVSSIQ_init* dd,uint8_t mode,uint32_t address,uint8_t *res,uint8_t reslen){
     address &= 0xFFFFFF;
    // 分解地址
    uint8_t addrHigh = (address >> 16) & 0xFF; // 高8位
    uint8_t addrMid = (address >> 8) & 0xFF;  // 中8位
    uint8_t addrLow = address & 0xFF;         // 低8位
    dd->csControl(0);
    if(mode==0){
    dd->write_data(dd,Read_Data);
    }
    else dd->write_data(dd,Fast_Read);
    dd->write_data(dd,addrHigh);
    dd->write_data(dd,addrMid);
    dd->write_data(dd,addrLow);
    if(mode==1) {
        dd->read_data(dd);//空读取
    }
    for(int i=0;i<reslen;++i){
        res[i] = dd->read_data(dd);
    }
    dd->csControl(1);
    int while_count=0;
    while(Flash_Write(dd,Read_Status_Register_1)<<7){
          while_count++;
          if(while_count>=255) {
              return 1;
          }
    }
    return 0;
}
/**
 * @brief 写数据
 * @param address  24位地址，按照32位传进去，分解
 * @param res 写入的数组
 * @return 0：写入成功，1：写入失败
 */
void FLASH_Page_Program(W25Q64JVSSIQ_init* dd,uint32_t address,uint8_t *res,int reslen)
{
    
    Flash_Write(dd,Write_Enable);
    address &= 0xFFFFFF;
    // 分解地址
    uint8_t addrHigh = (address >> 16) & 0xFF; // 高8位
    uint8_t addrMid = (address >> 8) & 0xFF;  // 中8位
    uint8_t addrLow = address & 0xFF;         // 低8位
    if(reslen > 255){
        addrLow = 0x00;
    }
    dd->csControl(0);
    
    dd->write_data(dd,Page_Program);
    dd->write_data(dd,addrHigh);
    dd->write_data(dd,addrMid);
    dd->write_data(dd,addrLow);
    
    for(int i=0;i<reslen;++i){
         dd->write_data(dd,res[i]);
    }
    dd->csControl(1);
    return ;
}

/*
 * @brief 擦除数据
 * @param address  24位地址，按照32位传进去，分解
 * @param 
           Block_Erase_4kb 0x20//页面擦除 4k
           Block_Erase_32kb 0x52   //32K
           Block_Erase_64kb 0xD8//64k
 * @return 0：写入成功，1：写入失败
 */
void FALSH_Sector_Erase(W25Q64JVSSIQ_init* dd,uint32_t address,uint8_t Block_Erase){
    address &= 0xFFFFFF;
    // 分解地址
    uint8_t addrHigh = (address >> 16) & 0xFF; // 高8位
    uint8_t addrMid = (address >> 8) & 0xFF;  // 中8位
    uint8_t addrLow = address & 0xFF;         // 低8位
    Flash_Write(dd,Write_Enable);
    dd->csControl(0);
    dd->write_data(dd,Block_Erase);
    dd->write_data(dd,addrHigh);
    dd->write_data(dd,addrMid);
    dd->write_data(dd,addrLow);
    dd->csControl(1);
    int while_count=0;
    while(Flash_Write(dd,Read_Status_Register_1)<<7){
          while_count++;
          if(while_count>=255) {
              return ;
          }
    }
    return;
}
/**
 * @brief 
   @param 
         Chip_Erase 0xC7//擦除所有数据
         Power_down 0xB9h//掉电

 * @return 无
 */
void FLASH_Sector(W25Q64JVSSIQ_init* dd,uint8_t data){
    dd->csControl(0);
    dd->write_data(dd,data);
    dd->csControl(1);
}
/**
 * @brief 
   @param mode =0掉电唤起,读取id
         Release_Power_down__Device_ID 0xAB//掉电唤起
 * @return 无
 */
uint8_t FALSH_Read_id(W25Q64JVSSIQ_init* dd,uint8_t mode){
    uint8_t id = 0;
    dd->csControl(0);
    dd->write_data(dd,Release_Power_down__Device_ID);
    if(mode) {
        for(int i=0;i<3;++i) {
            dd->write_data(dd,0x00);
        }
        id=dd->read_data(dd);
    }        
    dd->csControl(1);
    return id;
}

