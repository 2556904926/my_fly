#include "nrf24l01.h"
#include "gpio.h"
#include "spi.h"
#include "stdio.h"
#include "string.h"
#include "battery.h"
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x01,0x01,0x01,0x01,0x01}; 	
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0X01,0X01,0X01,0X01,0X01};
#define Frequency 88 //2400+88频点
void nrf2401_init(){
    Clr_NRF24L01_CE(); 	                                        //使能24L01
    Set_NRF24L01_CSN();                                         //SPI片选取消	 
    while(NRF24L01_Check()){};
	NRF24L01_TX_Mode();
    wireless_printf_battery("2.4G 通讯成功\r\n",0);
}
/**
 * @brief 读写函数
 * @param txdata 发送的数据
 * @return 接受数据
 */
unsigned char nrf2401_send_byte(unsigned char txdata){
	unsigned char rxdata=0;
  HAL_SPI_TransmitReceive(&hspi2,&txdata,&rxdata,1,0x10);
	return rxdata;
}
/**
 * @brief 写寄存器
 * @param regaddr 寄存器地址
 * @param txdata 写入的值
 * @return 接受数据
 */
uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
    uint8_t status;
    Clr_NRF24L01_CSN();                                         //使能SPI传输                                                    
    status =nrf2401_send_byte(regaddr);                                   //发送寄存器号 
    nrf2401_send_byte(data);                                              //写入寄存器的值                                        
    Set_NRF24L01_CSN();                                         //禁止SPI传输                                                         
    return(status);       		                                //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val;
 	Clr_NRF24L01_CSN();                                         //使能SPI传输	
	                                                            
  	nrf2401_send_byte(regaddr);                                           //发送寄存器号
  	reg_val=nrf2401_send_byte(0XFF);                                      //读取寄存器内容                                     
  	Set_NRF24L01_CSN();                                         //禁止SPI传输	
  	return(reg_val);                                            //返回状态值
}	
//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
	uint8_t status,ctr;
  	Clr_NRF24L01_CSN();                                         //使能SPI传输                                    
  	status=nrf2401_send_byte(regaddr);  //发送寄存器值(位置),并读取状态值   	
 	  for(ctr=0;ctr<datalen;ctr++) pBuf[ctr]=nrf2401_send_byte(0XFF);                              //读出数据
  	Set_NRF24L01_CSN();                                         //关闭SPI传输                                                  
  	return status;                                              //返回读到的状态值
}

//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status,ctr;	
 	Clr_NRF24L01_CSN();                                         //使能SPI传输                          
  status = nrf2401_send_byte(regaddr);                                  //发送寄存器值(位置),并读取状态值
  	for(ctr=0; ctr<datalen; ctr++)                              
       nrf2401_send_byte(*pBuf++);                                     //写入数据	        
  	Set_NRF24L01_CSN();                                         //关闭SPI传输
  	return status;                                              //返回读到的状态值
}
//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，相同时返回值:0，表示在位;否则返回1，表示不在位	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0x45,0x45,0x45,0x45,0x45};
	uint8_t buf1[5]={0};
	uint8_t i=0; 

	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);            //写入5个字节的地址.
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);                          //读出写入的地址 
	for(;i<5;i++)
	{
	    if(buf1[i]!=0x45)
		{
		    break;					   
		}
	}
//	printf("%s",buf);
	if(i!=5)
        return 1;                                               //NRF24L01不在位
	return 0;		                                            //NRF24L01在位
}	 	 
//启动NRF24L01发送一次数据
//sendBuff:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *sendBuff)
{
//	NRF24L01_TX_Mode();
  uint8_t state;   
	Clr_NRF24L01_CE();
	NRF24L01_Write_Buf(WR_TX_PLOAD,sendBuff,TX_PLOAD_WIDTH);
    
 	Set_NRF24L01_CE();		//启动发送
    
	while(READ_NRF24L01_IRQ()!=0);		//等待发送完成
    
	state=NRF24L01_Read_Reg(SPI_READ_REG+STATUS);		//读取状态寄存器的值	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);		//清除TX_DS或MAX_RT中断标志
    
//	NrfRxPacket();		//接收带负载的ACK数据
//	
//	NRF24L01_RX_Mode();
	if(state&MAX_TX){		//达到最大重发次数	
    NRF24L01_Write_Reg(FLUSH_TX,0xff);		//清除TX FIFO寄存器 	
		return MAX_TX; 
	}
	if(state&TX_OK){		//发送完成
		return TX_OK;
	}
	return 0xff;		//其他原因发送失败
}

//				 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
	Clr_NRF24L01_CE();		 
	NRF24L01_Write_Reg(SETUP_AW, 0x03); // 设置地址宽度为 5bytes

	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了接收ACK	  

	NRF24L01_Write_Reg(SPI_WRITE_REG+FEATURE, 0x06 );//使能动态负载长度及带负载的ACK应答
	NRF24L01_Write_Reg(SPI_WRITE_REG+DYNPD, 0x01); //使能接收管道0动态负载长度

	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);               //使能通道0的自动应答    
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);           //使能通道0的接收地址  
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,Frequency);  //设置RF通道
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);          //设置自动重发间隔时间:500us;最大自动重发次数:10次
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);						//设置射频数据率为1MHZ，发射功率为7dBm
	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG,0x0e);              //配置基本工作模式的参数;开启CRC，配置为发射模式,开启所有中断
	Set_NRF24L01_CE();                                          //CE为高,10us后启动发送
}		  
//接收模式	   
void NRF24L01_RX_Mode(void)
{
  Clr_NRF24L01_CE();	  
  NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0F);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC 
  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,Frequency);	     //设置RF通信频率		  
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	       
  NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  Set_NRF24L01_CE(); //CE为高,进入接收模式 
  HAL_Delay(1);
}	


//接收数据包
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    				
	sta = NRF24L01_Read_Reg(SPI_READ_REG+STATUS);
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,sta); 	
	if(sta&RX_OK)//接收成功
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		Set_NRF24L01_CE(); 
		return 0; 
	}
	return 1;
}					    
