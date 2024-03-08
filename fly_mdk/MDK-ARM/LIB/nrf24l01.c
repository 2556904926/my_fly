#include "nrf24l01.h"
#include "gpio.h"
#include "spi.h"
#include "stdio.h"
#include "string.h"
#include "battery.h"
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x01,0x01,0x01,0x01,0x01}; 	
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0X01,0X01,0X01,0X01,0X01};
#define Frequency 88 //2400+88Ƶ��
void nrf2401_init(){
    Clr_NRF24L01_CE(); 	                                        //ʹ��24L01
    Set_NRF24L01_CSN();                                         //SPIƬѡȡ��	 
    while(NRF24L01_Check()){};
	NRF24L01_TX_Mode();
    wireless_printf_battery("2.4G ͨѶ�ɹ�\r\n",0);
}
/**
 * @brief ��д����
 * @param txdata ���͵�����
 * @return ��������
 */
unsigned char nrf2401_send_byte(unsigned char txdata){
	unsigned char rxdata=0;
  HAL_SPI_TransmitReceive(&hspi2,&txdata,&rxdata,1,0x10);
	return rxdata;
}
/**
 * @brief д�Ĵ���
 * @param regaddr �Ĵ�����ַ
 * @param txdata д���ֵ
 * @return ��������
 */
uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
    uint8_t status;
    Clr_NRF24L01_CSN();                                         //ʹ��SPI����                                                    
    status =nrf2401_send_byte(regaddr);                                   //���ͼĴ����� 
    nrf2401_send_byte(data);                                              //д��Ĵ�����ֵ                                        
    Set_NRF24L01_CSN();                                         //��ֹSPI����                                                         
    return(status);       		                                //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val;
 	Clr_NRF24L01_CSN();                                         //ʹ��SPI����	
	                                                            
  	nrf2401_send_byte(regaddr);                                           //���ͼĴ�����
  	reg_val=nrf2401_send_byte(0XFF);                                      //��ȡ�Ĵ�������                                     
  	Set_NRF24L01_CSN();                                         //��ֹSPI����	
  	return(reg_val);                                            //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
	uint8_t status,ctr;
  	Clr_NRF24L01_CSN();                                         //ʹ��SPI����                                    
  	status=nrf2401_send_byte(regaddr);  //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	
 	  for(ctr=0;ctr<datalen;ctr++) pBuf[ctr]=nrf2401_send_byte(0XFF);                              //��������
  	Set_NRF24L01_CSN();                                         //�ر�SPI����                                                  
  	return status;                                              //���ض�����״ֵ̬
}

//��ָ��λ��дָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status,ctr;	
 	Clr_NRF24L01_CSN();                                         //ʹ��SPI����                          
  status = nrf2401_send_byte(regaddr);                                  //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(ctr=0; ctr<datalen; ctr++)                              
       nrf2401_send_byte(*pBuf++);                                     //д������	        
  	Set_NRF24L01_CSN();                                         //�ر�SPI����
  	return status;                                              //���ض�����״ֵ̬
}
//�ϵ���NRF24L01�Ƿ���λ
//д5������Ȼ���ٶ��������бȽϣ���ͬʱ����ֵ:0����ʾ��λ;���򷵻�1����ʾ����λ	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0x45,0x45,0x45,0x45,0x45};
	uint8_t buf1[5]={0};
	uint8_t i=0; 

	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);            //д��5���ֽڵĵ�ַ.
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);                          //����д��ĵ�ַ 
	for(;i<5;i++)
	{
	    if(buf1[i]!=0x45)
		{
		    break;					   
		}
	}
//	printf("%s",buf);
	if(i!=5)
        return 1;                                               //NRF24L01����λ
	return 0;		                                            //NRF24L01��λ
}	 	 
//����NRF24L01����һ������
//sendBuff:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *sendBuff)
{
//	NRF24L01_TX_Mode();
  uint8_t state;   
	Clr_NRF24L01_CE();
	NRF24L01_Write_Buf(WR_TX_PLOAD,sendBuff,TX_PLOAD_WIDTH);
    
 	Set_NRF24L01_CE();		//��������
    
	while(READ_NRF24L01_IRQ()!=0);		//�ȴ��������
    
	state=NRF24L01_Read_Reg(SPI_READ_REG+STATUS);		//��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);		//���TX_DS��MAX_RT�жϱ�־
    
//	NrfRxPacket();		//���մ����ص�ACK����
//	
//	NRF24L01_RX_Mode();
	if(state&MAX_TX){		//�ﵽ����ط�����	
    NRF24L01_Write_Reg(FLUSH_TX,0xff);		//���TX FIFO�Ĵ��� 	
		return MAX_TX; 
	}
	if(state&TX_OK){		//�������
		return TX_OK;
	}
	return 0xff;		//����ԭ����ʧ��
}

//				 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	Clr_NRF24L01_CE();		 
	NRF24L01_Write_Reg(SETUP_AW, 0x03); // ���õ�ַ���Ϊ 5bytes

	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ�˽���ACK	  

	NRF24L01_Write_Reg(SPI_WRITE_REG+FEATURE, 0x06 );//ʹ�ܶ�̬���س��ȼ������ص�ACKӦ��
	NRF24L01_Write_Reg(SPI_WRITE_REG+DYNPD, 0x01); //ʹ�ܽ��չܵ�0��̬���س���

	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);               //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);           //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,Frequency);  //����RFͨ��
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);          //�����Զ��ط����ʱ��:500us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);						//������Ƶ������Ϊ1MHZ�����书��Ϊ7dBm
	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG,0x0e);              //���û�������ģʽ�Ĳ���;����CRC������Ϊ����ģʽ,���������ж�
	Set_NRF24L01_CE();                                          //CEΪ��,10us����������
}		  
//����ģʽ	   
void NRF24L01_RX_Mode(void)
{
  Clr_NRF24L01_CE();	  
  NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0F);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC 
  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,Frequency);	     //����RFͨ��Ƶ��		  
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	       
  NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
  Set_NRF24L01_CE(); //CEΪ��,�������ģʽ 
  HAL_Delay(1);
}	


//�������ݰ�
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    				
	sta = NRF24L01_Read_Reg(SPI_READ_REG+STATUS);
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,sta); 	
	if(sta&RX_OK)//���ճɹ�
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		Set_NRF24L01_CE(); 
		return 0; 
	}
	return 1;
}					    
