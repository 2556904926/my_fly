#include "battery.h"
#include "dma.h"
#include "adc.h"
#include "stdlib.h"
#include "nrf24l01.h"
#include <stdio.h>
#include <stdint.h> 
#include <string.h>
#include <stdarg.h>
uint32_t ADC_Value=0;
void battery_init(){
   __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_ADC_Start_DMA(&hadc1,&ADC_Value,1);
}
uint32_t get_val(){
    return ADC_Value;
}
void wireless_printf_battery(const char* format, ...)
{
    char res[32] = {0};
    char* ptr = &res[1];
    
    // ʹ�ÿɱ�����б��������������Ĳ���
    va_list args;
    va_start(args, format);
    vsnprintf(ptr, 30, format, args);  // ����ʽ�����ַ���д�� ptr
    va_end(args);
    
    res[0] = strlen(ptr);
    
    // ����������жϣ�ȷ�����ݳ��Ȳ����� 31
    if (res[0] > 31) {
        res[0] = 31;  // ������ȳ��� 31���򽫳��Ƚض�Ϊ 31
    }
    NRF24L01_TxPacket((uint8_t*)res);  // ��������
}
void printArray(uint8_t* arr,int len) {
    for (int i = 0; i < len; i++) {
        printf("%c", arr[i]);
    }
    printf("\r\n");
}
//    FLASH_Read(w25_1,1,(uint32_t)address(0,0),res,31);