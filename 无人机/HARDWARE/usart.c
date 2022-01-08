#include "stm32f4xx.h"
#include "sys.h"

#include "usart.h"
#include "control.h"

#include "stdio.h"



void DR16_Init(void)
{
    GPIO_InitTypeDef  GPIO_TypeDefStructure;
	USART_InitTypeDef USART_TypeDefStructure;
    NVIC_InitTypeDef  NVIC_TypeDefStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	//PB7-Rx
	GPIO_TypeDefStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_TypeDefStructure.GPIO_Mode = GPIO_Mode_AF;		//复用功能
	GPIO_TypeDefStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_TypeDefStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_TypeDefStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_TypeDefStructure);
	
	USART_TypeDefStructure.USART_BaudRate = 100000;					       //波特率
	USART_TypeDefStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件控制流
	USART_TypeDefStructure.USART_Mode = USART_Mode_Rx; //接收模式
	USART_TypeDefStructure.USART_Parity = USART_Parity_Even; 		       //校验位
	USART_TypeDefStructure.USART_StopBits = USART_StopBits_1;        //停止位1
	USART_TypeDefStructure.USART_WordLength = USART_WordLength_8b;   //数据位8位
	USART_Init(USART1,&USART_TypeDefStructure);
    
    NVIC_TypeDefStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_TypeDefStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_TypeDefStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_TypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TypeDefStructure);

	USART_Cmd(USART1, ENABLE);                    //使能串口 
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);	
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}

void USART1_IRQHandler(void)
{
    u8 code;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        
        code = USART_ReceiveData(USART1);
				//USART2_TX_Byte(code);
        if(rcTimer >= 13)
            sbusNum = 0;
        if(sbusNum <= 18)
        {
            sbusBuf[sbusNum++] = code;
            rcTimer = 0;
        }
    }
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}







void usart2_Init(void)
{
    GPIO_InitTypeDef  GPIO_TypeDefStructure;
	USART_InitTypeDef USART_TypeDefStructure;
    NVIC_InitTypeDef  NVIC_TypeDefStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
    //PD6-Rx
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
    //PD5-Tx
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
	
	GPIO_TypeDefStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5;
	GPIO_TypeDefStructure.GPIO_Mode = GPIO_Mode_AF;		
	GPIO_TypeDefStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_TypeDefStructure.GPIO_PuPd = GPIO_PuPd_UP;   
	GPIO_TypeDefStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&GPIO_TypeDefStructure);
    
    USART_TypeDefStructure.USART_BaudRate = 115200;					       //波特率
	USART_TypeDefStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件控制流
	USART_TypeDefStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //接收与发送模式
	USART_TypeDefStructure.USART_Parity = USART_Parity_No; 		       //无校验位
	USART_TypeDefStructure.USART_StopBits = USART_StopBits_1;        //停止位1
	USART_TypeDefStructure.USART_WordLength = USART_WordLength_8b;   //数据位8位
	USART_Init(USART2,&USART_TypeDefStructure);
    
    NVIC_TypeDefStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_TypeDefStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_TypeDefStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_TypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TypeDefStructure);

	USART_Cmd(USART2, ENABLE);                    //使能串口 
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_ClearFlag(USART2, USART_FLAG_RXNE);	
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
}

//取消ARM的半主机工作模式
#pragma import(__use_no_semihosting)                             
struct __FILE { 
    int handle; 
}; 

FILE __stdout;          
void _sys_exit(int x) 
{ 
    x = x; 
}


int fputc(int ch, FILE *f){      
    while((USART2->SR&0X40)==0);
    USART2->DR = (u8) ch;      
    return ch;
}

void USART2_TX_Byte(unsigned char data)
{
	USART_SendData(USART2, data);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
}


void USART2_IRQHandler(void)
{
    u8 code;
    
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        
        code = USART_ReceiveData(USART2);
				printf("串口2:收到上位机发送的字符:%c\r\n",code);
        //USART2_TX_Byte(code);
    }
    USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}


