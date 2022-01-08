#include "usart7.h"

//#include "vision.h"
#include "led.h"


//视觉串口4配置,和裁判系统一样

/* TX */
#define    GPIO_TX                   GPIOE
#define    GPIO_PIN_TX7               GPIO_Pin_8
#define    GPIO_PINSOURCE_TX         GPIO_PinSource8
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOE

/* RX */
#define    GPIO_RX                   GPIOE
#define    GPIO_PIN_RX7               GPIO_Pin_7
#define    GPIO_PINSOURCE_RX         GPIO_PinSource7
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOE

/* DMA */
#define    DMA1_Stream_RX1            DMA1_Stream2

//接收到的视觉数据暂存在这里
 volatile unsigned char  Com4_Vision_Buffer[ VISION_BUFFER_LEN ] ;

int Usart7_Clean_IDLE_Flag = 0;

DMA_InitTypeDef xCom4DMAInit;

void UART7_Init( void )
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART7, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART7 );
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART7 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX7;
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_TX7;
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 115200;   
	xUsartInit.USART_WordLength          = USART_WordLength_8b;
	xUsartInit.USART_StopBits            = USART_StopBits_1;
	xUsartInit.USART_Parity              = USART_Parity_No;
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init( UART7, &xUsartInit );
	USART_Cmd( UART7, ENABLE );
	
	USART_ITConfig( UART7, USART_IT_IDLE, ENABLE  ); //注意要配置成串口空闲中断 

	USART_DMACmd( UART7, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART7, USART_DMAReq_Tx, ENABLE );
	
	UART7_DMA_Init( );//初始化usart5的DMA
	
	xNvicInit.NVIC_IRQChannel                    = UART7_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 0;
	xNvicInit.NVIC_IRQChannelSubPriority         = 0;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

//DMA初始化
void UART7_DMA_Init( void )
{		
	DMA_DeInit( DMA1_Stream2 );
	xCom4DMAInit.DMA_Channel = DMA_Channel_4;

	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	xCom4DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART7->DR);
	xCom4DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Com4_Vision_Buffer;
	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom4DMAInit.DMA_BufferSize = 100;
	xCom4DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom4DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom4DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom4DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom4DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom4DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom4DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom4DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom4DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom4DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream2, &xCom4DMAInit );	
	DMA_Cmd( DMA1_Stream2, ENABLE);  //stream0
}




//void USART7_TX_Byte(unsigned char data)   //发送
//{
//	USART_SendData(UART7, data);
//	while(USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);
//}

////视觉通信串口初始化
//void UART7_Init(void)
//{
//	USART_InitTypeDef  xUsartInit;
//	GPIO_InitTypeDef   xGpioInit;
//	NVIC_InitTypeDef   xNvicInit;

//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_DMA1, ENABLE );
//	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART7, ENABLE );

//	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART7 );//引脚复用
//	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART7 ); 

//	xGpioInit.GPIO_Pin   = GPIO_PIN_TX7 |GPIO_PIN_RX7;//0
//	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
//	xGpioInit.GPIO_OType = GPIO_OType_PP;
//	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;
//	GPIO_Init( GPIO_TX, &xGpioInit );
//	USART_DeInit(UART7);
//	xUsartInit.USART_BaudRate            = 115200;   //波特率
//	xUsartInit.USART_WordLength          = USART_WordLength_8b;//字长8比特
//	xUsartInit.USART_StopBits            = USART_StopBits_1;//一个停止位
//	xUsartInit.USART_Parity              = USART_Parity_No;//无奇偶校验
//	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
//	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	
//	USART_Init( UART7, &xUsartInit );
//	USART_ITConfig(UART7,USART_IT_RXNE,DISABLE);
//	USART_Cmd( UART7, ENABLE );

//	//DMA串口请求中断
//	USART_DMACmd( UART7, USART_DMAReq_Rx, ENABLE );
//	//USART_DMACmd( UART7, USART_DMAReq_Tx, ENABLE );
//	//中断配
//	xNvicInit.NVIC_IRQChannel                    = DMA1_Stream3_IRQn;
//	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 1;//优先级
//	xNvicInit.NVIC_IRQChannelSubPriority         = 1;
//	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
//	NVIC_Init( &xNvicInit );		
//	
//	DMA_Cmd(DMA1_Stream3, DISABLE);
//  while (DMA1_Stream3->CR & DMA_SxCR_EN);
//	DMA_DeInit( DMA1_Stream3 );
//	xCom4DMAInit.DMA_Channel = DMA_Channel_5;
//	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器
//	xCom4DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART7->DR);
//	xCom4DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Com4_Vision_Buffer;
//	xCom4DMAInit.DMA_BufferSize = 2;
//	xCom4DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	xCom4DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	xCom4DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	xCom4DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	xCom4DMAInit.DMA_Mode = DMA_Mode_Circular;
//	xCom4DMAInit.DMA_Priority = DMA_Priority_Medium;
//	xCom4DMAInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	xCom4DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	xCom4DMAInit.DMA_MemoryBurst = DMA_Mode_Normal;
//	xCom4DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init( DMA1_Stream3, &xCom4DMAInit );	
//	
//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
//	DMA_Cmd( DMA1_Stream3, ENABLE);  //stream0
//	led_red_on();

//}




//void DMA1_Stream3_IRQHandler(void)
//{

//	  led_red_off();
//	    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF6))
//     {
//        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF6);
//        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF6);
//			
//					u8 i=0;
//					for(i=0;i<18;i++)
//					{
//						USART7_TX_Byte(Com4_Vision_Buffer[i]);
//					}
//					USART7_TX_Byte(0XDD);
//          USART7_TX_Byte(0XDD);
//				
//				/*************/
//			}

//}


//串口4中断服务函数
void UART7_IRQHandler(void)
{	
	u8 code;
		if(USART_GetITStatus(UART7,USART_IT_IDLE)!=RESET)//检测到空闲线路
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		Usart7_Clean_IDLE_Flag = UART7->SR ;
		Usart7_Clean_IDLE_Flag = UART7->DR ;
		
		//DMA_Cmd(DMA1_Stream0,DISABLE);
		
	//	Usart5_Clean_IDLE_Flag = JUDGE_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);
		 code = USART_ReceiveData(UART7);
//    led_red_on();
		UART7_SendChar(code);
		
	//	memset(Judge_Buffer, 0, 200);
		DMA_Cmd(DMA1_Stream2,ENABLE);
	}
	
}

/**
  * @brief  串口一次发送一个字节数据
  * @param  数据
  * @retval void
  * @attention  8位
  */
void show(void)
{
			for(int i=0;i<16;i++)
		{
				UART7_SendChar(Com4_Vision_Buffer[i]);
		}
}
void UART7_SendChar(uint16_t cData)
{
	
		USART_SendData( UART7, cData );  
	while (USART_GetFlagStatus( UART7, USART_FLAG_TC ) == RESET);
	
 
}
