/*****************************************

 串口八是用于接收视觉数据下发，需要算法和电控设定好通信协议，用我们现在设定的通信协议也是可以的
 
 由于串口八无法使用DMA2 所以我没法改成DMA传输模式，你们可以试一试改成DMA接收数据，这样接收视觉数据会变得更快



****************************************/
#include "usart8.h"
#include "judge.h"
#include "main.h"


/* TX */
#define    GPIO_TX                   GPIOE
#define    GPIO_PIN_TX               GPIO_Pin_1
#define    GPIO_PINSOURCE_TX         GPIO_PinSource1
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOE

/* RX */
#define    GPIO_RX                   GPIOE
#define    GPIO_PIN_RX               GPIO_Pin_0
#define    GPIO_PINSOURCE_RX         GPIO_PinSource0
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOE

/* if use DMA */
#define    DMA1_Stream_RX            DMA1_Stream0

#define    COM5_PACKAGE_HEADER       JUDGE_FRAME_HEADER


//视觉发过来的数据暂存在这里
uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ] = {0};

int Usart5_Clean_IDLE_Flag = 0;

DMA_InitTypeDef xCom5DMAInit;


/***************************视觉串口初始化***********************************/
void UART8_Init( void )
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART8, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART8 );
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART8 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 115200;   
	xUsartInit.USART_WordLength          = USART_WordLength_8b;
	xUsartInit.USART_StopBits            = USART_StopBits_1;
	xUsartInit.USART_Parity              = USART_Parity_No;
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init( UART8, &xUsartInit );
	USART_Cmd( UART8, ENABLE );
	
	USART_ITConfig( UART8, USART_IT_IDLE, ENABLE  ); //注意要配置成串口空闲中断 
  USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);
	
	
	USART_DMACmd( UART8, USART_DMAReq_Rx, ENABLE );

	
	xNvicInit.NVIC_IRQChannel                    = UART8_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 0;
	xNvicInit.NVIC_IRQChannelSubPriority         = 0;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

//DMA初始化
void UART8_DMA_Init( void )
{		
	DMA_DeInit( DMA2_Stream5 );
	xCom5DMAInit.DMA_Channel = DMA_Channel_4;

	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	xCom5DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART8->DR);
	xCom5DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Judge_Buffer;
	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom5DMAInit.DMA_BufferSize = 100;
	xCom5DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom5DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom5DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom5DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom5DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom5DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom5DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom5DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom5DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom5DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream_RX, &xCom5DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  //stream0
}
/********************************************************************************/

extern u8 count;
extern u8 flag;
extern uint8_t rex[100];
extern uint8_t rex_tx[100];
//接收视觉的数据（基本是DMA1，DMA的参数表，先选型号:比如 F1 F4 F7 .. 对应DMA的串口）
void UART8_IRQHandler( void )
{
    int i;
    if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET)
    {
			  
			  USART_ClearFlag(UART8, USART_FLAG_RXNE);
        USART_ClearITPendingBit(UART8, USART_IT_RXNE); //只USART_ReceiveData也可以
         
        rex[count++] =  USART_ReceiveData(UART8);
			
			  if(rex[count-15] == 0xaa&&rex[count-1]==0xbb)
				{
					
					for(i = 0;i<count;i++)
					rex_tx[i]=rex[i];
					
						count=0;
					flag = 1;
				}
      
    }
     if(USART_GetFlagStatus(UART8, USART_FLAG_IDLE) != RESET)
    {       
       USART_ITConfig( UART8, USART_IT_IDLE, DISABLE  ); //注意要配置成串口空闲中断 
    }

	 


}


/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
void UART8_SendChar(uint8_t cData)
{
		USART_SendData( UART8, cData );   
	while (USART_GetFlagStatus( UART8, USART_FLAG_TC ) == RESET);
	

}


