/*****************************************

 ���ڰ������ڽ����Ӿ������·�����Ҫ�㷨�͵���趨��ͨ��Э�飬�����������趨��ͨ��Э��Ҳ�ǿ��Ե�
 
 ���ڴ��ڰ��޷�ʹ��DMA2 ������û���ĳ�DMA����ģʽ�����ǿ�����һ�Ըĳ�DMA�������ݣ����������Ӿ����ݻ��ø���



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


//�Ӿ��������������ݴ�������
uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ] = {0};

int Usart5_Clean_IDLE_Flag = 0;

DMA_InitTypeDef xCom5DMAInit;


/***************************�Ӿ����ڳ�ʼ��***********************************/
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
	
	USART_ITConfig( UART8, USART_IT_IDLE, ENABLE  ); //ע��Ҫ���óɴ��ڿ����ж� 
  USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);
	
	
	USART_DMACmd( UART8, USART_DMAReq_Rx, ENABLE );

	
	xNvicInit.NVIC_IRQChannel                    = UART8_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 0;
	xNvicInit.NVIC_IRQChannelSubPriority         = 0;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

//DMA��ʼ��
void UART8_DMA_Init( void )
{		
	DMA_DeInit( DMA2_Stream5 );
	xCom5DMAInit.DMA_Channel = DMA_Channel_4;

	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������赽�洢��

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
//�����Ӿ������ݣ�������DMA1��DMA�Ĳ�������ѡ�ͺ�:���� F1 F4 F7 .. ��ӦDMA�Ĵ��ڣ�
void UART8_IRQHandler( void )
{
    int i;
    if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET)
    {
			  
			  USART_ClearFlag(UART8, USART_FLAG_RXNE);
        USART_ClearITPendingBit(UART8, USART_IT_RXNE); //ֻUSART_ReceiveDataҲ����
         
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
       USART_ITConfig( UART8, USART_IT_IDLE, DISABLE  ); //ע��Ҫ���óɴ��ڿ����ж� 
    }

	 


}


/**
  * @brief  ����һ�η���һ���ֽ�����
  * @param  �Լ�����õ�Ҫ�������е�����
  * @retval void
  * @attention  ������λ����
  */
void UART8_SendChar(uint8_t cData)
{
		USART_SendData( UART8, cData );   
	while (USART_GetFlagStatus( UART8, USART_FLAG_TC ) == RESET);
	

}


