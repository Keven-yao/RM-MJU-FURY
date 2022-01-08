
#include "stm32f4xx.h"
//#include "sys.h"
#include "delay.h"
#include "main.h"
void powerInit(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       
    GPIO_Init(GPIOH, &GPIO_InitStructure);             
	

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
    GPIO_Init(GPIOG, &GPIO_InitStructure);            
	
		
		//��Դ�ӿڳ�ʼ��,��ʱ���������õ�ڣ��ο��ٷ�������ʱ
		GPIO_SetBits(GPIOG, GPIO_Pin_13);//������5v�ϵ�
		//delay_us(709);
		GPIO_SetBits(GPIOH, GPIO_Pin_2);//24V OUT1 J20���½� 6020id5
		//delay_us(709);
		GPIO_SetBits(GPIOH, GPIO_Pin_3);//24V OUT2 J22�� 		 6020id6
		//delay_us(709);
		GPIO_SetBits(GPIOH, GPIO_Pin_4);//24V OUT3 J19���Ͻ� 3508id7
		//delay_us(709);
	  GPIO_SetBits(GPIOH, GPIO_Pin_5);//24V OUT4 J21��     ��
		//0 delay_us(709);
		GPIO_SetBits(GPIOG, GPIO_Pin_1);
}

