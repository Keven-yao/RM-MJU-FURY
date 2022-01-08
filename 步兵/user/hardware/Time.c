#include "stm32f4xx.h"
//#include "sys.h"
#include "start_task.h"
#include "feedback.h"
#include "start_task.h"
#include "control.h"
#include "init.h"
#include "imu.h"
#include "main.h"
unsigned long long sysTime = 0;      //ʱ��� ��λms

void Tick_TIM5_Init (u16 arr )   //���Ϊ0.01ms * arr
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///ʹ��TIM7ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=850 -1 ;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//��ʼ��TIM7
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //����ʱ��7�����ж�
	TIM_Cmd(TIM5,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //��ʱ��7�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void TIM2_Int_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ����ʱ��2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2ʱ��ʹ��
	
	TIM_TimeBaseStructure.TIM_Prescaler=85-1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=10000;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM4
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
}

void TIM5_IRQHandler( void )
{
	if ( TIM_GetITStatus( TIM5, TIM_IT_Update ) == SET )
	{
		IMU_getYawPitchRoll( angle );
		//printf("angle0: %f\r\n",angle[0]);
	//	printf("angle0: %f  1:%f  2:%f\r\n",angle[0],angle[1],angle[2]);
		GetPitchYawGxGyGz();	
	}
	TIM_ClearITPendingBit( TIM5, TIM_IT_Update );
}


