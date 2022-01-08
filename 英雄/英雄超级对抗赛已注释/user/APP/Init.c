
#include "main.h" 
#include "time.h"
#include "spi.h"
#include "imu.h"
//#include "led.h"
void Init(void)
{
		delay_Init();   //��ʱ��ʼ��
	
		SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		MPU6500_Init();
		
		powerInit();   //led��ʼ��
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //16����ռ���ȼ�   
	
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//CAN1��ʼ��
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//CAN1��ʼ��
    
	
		usart6_Init();  //ң����
		PWM_Init();     //pwm��ʼ��
	
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

	

}
