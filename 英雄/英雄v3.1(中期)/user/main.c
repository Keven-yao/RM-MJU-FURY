#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"

#include "init.h"
#include "time.h"
#include "spi.h"
#include "imu.h"

#include "delay.h"


//�弶��ʼ����������ʼ������
void BSP_init(void);

int main(void)
{		
		//�弶��ʼ����������ʼ������
		BSP_init();
		//��ʼ��������
		start_task();
		//��ʼ�������
		vTaskStartScheduler();   
		//������ʼled_configuration����
		while (1)
    {
        ;
    }

}
void BSP_init()
{
	  //�ж���4���������ȼ�0-15��Ӧ���ȼ�0
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		//��ʼ��ϵͳʱ��
		delay_init(configTICK_RATE_HZ);
		//��ʼ����ˮ��
		led_configuration(); 
		//��ʼ������1����ң�������ݣ�������
    USART1_Configuration();
		//����ʹ�õĴ���6�����ùص�
    usart6_Init();
		//��Դ������ƿ� ��ʼ��
		powerInit();
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	
	
	  SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		MPU6500_Init();
	
		PWM_Init();     //pwm��ʼ��
	
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

}






