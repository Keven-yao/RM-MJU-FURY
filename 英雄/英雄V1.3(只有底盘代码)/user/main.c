#include "stm32f4xx.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "sys.h"
#include "shoot.h"
#include "timer.h"
#include "can.h"
#include "PWM.h"
#include "delay.h"
#include "usart.h"
#include "gimbal.h"
#include "chassis.h"
#include "stdio.h"
#include "power.h"
#include "init.h"
#include "timers.h"
#include "start_task.h"



//�弶��ʼ����������ʼ������
void BSP_init();

int main(void)
{		
		//�弶��ʼ����������ʼ������
		BSP_init();
		//��ʼ��������
		start_task();
		//��ʼ�������
		vTaskStartScheduler();   
		//������ʼled_configuration����
		

}
void BSP_init()
{
	  //�ж���4���������ȼ�0-15��Ӧ���ȼ�0
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		//��ʼ��ϵͳʱ��
		delay_Init();
		//��ʼ����ˮ��
		led_configuration(); 
		//��ʼ������1����ң�������ݣ�������
    USART1_Configuration();
		//����ʹ�õĴ���6�����ùص�
    USART6_Configuration(115200);
		//��Դ������ƿ� ��ʼ��
		powerInit();
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

}






