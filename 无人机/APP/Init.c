#include "stm32f4xx.h"
#include "sys.h"
#include "init.h"
#include "shoot.h"
#include "time.h"
#include "can.h"
#include "PWM.h"
#include "delay.h"
#include "usart.h"
#include "gimbal.h"
#include "chassis.h"
#include "stdio.h"
#include "power.h"

void Init(void)
{
		powerInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    //CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    delay_Init();
    
		DR16_Init();
		usart2_Init();
		PWM_Init();
    gimbalInit();
	  chassisInit();
    shootInit();   
		TIM3_Int_Init();
		TIM4_Int_Init();
		printf("初始化完成！\r\n");
}
