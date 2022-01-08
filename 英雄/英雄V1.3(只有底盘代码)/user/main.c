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



//板级初始化函数，初始化外设
void BSP_init();

int main(void)
{		
		//板级初始化函数，初始化外设
		BSP_init();
		//开始创建任务
		start_task();
		//开始任务调度
		vTaskStartScheduler();   
		//创建开始led_configuration任务
		

}
void BSP_init()
{
	  //中断组4，抢断优先级0-15响应优先级0
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		//初始化系统时钟
		delay_Init();
		//初始化流水灯
		led_configuration(); 
		//初始化串口1接收遥控器数据，并处理
    USART1_Configuration();
		//调试使用的串口6，不用关掉
    USART6_Configuration(115200);
		//电源输出控制口 初始化
		powerInit();
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

}






