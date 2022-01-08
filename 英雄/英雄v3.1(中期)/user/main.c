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


//板级初始化函数，初始化外设
void BSP_init(void);

int main(void)
{		
		//板级初始化函数，初始化外设
		BSP_init();
		//开始创建任务
		start_task();
		//开始任务调度
		vTaskStartScheduler();   
		//创建开始led_configuration任务
		while (1)
    {
        ;
    }

}
void BSP_init()
{
	  //中断组4，抢断优先级0-15响应优先级0
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		//初始化系统时钟
		delay_init(configTICK_RATE_HZ);
		//初始化流水灯
		led_configuration(); 
		//初始化串口1接收遥控器数据，并处理
    USART1_Configuration();
		//调试使用的串口6，不用关掉
    usart6_Init();
		//电源输出控制口 初始化
		powerInit();
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	
	
	  SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		MPU6500_Init();
	
		PWM_Init();     //pwm初始化
	
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

}






