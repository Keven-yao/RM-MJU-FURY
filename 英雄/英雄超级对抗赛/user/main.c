/***************************************
           2021.8.7   周志勇
英雄代码总体框架由FreeRTOS实时操作系统，通过任务优先级高低在进行哪项任务

                            1.底盘任务 控制底盘四个3508电机，小陀螺模式下的运动控制，
                            2.云台任务 控制两个GM6020电机 对云台俯仰控制 小陀螺模式下云台保持稳定
英雄机器人控制分为四个任务：3.射击任务 控制三个3508电机  一个作为42mm子弹抬升提供动力，两个作为摩擦轮为子弹提供加速
                            4.超级电容任务 通过读取裁判系统的数据，对机器人的等级进行电容功率的升级

英雄视觉数据分为两个任务： 1.视觉数据发送任务  通过读取裁判系统的数据，将需要的数据进行编码 发送到上位机
                           2.视觉数据接收任务  通过读取上位机发送的数据进行解析，使云台进行跟踪控制射击

由于英雄视觉控制没有试过，可以借鉴步兵视觉控制。

**************************************/
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
	
    //视觉任务有算法就可以开启进行数据交互
  	//Vision_task();
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
    USART6_Configuration(115200);
		//电源输出控制口 初始化
		powerInit();
    //CAN接口初始化
	  
	  UART7_Init();
	  //串口八用于接收视觉数据并处理
	  UART8_Init();
	
	  //初始化can1和can2
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	
		//set_TIM8_fric1();
		
	
		set_fric();
	  SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		IMU_Init();
	
		PWM_Init();     //pwm初始化
		//set_TIM8_fric1();
	
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

}









