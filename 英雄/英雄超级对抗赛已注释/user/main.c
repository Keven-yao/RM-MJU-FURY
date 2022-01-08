/***************************************
    2021.8.7   周志勇  电话：15059735755  QQ:1349593434
英雄代码总体框架由FreeRTOS实时操作系统，通过任务优先级高低在进行哪项任务

                            1.底盘任务 控制底盘四个3508电机，小陀螺模式下的运动控制，
                            2.云台任务 控制两个GM6020电机 对云台俯仰控制 小陀螺模式下云台保持稳定
英雄机器人控制分为四个任务：3.射击任务 控制三个3508电机  一个作为42mm子弹抬升提供动力，两个作为摩擦轮为子弹提供加速
                            4.超级电容任务 通过读取裁判系统的数据，对机器人的等级进行电容功率的升级

英雄视觉数据分为两个任务： 1.视觉数据发送任务  通过读取裁判系统的数据，将需要的数据进行编码 发送到上位机
                           2.视觉数据接收任务  通过读取上位机发送的数据进行解析，使云台进行跟踪控制射击

由于英雄视觉控制没有试过，可以借鉴步兵视觉控制。外设引脚具体看数据手册 <开发板A原理图>
（对官方代码需要有一定的了解，他的控制算法值得借鉴）
代码习惯最好  .c 函数代码实现 .h函数申明和结构体的定义
底盘云台可以有两个控制模式 云台跟随底盘  底盘跟随云台（我们没有去做过，做出来对小陀螺模式会更加稳定）
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
	
		//初始化流水灯 可以通过点个灯来判断代码进行到哪一步
		led_configuration(); 
	
		//初始化串口1接收遥控器数据，并处理
    USART1_Configuration();
	
		//调试使用的串口6，不用关掉
    USART6_Configuration(115200);
	
		//电源输出控制口 初始化
		powerInit();
   
	  //串口7初始化 可以用于打印信息
	  UART7_Init();
	
	  //串口八用于接收视觉数据并处理
	  UART8_Init();
	
	  //初始化can1和can2
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	
		//imu初始化  通过SPI进行数据传递
	  SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		IMU_Init();
	  //pwm初始化
		PWM_Init();     
		
	  //定时器5 和定时器2 初始化，具体没啥用
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

}









