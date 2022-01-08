#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"

//开始创建任务优先级
#define vision_TASK_PRIO		2
//开始创建任务堆栈大小	
#define vision_STK_SIZE 		512  
//开始创建任务句柄
TaskHandle_t visionTask_Handler;


//创建视觉数据发送任务优先级
#define Transmitter_TASK_PRIO		6
//任务堆栈大小	
#define Transmitter_STK_SIZE 		512  
//任务句柄
TaskHandle_t TransmitterTask_Handler;


//创建视觉数据接收任务优先级
#define Receiving_visual_TASK_PRIO		7
//任务堆栈大小	
#define Receiving_visual_STK_SIZE 		512  
//任务句柄
TaskHandle_t Receiving_visual_Handler;
void vision_rob_task(void *pvParameters)
{
		
	//进入代码临界保护区，不得打断
 	  taskENTER_CRITICAL();
								
		xTaskCreate((TaskFunction_t )Transmitter_vision_task,            //任务函数
                (const char*    )"Transmitter_vision_task",          //任务名称
                (uint16_t       )Transmitter_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )Transmitter_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&TransmitterTask_Handler);   //任务句柄   
								
								
	  xTaskCreate((TaskFunction_t )Receiving_visual_task,            //任务函数
                (const char*    )"Receiving_visual_task",          //任务名称
                (uint16_t       )Receiving_visual_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )Receiving_visual_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&Receiving_visual_Handler);   //任务句柄  

		//删除开始任务
	vTaskDelete(visionTask_Handler);
	//退出临界区				
  taskEXIT_CRITICAL();    
}

void Vision_task()
{
	  
	 xTaskCreate((TaskFunction_t )vision_rob_task,            //任务函数
                (const char*    )"vision_rob_task",          //任务名称
                (uint16_t       )vision_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )vision_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&visionTask_Handler);   //任务句柄       
	
}

