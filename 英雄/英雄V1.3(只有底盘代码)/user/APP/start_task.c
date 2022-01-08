
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//��ʼ�����������ȼ�
#define START_TASK_PRIO		1
//��ʼ���������ջ��С	
#define START_STK_SIZE 		512  
//��ʼ����������
TaskHandle_t StartTask_Handler;

//�����������ȼ�
#define Chassis_TASK_PRIO 7
//���������ջ
#define Chassis_STK_SIZE 512
//�������񴴽����
TaskHandle_t ChassisTask_Handler;

void start_rob_task(void *pvParameters)
{
	//��������ٽ籣���������ô��
	taskENTER_CRITICAL();
	xTaskCreate((TaskFunction_t)chassis_task,
						(const char *)"ChassisTask",
						(uint16_t)Chassis_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)Chassis_TASK_PRIO,
						(TaskHandle_t *)&ChassisTask_Handler);
	//ɾ����ʼ����
	vTaskDelete(StartTask_Handler);
	//�˳��ٽ���				
  taskEXIT_CRITICAL();            

}

void start_task()
{
	  
	 xTaskCreate((TaskFunction_t )start_rob_task,            //������
                (const char*    )"start_rob_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
     
}




