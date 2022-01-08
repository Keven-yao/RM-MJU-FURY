
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"
//��ʼ�����������ȼ�
#define START_TASK_PRIO		1
//��ʼ���������ջ��С	
#define START_STK_SIZE 		512  
//��ʼ����������
TaskHandle_t StartTask_Handler;

//�����������ȼ�
#define Chassis_TASK_PRIO 15      
//���������ջ
#define Chassis_STK_SIZE 512
//�������񴴽����
TaskHandle_t ChassisTask_Handler;

//��̨�������ȼ�
#define GIMBAL_TASK_PRIO 19
//��̨�����ջ
#define GIMBAL_STK_SIZE 512
//��̨������
TaskHandle_t GIMBALTask_Handler;

//�����������ȼ�=
#define SHOOT_TASK_PRIO 12
//���������ջ
#define SHOOT_STK_SIZE 512
//����������
TaskHandle_t SHOOTTask_Handler;

void start_rob_task(void *pvParameters)
{
	//��������ٽ籣���������ô��
	taskENTER_CRITICAL();
	//������������
	xTaskCreate((TaskFunction_t)chassis_task, 
						(const char *)"ChassisTask",
						(uint16_t)Chassis_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)Chassis_TASK_PRIO,
						(TaskHandle_t *)&ChassisTask_Handler);
						
	//������̨����
	xTaskCreate((TaskFunction_t)gimbal_Task,
						(const char *)"gimbal_Task",
						(uint16_t)GIMBAL_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)GIMBAL_TASK_PRIO,
						(TaskHandle_t *)&GIMBALTask_Handler);
	//������������
	xTaskCreate((TaskFunction_t)shoot_task,
						(const char *)"shoot_task",
						(uint16_t)SHOOT_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)SHOOT_TASK_PRIO,
						(TaskHandle_t *)&SHOOTTask_Handler);
	
	
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




