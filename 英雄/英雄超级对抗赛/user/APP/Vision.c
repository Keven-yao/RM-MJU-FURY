#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"

//��ʼ�����������ȼ�
#define vision_TASK_PRIO		2
//��ʼ���������ջ��С	
#define vision_STK_SIZE 		512  
//��ʼ����������
TaskHandle_t visionTask_Handler;


//�����Ӿ����ݷ����������ȼ�
#define Transmitter_TASK_PRIO		6
//�����ջ��С	
#define Transmitter_STK_SIZE 		512  
//������
TaskHandle_t TransmitterTask_Handler;


//�����Ӿ����ݽ����������ȼ�
#define Receiving_visual_TASK_PRIO		7
//�����ջ��С	
#define Receiving_visual_STK_SIZE 		512  
//������
TaskHandle_t Receiving_visual_Handler;
void vision_rob_task(void *pvParameters)
{
		
	//��������ٽ籣���������ô��
 	  taskENTER_CRITICAL();
								
		xTaskCreate((TaskFunction_t )Transmitter_vision_task,            //������
                (const char*    )"Transmitter_vision_task",          //��������
                (uint16_t       )Transmitter_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )Transmitter_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&TransmitterTask_Handler);   //������   
								
								
	  xTaskCreate((TaskFunction_t )Receiving_visual_task,            //������
                (const char*    )"Receiving_visual_task",          //��������
                (uint16_t       )Receiving_visual_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )Receiving_visual_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&Receiving_visual_Handler);   //������  

		//ɾ����ʼ����
	vTaskDelete(visionTask_Handler);
	//�˳��ٽ���				
  taskEXIT_CRITICAL();    
}

void Vision_task()
{
	  
	 xTaskCreate((TaskFunction_t )vision_rob_task,            //������
                (const char*    )"vision_rob_task",          //��������
                (uint16_t       )vision_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )vision_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&visionTask_Handler);   //������       
	
}

