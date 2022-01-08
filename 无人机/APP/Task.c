#include "stm32f4xx.h"
#include "sys.h"
#include "gimbal.h"
#include "task.h"
#include "control.h"
#include "chassis.h"
#include "shoot.h"
#include "stdio.h"
#include "delay.h"

//static u32 tt=0;

void taskList(void)
{
    if(sbusNum == 18)
        sbusToRc();
    control();
		
		if(rcTimer < 140)//ң�������ж�����Ϊң����140ms����������ң������
		{
			gimbalTask();
		//	chassisTask();
			shootTask();
		}
		else 
		{
			chassisSend(0,0,0,0);
		  gimbalSend(0,0);
			shootSend(0,0,0,0);
		}
			
			
			
//		if(tt>=1000)
//		{
//			printf("����ɨ��ѭ������\r\n");
//			tt=0;
//		}
//		else tt++;
}
