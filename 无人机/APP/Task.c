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
		
		if(rcTimer < 140)//遥控离线判定条件为遥控器140ms内有完整的遥控数据
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
//			printf("任务扫描循环正常\r\n");
//			tt=0;
//		}
//		else tt++;
}
