#include "main.h"
#include "super_cap.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

float Chass_Realtime_Power;//实时热量
float Chass_Realtime_RemainEnergy;//实时剩余焦耳量

extern int current_level;//记录当前机器人的等级
extern int16_t setW_S_A_Dspeed;//设置底盘速度系数
void CAP_Send(int16_t temPower) 
{
     uint16_t set_temPower = temPower;
	   uint8_t sendbuf[8];
     sendbuf[0] = set_temPower >> 8;
	   sendbuf[1] = set_temPower;
	   CAN1_Send_Msg(0x210, sendbuf, 0x02);
	
	
    
}
//英雄 1. 70w  2. 90w 3.120w
//步兵 1.  60w 2. 80w 3.100w
void CAP_Ctrl(void)
{ 
	 //读取当前等级 设置超级电容功率。
	  current_level = JUDGE_ucGetRobotLevel(); 
	
	  if(current_level == 0)
	  { 
		 CAP_Send(5000);
	  }
	  else if(current_level == 1)
		{  
			 CAP_Send(7000);
			 chassis_ctrl(1);
			setW_S_A_Dspeed = 460;
		}else if(current_level == 2)
		{
			 CAP_Send(9000);
			chassis_ctrl(2);
			setW_S_A_Dspeed = 540;
		}else if(current_level == 3)
		{
			
			CAP_Send(12000);
			chassis_ctrl(2);
			setW_S_A_Dspeed = 640;
		}

	
	
}

void super_task(void *pvParameters)
{
  	
	while(1)
	{
		CAP_Ctrl();
		vTaskDelay(10);
	}
	
}

