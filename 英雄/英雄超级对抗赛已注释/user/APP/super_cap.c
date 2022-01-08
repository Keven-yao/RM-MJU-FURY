/**********************************
   2021 8.8 
  我们现在使用的超级电容模块只需要我们得到机器人当前的等级，对超级电容进行功率设定。
	英雄机器人 1级70w  2级 90w  3级 120w
	超级电容的控制会帮我们英雄机器人将功率限制在设置的范围内，防止超功率扣血的情况，在电容储存的能量耗尽，
	机器人将无法移动，等超级电容充电就可以继续使用。在电脑端可以看到功率的使用情况
	通信方式是can1通信



***********************************/
#include "main.h"
#include "super_cap.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

float Chass_Realtime_Power;//实时热量
float Chass_Realtime_RemainEnergy;//实时剩余焦耳量

extern int current_level;//记录当前机器人的等级
extern int16_t setW_S_A_Dspeed;//设置底盘速度系数
//超级电容设定值反馈
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
 //读取当前等级 设置超级电容功率。
//setW_S_A_Dspeed 是为了在等级提升后，机器人的移动变快
void CAP_Ctrl(void)
{ 
	
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

