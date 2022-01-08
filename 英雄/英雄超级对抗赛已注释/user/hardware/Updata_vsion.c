
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"
#include "imu.h"
SendData s;

 u8 count;
 u8 flag;
 uint8_t rex[100] = {0x00};
 uint8_t rex_tx[100];
 bool Buff_Color ;
 //将imu的数据发送到视觉
 
//上正下负 左正右负
void Transmitter_vision_task(void *pvParameters)
{
	
   //MU_getYawPitchRoll( angle );
	//100ms发送一次用户数据
    while(1)
	  {
			
			
	    Buff_Color = is_red_or_blue();
			

      s.yaw_data.f=angle_imu[2];
		  s.pitch_data.f=angle_imu[1];
	  //printf("angle0: %f  1:%f  2:%f\r\n",angle_imu[0], s.pitch_data.f,s.yaw_data.f);
	    s.IsBufMode = false;
	    s.shoot_speed = 16;
	    sendDatas(s);
		
		  vTaskDelay(10);
			
	   }


}

void Receiving_visual_task(void *pvParameters)
{
	    
	   while(1)
	   {
				//接收视觉数据
        if(flag == 1)
           {
          
            flag = 0;
						vision_process(rex_tx);		
				    }
					 
				  vTaskDelay(10);
	   }
}
