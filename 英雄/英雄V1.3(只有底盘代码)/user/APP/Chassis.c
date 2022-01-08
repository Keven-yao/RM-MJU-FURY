#include "stm32f4xx.h"
#include "sys.h"

#include "chassis.h"
#include "can.h"
#include "feedback.h"
#include "stdio.h"
#include "usart.h"



int16_t inputVxData; 
int16_t inputVyData;
int16_t inputWzData;

int16_t setVx,setVy,setWz;

int16_t addVx,addVy,addWz;

int16_t nowM1Speed,nowM2Speed,nowM3Speed,nowM4Speed;
int16_t setM1Speed,setM2Speed,setM3Speed,setM4Speed;

float speedControl;

Chassis_PID_t chassisPid;
static float dErr_M[5] = {0};



//向can总线发送器传输电机速度的控制数据
void chassisSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3, int16_t chassisMotor4)

{
    u8 chassisMotorData[8];
    chassisMotorData[0] = chassisMotor1 >> 8;
    chassisMotorData[1] = chassisMotor1 & 0xFF;
    chassisMotorData[2] = chassisMotor2 >> 8;
    chassisMotorData[3] = chassisMotor2 & 0xFF;
    chassisMotorData[4] = chassisMotor3 >> 8;
    chassisMotorData[5] = chassisMotor3 & 0xFF;
    chassisMotorData[6] = chassisMotor4 >> 8;
    chassisMotorData[7] = chassisMotor4 & 0xFF;
    CAN2_Send_Msg(0x200, chassisMotorData, 0x08);
}



void chassisConversion(void)
{
	inputVxData=RC_Ctl.rc.ch[1] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)*100 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)*100;
	inputVyData=RC_Ctl.rc.ch[0] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)*80 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)*80;
	inputWzData=RC_Ctl.rc.ch[2] + RC_Ctl.mouse.x*20;

	if(RC_Ctl.rc.s2 == 3) inputWzData=0;
	
	addVx=inputVxData*15;
	addVy=inputVyData*15;
	addWz=inputWzData*4;
	
	speedControl = (1 + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)*0.5) * (1 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL)*0.5);
	
	setVx = addVx * speedControl;
	setVy = addVy * speedControl;
	setWz = addWz * speedControl;
						if(USE_debug==3)
				{
            USART6_TX_Byte(0XFF);    
						USART6_TX_Byte(setVx);		
						USART6_TX_Byte(setVy);		
						USART6_TX_Byte(setWz);	
					 
				}	
	
	
	//假设左前方为ID 1 顺时针排序。
	setM1Speed = + setVx + setVy + setWz; 
	setM2Speed = - setVx + setVy + setWz; 
	setM3Speed = - setVx - setVy + setWz; 
	setM4Speed = + setVx - setVy + setWz;
						if(USE_debug==3)
				{
            USART6_TX_Byte(0XFF);    
						USART6_TX_Byte(setM1Speed);		
						USART6_TX_Byte(setM2Speed);		
						USART6_TX_Byte(setM3Speed);	
					  USART6_TX_Byte(setM4Speed);
				}	
}

void chassisFeedback(void)
{
	nowM1Speed=M3508_1.Rotor_speed;
	nowM2Speed=M3508_2.Rotor_speed;
	nowM3Speed=M3508_3.Rotor_speed;
	nowM4Speed=M3508_4.Rotor_speed;
							if(USE_debug==3)
				{
            USART6_TX_Byte(0XFF);    
						USART6_TX_Byte(nowM1Speed);		
						USART6_TX_Byte(nowM2Speed);		
						USART6_TX_Byte(nowM2Speed);	
					  USART6_TX_Byte(nowM2Speed);
				}	
}


static void chassisPidInit(Chassis_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
		
}

static float chassisPidContr0l(Chassis_PID_t *pid, float get, float set, u8 i)
{
    float err;
		
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * (pid->err - dErr_M[i]); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M[i]=pid->err;
		return pid->out;
}

//设定每个电机旋转输出的速度
void chassisSpeedSet(void)
{
	
	setM1Speed = chassisPidContr0l(&chassisPid, nowM1Speed, setM1Speed, 1);//PID调节M1电机输出
	setM2Speed = chassisPidContr0l(&chassisPid, nowM2Speed, setM2Speed, 2);//PID调节M2电机输出
	setM3Speed = chassisPidContr0l(&chassisPid, nowM3Speed, setM3Speed, 3);//PID调节M3电机输出
	setM4Speed = chassisPidContr0l(&chassisPid, nowM4Speed, setM4Speed, 4);//PID调节M4电机输出
	
								if(USE_debug==2)
				{
            USART6_TX_Byte(0XFF);    
						USART6_TX_Byte(setM1Speed);		
						USART6_TX_Byte(setM2Speed);		
						USART6_TX_Byte(setM3Speed);	
					  USART6_TX_Byte(setM4Speed);
				}	
	
	chassisSend(setM1Speed,setM2Speed,setM3Speed,setM4Speed); //发送计算好的电机速度控制电流

}

void chassisInit(void)
{
							
	chassisPidInit(&chassisPid, M3508_PID_MAX_OUT, M3508_PID_MAX_IOUT, M3508_PID_KP, M3508_PID_KI, M3508_PID_KD);
}


void chassisTask(void)//云台任务 调用所有函数
{
	chassisConversion();
	chassisFeedback();
	chassisSpeedSet();
	
}
void chassis_task(void *pvParameters)
{
	
	//底盘电机初始化
	delay_ms(50);

	
	led_green_on();
	while(1)
	{
			chassisInit();
		chassisTask();
	}
	led_red_on();

}



