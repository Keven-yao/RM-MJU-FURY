#include "stm32f4xx.h"
#include "sys.h"
#include "control.h"
#include "chassis.h"
#include "can.h"
#include "feedback.h"
#include "stdio.h"

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
    CAN1_Send_Msg(0x200, chassisMotorData, 0x08);
}



void chassisConversion(void)
{
	inputVxData=rcCtrl.rc.ch[1] + (rcCtrl.key.v & KEY_PRESSED_OFFSET_W)*100 - (rcCtrl.key.v & KEY_PRESSED_OFFSET_S)*100;
	inputVyData=rcCtrl.rc.ch[0] + (rcCtrl.key.v & KEY_PRESSED_OFFSET_D)*80 - (rcCtrl.key.v & KEY_PRESSED_OFFSET_A)*80;
	inputWzData=rcCtrl.rc.ch[2] + rcCtrl.mouse.x*20;
	if(rcCtrl.rc.s2 == 3) inputWzData=0;
	
	addVx=inputVxData*15;
	addVy=inputVyData*15;
	addWz=inputWzData*4;
	
	speedControl = (1 + (rcCtrl.key.v & KEY_PRESSED_OFFSET_SHIFT)*0.5) * (1 - (rcCtrl.key.v & KEY_PRESSED_OFFSET_CTRL)*0.5);
	
	setVx = addVx * speedControl;
	setVy = addVy * speedControl;
	setWz = addWz * speedControl;
	
	
	//假设左前方为ID 1 顺时针排序。
	setM1Speed = + setVx + setVy + setWz; 
	setM2Speed = - setVx + setVy + setWz; 
	setM3Speed = - setVx - setVy + setWz; 
	setM4Speed = + setVx - setVy + setWz; 
}

void chassisFeedback(void)
{
	nowM1Speed=M3508_1.Rotor_speed;
	nowM2Speed=M3508_2.Rotor_speed;
	nowM3Speed=M3508_3.Rotor_speed;
	nowM4Speed=M3508_4.Rotor_speed;
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
		
//		printf("底盘pid初始化完成\r\n");
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
	
	
	chassisSend(setM1Speed,setM2Speed,setM3Speed,setM4Speed); //发送计算好的电机速度控制电流
//	chassisSend(1000,-1000,-1000,1000);
}

void chassisInit(void)
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	chassisPidInit(&chassisPid, M3508_PID_MAX_OUT, M3508_PID_MAX_IOUT, M3508_PID_KP, M3508_PID_KI, M3508_PID_KD);
}


void chassisTask(void)//云台任务 调用所有函数
{
	chassisConversion();
	chassisFeedback();
	chassisSpeedSet();
}
