/*****************************************************
name:gimbal.c
brief:云台控制文件
function:通过收到的遥控信号、反馈、以及其他信号来控制云台的转向
author:CadenLei
date:V1.0:2019.4.20
date:V2.0:2019.4.30 CadenLei Implementing PID algorithm.
date:V2.1:2020.1.12 Caden 为新版英雄机器人适配代码，主要修改：
										1）取消小弹丸发射机构
										2）增加一个6020云台pitch轴电机
										3）供弹机构为6020驱动，再带一个2006枪口限位拨弹机构

*******************************************************/
#include "stm32f4xx.h"
#include "gimbal.h"
#include "can.h"
#include "feedback.h"
#include "control.h"
#include "stdio.h"
#include "usart.h"

#define midYaw 4100        //yaw轴中点位置
#define midPitch 7200         //pitch轴中点位置

int16_t inputYawData;          //遥控输入的数据
int16_t inputPitchData;

int16_t setYaw=midYaw,setPitch=midPitch;              //设定的位置初始为中点
int16_t addYaw,addPitch;															//yaw轴增添量/pitch轴增添量
int16_t leftLimit,rightLimit,upLimit,downLimit;       //上下左右限位位置变量
int16_t nowYaw,nowPitch;															//当前两个轴的位置   由can总线反馈的数据提供
int16_t yawSpeed,pitchSpeed ,i;													//两轴速度

Gimbal_PID_t gimbalPid_yaw,gimbalPid_pitch;           //定义云台pid数据结构

static float dErr_yaw = 0;														//yaw轴误差
static float dErr_pitch = 0;													//pitch轴误差


/*gimbal's signal data send by can1
**now yawMotor ID = 5   pitchMotor1 ID = 6   pitchMotor2 ID = 7*/
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2)
{
    u8 gimebalMotorData[4];
    gimebalMotorData[0] = chassisMotor1 >> 8;  
    gimebalMotorData[1] = chassisMotor1 & 0xFF;             
    gimebalMotorData[2] = chassisMotor2 >> 8;
    gimebalMotorData[3] = chassisMotor2 & 0xFF;
	//	gimebalMotorData[4] = -(chassisMotor2) >> 8;
  //  gimebalMotorData[5] = -(chassisMotor2) & 0xFF;

		CAN1_Send_Msg(0x2FF, gimebalMotorData, 0x04); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff

}

//后期改成自动完成!!!
void limitSet(void)//限位
{
	leftLimit=  6500;             //上限
	rightLimit=	3100;							//下限
	upLimit=		6830;							//下限
	downLimit=	7500;							//上限
}

/*************************************************
角度换算
获取信道数据，将其转换成角度增加值
判断是否超过限位值 超过的话返回限位值
**************************************************/
void angleConversion(void)
{
	if (RC_Ctl.rc.s2 == 1) 
	{
		inputYawData=0;
		setYaw = midYaw;
	}
	else if (RC_Ctl.rc.s2 == 3) inputYawData=(-RC_Ctl.rc.ch[2] - RC_Ctl.mouse.x*20);
	else if (RC_Ctl.rc.s2 == 2) inputYawData=(RC_Ctl.rc.ch[2] + RC_Ctl.mouse.x*20);
	
	inputPitchData= RC_Ctl.rc.ch[3]*0.3 + RC_Ctl.mouse.y*20;
	addYaw=inputYawData*0.6;
	addPitch=inputPitchData*0.8;  //遥控灵敏度

	if (addYaw != 0)//上下限位处理
	{
	if (nowYaw+addYaw <= leftLimit && nowYaw+addYaw >= rightLimit)  setYaw = nowYaw + addYaw;
		else if (nowYaw+addYaw > leftLimit) setYaw = leftLimit;
		else setYaw = rightLimit;
	}
	addYaw=0;
	if (addPitch != 0)
	{
	if (nowPitch+addPitch <= downLimit && nowPitch+addPitch >= upLimit)  setPitch = nowPitch + addPitch;
		else if (nowPitch+addPitch > downLimit) setPitch = downLimit;
		else setPitch = upLimit;
	}
	
//		USART6_TX_Byte(nowYaw);
//  USART6_TX_Byte(addYaw);
//	USART6_TX_Byte(setYaw);
	
	//USART6_TX_Byte(setPitch);
	
}

//获取feedback中得到的电机角度值
void angleFeedback(void)
{
	nowYaw=GM6020_Yaw.Mechanical_angle;
	nowPitch=GM6020_Pitch.Mechanical_angle;
	
	
}


static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
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
		
		//printf("云台pid初始化完成\r\n");
}
static float gimbalPidContr0l(Gimbal_PID_t *pid, float get, float set, float dErr)
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
    pid->Dout = pid->kd * (pid->err - dErr); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr=pid->err;
    USART6_TX_Byte(pid->out);
		return pid->out;

}

//设定每个电机旋转输出的速度
void speedSet(void)
{
	yawSpeed = gimbalPidContr0l(&gimbalPid_yaw, nowYaw, setYaw, dErr_yaw);//pid调节yaw轴输出
	pitchSpeed = gimbalPidContr0l(&gimbalPid_pitch, nowPitch, setPitch, dErr_pitch);//pid调节pitch轴输出
	gimbalSend(yawSpeed,pitchSpeed); //max speed=30000 min speed=-30000
}


void gimbalInit(void)
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
	gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
}


void gimbalTask(void)//云台任务 调用所有函数
{
	limitSet();
	angleConversion();
	angleFeedback();
	speedSet();
}


