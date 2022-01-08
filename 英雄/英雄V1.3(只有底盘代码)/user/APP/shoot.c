#include "stm32f4xx.h"
#include "delay.h"
#include "can.h"
#include "shoot.h"
#include "control.h"
#include "feedback.h"
#include "stdio.h"
#include "time.h"
#include "usart.h"



shoot_PID_t shootPid2006_1;                       //2006拨弹电机PID定义
shoot_PID_t shootPid;															//3508摩擦电机PID定义
shoot_PID_t shootPid6020;													//6020供弹电机PID定义
int16_t inputShootData;														//输入的发射相关数据
int16_t setShootSpeed;														//设置大弹丸摩擦轮速度
int16_t shootMotor2006_1,shootM4Speed;        		//拨弹轮速度和供弹轮速度
int16_t dErr_M3[3]={0};														//2个3508和1个6020 pid中的误差项
int16_t dErr_M2006; 															//2006电机中的pid的误差项
int16_t nowM5Speed,nowM6Speed;										//2个3508和1个6020 pid中的当前值项
int16_t nowGM4Speed;				
int16_t setM5Speed,setM6Speed;										//个6020 pid中的设定值项
int16_t setGM4Speed;				
//int16_t nowAngle2006_1;														//2006电机中的pid当前值项
//int16_t setAngle2006_1;														//2006电机中的pid设定值项
int16_t setSpeed2006_1;
int16_t nowSpeed2006_1;
int16_t setA2006_1;
int16_t setGM4Speed_out;
u8 inputFireData_2006_1;
u16 pressTime_l,pressTime_r;
u8 open = 0;
u8 en = 0;
u16 delay;
u8 fire = 0;
//static int16_t ecd_count1 = 0;
int16_t lastAngle2006_1=0;
u8 first = 1;


//static int16_t last_angle = 9000;
//static int16_t count=0;
//static int32_t num=0;
	

void shootSend(u16 shootMotor1, u16 shootMotor2,u16 shootMotor2006_1, u16 shootMotor6020)
{
    u8 shootMotorData[4];
    shootMotorData[0] = shootMotor1 >> 8;
    shootMotorData[1] = shootMotor1 & 0xFF;
    shootMotorData[2] = shootMotor2 >> 8;
    shootMotorData[3] = shootMotor2 & 0xFF;

	  CAN1_Send_Msg(0x1ff, shootMotorData, 0x04);
		
		u8 shootMotorData2[4];
    shootMotorData2[0] = shootMotor6020>> 8;
    shootMotorData2[1] = shootMotor6020 & 0xFF;
			shootMotorData2[2] = shootMotor2006_1 >> 8;           //2006 拨弹 ID：7
    shootMotorData2[3] = shootMotor2006_1 & 0xFF;	
	  CAN2_Send_Msg(0x1ff, shootMotorData2, 0x04);
	

}

void shootConversion(void)   //输入信号处理转换
{
	if (RC_Ctl.rc.s1 == 1 && open == 0 && en == 0) 
	{
		open=1;
		en=1;
	}
	if (RC_Ctl.rc.s1 == 3 ) en=0;
	if (RC_Ctl.rc.s1 == 1 && open == 1 && en == 0) 
	{
		open=0;
		en =1;
	}
	
	if(open)
	{		
		setShootSpeed = 7000;
		
	}
	else
	{
		if (setShootSpeed>800) setShootSpeed -=66 ;
		else if (setShootSpeed<800) setShootSpeed =0;
	
		delay =0;
	}		
		
	setM5Speed = setShootSpeed;
	setM6Speed = -setShootSpeed;
	
//		if ((RC_Ctl.mouse.press_l||RC_Ctl.rc.s1==2) && open)
//		{
//			inputFireData_2006_1=1;
//		
//		}
//	///	else if (RC_Ctl.mouse.press_r || RC_Ctl.rc.ch[4] > 600)    //鼠标右键或遥控器滑轮滚动时 反向拨弹
//	//	{
//	//		inputFireData_2006_1=2;
//	//	}
//		else inputFireData_2006_1 = 0;
		if(RC_Ctl.rc.s1 == 2) setSpeed2006_1 = -500;
		else setSpeed2006_1 = 0;
	
		if (RC_Ctl.rc.ch[4] == 660) setGM4Speed = 70; 
		else setGM4Speed = 0;		
 // else if (RC_Ctl.rc.s2 == 2) setGM4Angle = setGM4Angle;    
		
}

void shootFeedback(void)
{
	nowGM4Speed=GM6020_give.Rotor_speed;
	nowM5Speed=M3508_5.Rotor_speed;
	nowM6Speed=M3508_6.Rotor_speed;
	//nowAngle2006_1=M2006_1.Mechanical_angle;
	nowSpeed2006_1=M2006_1.Rotor_speed;

//	if(last_angle==9000)  {
//		last_angle = M2006_1.Mechanical_angle;
//		setAngle2006_1 =last_angle;
//	}
//		if (M2006_1.Mechanical_angle - last_angle > 4500) count--;
//		else if(M2006_1.Mechanical_angle - last_angle < -4500) count++;
//		
//		num=8191*count+M2006_1.Mechanical_angle;
//		last_angle = M2006_1.Mechanical_angle;
//	
   // nowAngle2006_1=num;

	
	//M2006_1.Rotor_speed;
//		printf("%d \r\n",nowAngle2006_1);

//	 if (M2006_1.Mechanical_angle - lastAngle2006_1 > Half_ecd_range)
//    {
//        ecd_count1--;
//    }
//    else if (M2006_1.Mechanical_angle - lastAngle2006_1 < -Half_ecd_range)
//    {
//        ecd_count1++;
//    }

//    if (ecd_count1 == FULL_COUNT)
//    {
//        ecd_count1 = -(FULL_COUNT - 1);
//    }
//    else if (ecd_count1 == -FULL_COUNT)
//    {
//        ecd_count1 = FULL_COUNT - 1;
//    }

//    //计算输出轴角度
//		nowAngle2006_1 = (ecd_count1 * ecd_range + M2006_1.Mechanical_angle) * Motor_ECD_TO_ANGLE;
//		lastAngle2006_1 = M2006_1.Mechanical_angle;
//		//printf("%d \r\n",nowAngle2006_1);
//		//USART6_TX_Byte(nowAngle2006_1>>24);
//		USART6_TX_Byte(nowAngle2006_1>>16);
//		USART6_TX_Byte(ecd_count1>>8);
//		USART6_TX_Byte(ecd_count1);

}


static void shootPidInit(shoot_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
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
		
//		printf("42摩擦轮pid初始化完成\r\n");
}

static float shootPidControl(shoot_PID_t *pid, float get, float set, u8 i)
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
    pid->Dout = pid->kd * (pid->err - dErr_M3[i]); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M3[i]=pid->err;
		return pid->out;
}
static float shootPidContr02(shoot_PID_t *pid, float get, float set, u8 i)
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
    pid->Dout = pid->kd * (pid->err - dErr_M2006); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M2006=pid->err;
		return pid->out;
}


void shootSpeedSet(void)
{
	
	setGM4Speed_out = shootPidControl(&shootPid6020, nowGM4Speed, setGM4Speed, 0);//PID调节M5电机输出
	setM5Speed = shootPidControl(&shootPid, nowM5Speed, setM5Speed, 1);//PID调节M5电机输出
	setM6Speed = shootPidControl(&shootPid, nowM6Speed, setM6Speed, 2);//PID调节M6电机输出

}
void shootAngleSet(void)
{
	
//		
//	if (first) {
//		setAngle2006_1 = num;
//		first = 0;
//	}
//	
//	if ( fire) //&& (nowAngle2006_1 - setAngle2006_1 < 58975))
//		setSpeed2006_1 = -500;
//	
//	else 
//	{
//		setSpeed2006_1 = 0;
//		setAngle2006_1 += 58975;
//		fire =0;
//	}
	
	
	
	//else setAngle2006_1 = 0;
	
	
//	if (RC_Ctl.rc.s1 == 2)
//	//setAngle2006_1 = -300;//直接改成速度//
//	setAngle2006_1 = setAngle2006_1 + inputFireData_2006_1 * pi / 4;
////	else if (RC_Ctl.rc.ch[4] > 600)
////	setAngle2006_1 = -200;//nowAngle2006_2 + inputFireData_2006_2 * pi / 3;
//	else setAngle2006_1 = 0;
//	
	setA2006_1 = shootPidContr02(&shootPid2006_1 , nowSpeed2006_1, setSpeed2006_1, 3);

}


void shootInit(void)//电机初始化
{
//	TIM_SetCompare1(TIM1, 2000);
//  TIM_SetCompare4(TIM1, 2000);
//  delay_ms(200);
//  TIM_SetCompare1(TIM1, 1000);
//	TIM_SetCompare4(TIM1, 1000);
//	pressTime_l = 0;
//	pressTime_r = 0;
	shootPidInit(&shootPid, SHOOT_M3508_PID_MAX_OUT, SHOOT_M3508_PID_MAX_IOUT, SHOOT_M3508_PID_KP, SHOOT_M3508_PID_KI, SHOOT_M3508_PID_KD);
	shootPidInit(&shootPid2006_1, SHOOT_2006_PID_MAX_OUT, SHOOT_2006_PID_MAX_IOUT, SHOOT_2006_PID_KP, SHOOT_2006_PID_KI, SHOOT_2006_PID_KD);
	shootPidInit(&shootPid6020, SHOOT_6020_PID_MAX_OUT, SHOOT_6020_PID_MAX_IOUT, SHOOT_6020_PID_KP, SHOOT_6020_PID_KI, SHOOT_6020_PID_KD);
}

void shootTask(void)//发射任务 调用所有函数
{
	shootFeedback();
	shootConversion();
	
	shootSpeedSet();
	shootAngleSet();
	
	shootSend(setM5Speed,setM6Speed,setA2006_1,setGM4Speed_out);
//	else shootSend(setM5Speed,setM6Speed,0,setGM4Angle_out);
//	
//	if (RC_Ctl.rc.ch[4] > 600)
//	shootSend(setM5Speed,setM6Speed,0,500);
//	else shootSend(setM5Speed,setM6Speed,0,0);
	
//	shootSend(0,0,setA2006_1,0);
}


