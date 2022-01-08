#include "stm32f4xx.h"
#include "delay.h"
#include "can.h"
#include "shoot.h"
#include "control.h"
#include "feedback.h"
#include "stdio.h"
#include "time.h"


shoot_PID_t shootPid2006_1;//shootPid,,shootPid2006_2;
//int16_t inputShootData;
int16_t setShootSpeed;
int16_t shootM1Speed,shootM2Speed,shootMotor2006_1,shootMotor2006_2;
//int16_t dErr_M3[3]={0};
int16_t dErr_M2006;//[3]={0};
//int16_t nowM5Speed,nowM6Speed;
//int16_t setM5Speed,setM6Speed;
int16_t nowAngle2006_1;//,nowAngle2006_2;
int16_t setAngle2006_1;//,setAngle2006_2;
int16_t setA2006_1;//,setA2006_2;
u8 inputFireData_2006_1;//,inputFireData_2006_2;
u16 pressTime_l,pressTime_r;
u8 open = 0;
u8 en = 0;
u16 delay;
//static int16_t ecd_count1 = 0;
//static int16_t ecd_count2 = 0;
int16_t lastAngle2006_1;//,lastAngle2006_2;
	

void shootSend(u16 shootMotor1, u16 shootMotor2,u16 shootMotor2006_1, u16 shootMotor2006_2)
{
//    u8 shootMotorData[4];
//    shootMotorData[0] = shootMotor1 >> 8;
//    shootMotorData[1] = shootMotor1 & 0xFF;
//    shootMotorData[2] = shootMotor2 >> 8;
//    shootMotorData[3] = shootMotor2 & 0xFF;
    
   //CAN1_Send_Msg(0x1ff, shootMotorData, 0x08);
	  //CAN2_Send_Msg(0x1ff, shootMotorData, 0x04);
	
		u8 shootMotorData[2];
		shootMotorData[0] = shootMotor2006_1 >> 8;           //2006 小子弹 ID：5
    shootMotorData[1] = shootMotor2006_1 & 0xFF;
//    shootMotorData[2] = shootMotor2006_2 >> 8;				 	//2006 大子弹 ID：6
//    shootMotorData[3] = shootMotor2006_2 & 0xFF;		
	  CAN1_Send_Msg(0x1ff, shootMotorData, 0x02);

}

void shootConversion(void)   //输入信号处理转换
{
	if (rcCtrl.rc.s1 == 1 && open == 0 && en == 0) 
	{
		open=1;
		en=1;
	}
	if (rcCtrl.rc.s1 == 3 ) en=0;
	if (rcCtrl.rc.s1 == 1 && open == 1 && en == 0) 
	{
		open=0;
		en =1;
	}
	
	if(open)
	{		
		//setShootSpeed = 7000;
		fric1_on(Fric_DOWN);
		if (delay<200) delay++;
		if (delay > 150)
		fric2_on(Fric_DOWN);
	}
	else
	{
		if (setShootSpeed>800) setShootSpeed -=66 ;
		else if (setShootSpeed<800) setShootSpeed =0;
		TIM_SetCompare1(TIM1, Fric_OFF);
		TIM_SetCompare4(TIM1, Fric_OFF);	
		delay =0;
	}		
		
	//setM5Speed = setShootSpeed;
	//setM6Speed = -setShootSpeed;
	
		if ((rcCtrl.mouse.press_l||rcCtrl.rc.s1==2) && open)
		{
			inputFireData_2006_1=1;
		
		}
		else if (rcCtrl.mouse.press_r || rcCtrl.rc.ch[4] > 600) 
		{
			inputFireData_2006_1=2;
		}
		else inputFireData_2006_1 = 0;
	
		
	
	if (inputFireData_2006_1 == 1)	setAngle2006_1=1000;
	else if (inputFireData_2006_1 == 2) setAngle2006_1=-800;
	else setAngle2006_1 = 0;
	
}

void shootFeedback(void)
{
//	nowM5Speed=M3508_5.Rotor_speed;
//	nowM6Speed=M3508_6.Rotor_speed;
	nowAngle2006_1=M2006_1.Rotor_speed;
	//	printf("%d \r\n",nowAngle2006_1);
//	nowAngle2006_2=M2006_2.Rotor_speed;
	 /*if (M2006_1.Mechanical_angle - lastAngle2006_1 > Half_ecd_range)
    {
        ecd_count1--;
    }
    else if (M2006_1.Mechanical_angle - lastAngle2006_1 < -Half_ecd_range)
    {
        ecd_count1++;
    }

    if (ecd_count1 == FULL_COUNT)
    {
        ecd_count1 = -(FULL_COUNT - 1);
    }
    else if (ecd_count1 == -FULL_COUNT)
    {
        ecd_count1 = FULL_COUNT - 1;
    }

    //计算输出轴角度
		nowAngle2006_1 = (ecd_count1 * ecd_range + M2006_1.Mechanical_angle) * Motor_ECD_TO_ANGLE;
		lastAngle2006_1 = M2006_1.Mechanical_angle;
		
		 if (M2006_2.Mechanical_angle - lastAngle2006_2 > Half_ecd_range)
    {
        ecd_count2--;
    }
    else if (M2006_2.Mechanical_angle - lastAngle2006_2 < -Half_ecd_range)
    {
        ecd_count2++;
    }

    if (ecd_count2 == FULL_COUNT)
    {
        ecd_count2 = -(FULL_COUNT - 1);
    }
    else if (ecd_count2 == -FULL_COUNT)
    {
        ecd_count2 = FULL_COUNT - 1;
    }

    //计算输出轴角度
    nowAngle2006_2 = (ecd_count2 * ecd_range + M2006_2.Mechanical_angle) * Motor_ECD_TO_ANGLE;
		lastAngle2006_2 = M2006_2.Mechanical_angle;*/
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

//static float shootPidContr0l(shoot_PID_t *pid, float get, float set, u8 i)
//{
//    float err;
//		
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
//    pid->get = get;
//    pid->set = set;

//    err = set - get;
//    pid->err = err;
//    pid->Pout = pid->kp * pid->err;
//    pid->Iout += pid->ki * pid->err;
//    pid->Dout = pid->kd * (pid->err - dErr_M3[i]); //由于pid有周期性，所以微分直接为本次误差-上次误差
//		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
//    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
//    pid->out = pid->Pout + pid->Iout + pid->Dout;
//    if (pid->out > pid->max_out)            pid->out = pid->max_out;
//    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
//		dErr_M3[i]=pid->err;
//		return pid->out;
//}
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


//void shootSpeedSet(void)
//{
//	setM5Speed = shootPidContr0l(&shootPid, nowM5Speed, setM5Speed, 1);//PID调节M5电机输出
//	setM6Speed = shootPidContr0l(&shootPid, nowM6Speed, setM6Speed, 2);//PID调节M6电机输出

//}
void shootAngleSet(void)
{
//	if (rcCtrl.rc.s1 == 2)
//	setAngle2006_1 = 300;//直接改成速度//nowAngle2006_1 + inputFireData_2006_1 * pi / 4;
//	else if (rcCtrl.rc.ch[4] > 600)
//	setAngle2006_1 = -200;//nowAngle2006_2 + inputFireData_2006_2 * pi / 3;
//	else setAngle2006_1 = 0;
//	
	setA2006_1 = shootPidContr02(&shootPid2006_1 , nowAngle2006_1, setAngle2006_1, 1);
//	setA2006_2 = shootPidContr02(&shootPid2006_2 , nowAngle2006_2, setA2006_2, 2);
//	
	
}


void shootInit(void)//snail电机初始化
{
	TIM_SetCompare1(TIM1, 2000);
  TIM_SetCompare4(TIM1, 2000);
  delay_ms(200);
  TIM_SetCompare1(TIM1, 1000);
	TIM_SetCompare4(TIM1, 1000);
	pressTime_l = 0;
	pressTime_r = 0;
//	shootPidInit(&shootPid, SHOOT_M3508_PID_MAX_OUT, SHOOT_M3508_PID_MAX_IOUT, SHOOT_M3508_PID_KP, SHOOT_M3508_PID_KI, SHOOT_M3508_PID_KD);
	shootPidInit(&shootPid2006_1, SHOOT_2006_PID_MAX_OUT, SHOOT_2006_PID_MAX_IOUT, SHOOT_2006_PID_KP, SHOOT_2006_PID_KI, SHOOT_2006_PID_KD);
//	shootPidInit(&shootPid2006_2, SHOOT_2006_PID_MAX_OUT, SHOOT_2006_PID_MAX_IOUT, SHOOT_2006_PID_KP, SHOOT_2006_PID_KI, SHOOT_2006_PID_KD);
}

void shootTask(void)//发射任务 调用所有函数
{
	shootConversion();
	shootFeedback();
//	shootSpeedSet();
	shootAngleSet();
	
//	if (rcCtrl.rc.s1 == 2)
//	shootSend(setM5Speed,setM6Speed,500,0);
//	else shootSend(setM5Speed,setM6Speed,0,0);
//	
//	if (rcCtrl.rc.ch[4] > 600)
//	shootSend(setM5Speed,setM6Speed,0,500);
//	else shootSend(setM5Speed,setM6Speed,0,0);
	
	shootSend(0,0,setA2006_1,0);
}


