/**************************************
   
	   2021.8.7 
		 
		射速的控制方法采用云台角度不同摩擦轮转速不同，有按键可以对摩擦轮进行加速减速的操作，对电机pid的修改也可以使拨弹更有力气 
		
   当前版本英雄机械的供弹和拨弹弹道有很大问题，需要机械进行修改，在修改的过程最好单独拿出来测试弹道和摩擦轮的可行度，这就需要机械和电控一起负责。
   
	 英雄机器人在比赛时电脑端选择最好是底盘功率优先，射击射速优先 :这是因为英雄机器人在爆发优先的范围内，射程太低了。


**************************************/
#include "main.h"
#include "shoot.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define SHOOT_M3508_7_PID_KP 13.0f
#define SHOOT_M3508_7_PID_KI 0.00f
#define SHOOT_M3508_7_PID_KD 3.0f
#define SHOOT_M3508_7_PID_MAX_OUT 10000.0f
#define SHOOT_M3508_7_PID_MAX_IOUT 8000.0f



shoot_PID_t shootPid_5_6,shootPid_7;							//3508摩擦电机PID定义
int16_t setShootSpeed=0;														//设置大弹丸摩擦轮速度
int16_t shootMotor2006_1,shootM4Speed;        		//拨弹轮速度和供弹轮速度
int16_t dErr_M3[3]={0};														//2个3508和1个6020 pid中的误差项
int16_t nowM5Speed,nowM6Speed,nowM7Speed;										//2个3508和1个6020 pid中的当前值项	
int16_t setM5Speed,setM6Speed,setM7Speed,nowangle;										//个6020 pid中的设定值项
int16_t setGM4Speed;				
int flag_shoot = 0;
//云台Pitch值反馈
int16_t nowPitch;
//云台补偿线性值
int16_t  oldpitch;
double linear;//摩擦轮线性变化
	

void shootSend(u16 shootMotor1,u16 shootMotor2,u16 shootMotor3)
{
    u8 shootMotorData[6];

  	shootMotorData[0] = shootMotor1 >> 8;
    shootMotorData[1] = shootMotor1 & 0xFF;
    shootMotorData[2] = shootMotor2 >> 8;
    shootMotorData[3] = shootMotor2 & 0xFF;				//控制电调id为 5，6  3508电机 
		shootMotorData[4] = shootMotor3 >> 8;
    shootMotorData[5] = shootMotor3 & 0xFF;
		CAN1_Send_Msg(0x1ff, shootMotorData, 0x06);    //控制电调id为 1，2006 
}

void shootConversion(void)   
{	

  static int addshootspeed = 0;	
	static int flag_key = 0;
	//按下C V可以进行摩擦轮加速减速   在比赛的时候为了过检测就设置一个键，比赛开始后按下X可以达成摩擦轮转速控制
	if(KEY_PRESSED_OFFSET_X & RC_Ctl.key.v ||flag_shoot == 1)
	{
  Rate_of_fire_adaptive();
		flag_shoot = 1;
	}else 
	{
		setShootSpeed = 4600;
	}
  if(KEY_PRESSED_OFFSET_C & RC_Ctl.key.v &&flag_key == 0)
	{	
	 
		addshootspeed = addshootspeed +20;
		flag_key = 1;
		
	}else if(KEY_PRESSED_OFFSET_V&RC_Ctl.key.v  && flag_key == 0)
	{
		led_red_off();
		addshootspeed = addshootspeed -20;
		flag_key = 1;
	}else {
		flag_key = 0;
	}

	setM5Speed = setShootSpeed + addshootspeed;   
	setM6Speed = -addshootspeed - setShootSpeed;  //两个摩擦轮3508 转速相同，方向相反	
	setM7Speed = 0;
	
	//电脑端左键拨弹，右键退弹，遥控器右边最下面2 中间3 最上面1
	if((RC_Ctl.rc.s1==2)||(RC_Ctl.mouse.press_l==1)) 
	{
	
			setM7Speed = -1310;	
	}
	else if((RC_Ctl.rc.s1==3)||(RC_Ctl.mouse.press_r==1))
	{
		setM7Speed = 1020;
	}else {
		setM7Speed = 0;
	}
	


}



//步兵1 16 2. 
//英雄1.10 2.
//射速自定义，通过云台俯仰值对摩擦轮进行变速,原因是低头转速一样容易漏弹
void Rate_of_fire_adaptive(void)
{
	

    if(nowPitch >= 4100)
	  {
	  	linear = nowPitch /4100.0-1;
		
			setShootSpeed = 5000 + 2000*linear;
		
	  }else if(nowPitch < 3820)
	  {
		  linear = 4100.0/nowPitch-1;
		
			setShootSpeed = 5000+ 1700 * linear;
		
		
	  }else if(nowPitch >= 3820&&nowPitch <4100)
	 {
		  linear = 4100.0/nowPitch-1;
		 setShootSpeed = 5000 + 1700*linear;
	 }	
 

}


void shootFeedback(void)  //存入的是速度，不是角度
{
	nowangle=M3508_7.Mechanical_angle;
	nowM5Speed=M3508_5.Rotor_speed;
	nowM6Speed=M3508_6.Rotor_speed;
	nowM7Speed=M3508_7.Rotor_speed;
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



void shootSpeedSet(void)
{
	setM5Speed = shootPidControl(&shootPid_5_6, nowM5Speed, setM5Speed, 1);//PID调节M5电机输出 3508
	setM6Speed = shootPidControl(&shootPid_5_6, nowM6Speed, setM6Speed, 2);//PID调节M6电机输出 3508
	setM7Speed = shootPidControl(&shootPid_7, nowM7Speed, setM7Speed, 3);
	shootSend(setM5Speed,setM6Speed,setM7Speed);
}


//电机初始化
void shootInit(void)
{
	shootPidInit(&shootPid_5_6, SHOOT_M3508_5_6_PID_MAX_OUT, SHOOT_M3508_5_6_PID_MAX_IOUT, SHOOT_M3508_5_6_PID_KP, SHOOT_M3508_5_6_PID_KI, SHOOT_M3508_5_6_PID_KD);
	shootPidInit(&shootPid_7, SHOOT_M3508_7_PID_MAX_OUT, SHOOT_M3508_7_PID_MAX_IOUT, SHOOT_M3508_7_PID_KP, SHOOT_M3508_7_PID_KI, SHOOT_M3508_7_PID_KD);
}
//发射任务 调用所有函数
void shootTask(void)
{
	shootFeedback();   //射击 反馈       //can1 获取两个摩擦轮3508速度 can2 获取2006
	shootConversion(); //射击 转换       //s1控制 两个3502，一个6020供弹，一个2006拨弹 的转与停，以及转速 
	shootSpeedSet();   //射击 角度 设置  //pid  
}

void shoot_task(void *pvParameters)
{
	shootInit();
	while(1)
	{
		
		shootTask();
		vTaskDelay(15);
		   
	}
	
	
}

