#include "main.h"
#include "shoot.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define   clock_num         7
#define   unclock_num       2
#define		clock_speed       600
#define   unclock_speed     600

#define SHOOT_M3508_7_PID_KP 13.0f
#define SHOOT_M3508_7_PID_KI 0.00f
#define SHOOT_M3508_7_PID_KD 3.0f
#define SHOOT_M3508_7_PID_MAX_OUT 10000.0f
#define SHOOT_M3508_7_PID_MAX_IOUT 8000.0f

shoot_PID_t shootPid2006_1;                       //2006拨弹电机PID定义

shoot_PID_t shootPid_5_6,shootPid_7;															//3508摩擦电机PID定义

shoot_PID_t shootPid6020;													//6020供弹电机PID定义

int16_t inputShootData;														//输入的发射相关数据

int16_t setShootSpeed=0;														//设置大弹丸摩擦轮速度

int16_t shootMotor2006_1,shootM4Speed;        		//拨弹轮速度和供弹轮速度

int16_t dErr_M3[3]={0};														//2个3508和1个6020 pid中的误差项

int16_t dErr_M2006; 															//2006电机中的pid的误差项

int16_t nowM5Speed,nowM6Speed,nowM7Speed;										//2个3508和1个6020 pid中的当前值项

int16_t nowGM4Speed;				

int16_t setM5Speed,setM6Speed,setM7Speed,nowangle;										//个6020 pid中的设定值项

int16_t setGM4Speed;				

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

int16_t lastAngle2006_1=0;
u8 first = 1;
int flag_shoot = 0;
//云台Pitch值反馈
int16_t nowPitch;
//云台补偿线性值
int16_t  oldpitch;
double linear;//摩擦轮线性变化
#define Dial_linear   0.05f//拨弹盘

int yigeyige(int a,int16_t *Speed,int16_t now_angle)
{
	static int  clock=clock_num;
	static int  unclock=unclock_num;  //speed>0 ??ê±??   êy?μ′óD?±?′ó
	static int16_t old_angle=4000;
	
				if(unclock > 0)  
				{
					*Speed =  unclock_speed;		//??ê±??
				}			
				else if(clock>0) 
				{
					*Speed = -clock_speed;	//  ?3ê±??
				}
				else
				{
					a  = 1 ; 
					*Speed = 0 ;
					clock = clock_num;
					unclock = unclock_num;
				}				
				
				if(old_angle<2000&&now_angle>6000)    //?D?? ?3ê±?? μ??ú×a?ˉ·??ò
				{
					clock--;
				}
				 if(old_angle>6000&&now_angle<2000)   //?D?? ??ê±?? μ??ú×a?ˉ·??ò
				{
					unclock--;
				}
				
				old_angle=now_angle;
				
		return a;
}
	
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

void shootConversion(void)   //输入信号处理转换
{	

  static int addshootspeed = 0;	
	static int flag_key = 0;
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
	  led_red_on();	
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

  //setShootSpeed = 0;
	setM5Speed = setShootSpeed + addshootspeed;   
	setM6Speed = -addshootspeed - setShootSpeed;  //两个摩擦轮3508 转速相同，方向相反	
	setM7Speed = 0;
	
	//4200在8.3——9.1 4420 在8.9~9.8
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

//拨弹调速
void playing_speed(void)
{
	//	static int flag_flag=1;
//	if( RC_Ctl.mouse.press_l==1 && flag_flag == 1 )//éú±??-?·íù?′???ˉ￡?oó?ú?ééy???a?T1??ò???ˉ￡?oó?ú?óè??T???a1?￡??T??×óóò??ò????T?￡
//	{
//		flag_flag = 0 ;
//		setM7Speed= 0 ;
//	}
//	if(flag_flag == 0)
//	{
//		flag_flag=yigeyige(flag_flag,&setM7Speed,nowangle);  //speed>0 ??ê±??   êy?μ′óD?±?′ó	
//	}

//	if(flag_flag==1)
//	{
//		setM7Speed=0;
//	}
//	if((RC_Ctl.rc.s1==3)||(RC_Ctl.mouse.press_r==1))
//	{
//		setM7Speed = 1000;
//	}
		//再加一个按键设置
}

//步兵1 16 2. 
//英雄1.10 2.
//射速自定义，根据当前机器人等级，目前射速比设定值由偏差进行改变
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



void shootInit(void)//电机初始化
{
	shootPidInit(&shootPid_5_6, SHOOT_M3508_5_6_PID_MAX_OUT, SHOOT_M3508_5_6_PID_MAX_IOUT, SHOOT_M3508_5_6_PID_KP, SHOOT_M3508_5_6_PID_KI, SHOOT_M3508_5_6_PID_KD);
	shootPidInit(&shootPid_7, SHOOT_M3508_7_PID_MAX_OUT, SHOOT_M3508_7_PID_MAX_IOUT, SHOOT_M3508_7_PID_KP, SHOOT_M3508_7_PID_KI, SHOOT_M3508_7_PID_KD);
}

void shootTask(void)//发射任务 调用所有函数
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

