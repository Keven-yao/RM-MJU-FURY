/*****************************************************
   2021.8.7 
   云台任务有两个重要 1.电脑端云台俯仰控制的灵敏度和精度（鼠标移动，视野可以准确稳定移动到对应方向）
	                    2.小陀螺模式下云台的稳定性  有两种方式 1.差速 （现在再用） 2. imu稳定（写过但不稳定，需要改进）
                      所谓差速，就是底盘在动的时候给云台一定的值进行抵消底盘带动的力，在经过pid减小抖动
											imu是通过返回板子的角度，通过角度变化来保持云台稳定


*******************************************************/
#include "stm32f4xx.h"
#include "gimbal.h"
#include "imu.h"
#include "main.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Chassis.h"





float YAW_PID_KP= 38.0f ;
float YAW_PID_KI= 0.00f;
float YAW_PID_KD= 10.00f;
float YAW_PID_MAX_OUT =20000.0f;
float YAW_PID_MAX_IOUT =3000.00f;

//pitch 角度环 PID参数以及 PID最大输出，积分输出
float PITCH_PID_KP =22.0f;
float PITCH_PID_KI= 0.00f;
float PITCH_PID_KD =10.00f;
float PITCH_PID_MAX_OUT= 20000.0f;
float PITCH_PID_MAX_IOUT =3000.0f;




int16_t ANflag=0; //按下键盘按键标识位
int16_t inputYawData;          //遥控输入 水平 角 数据
int16_t gyroYawData;          //小陀螺左右移动输入 水平 角 数据
int16_t inputPitchData;        //遥控输入 俯仰 角 数据

int16_t setYaw=midYaw,setPitch=midPitch;              //设定的位置初始为中点

int16_t addYaw,addPitch;															//yaw轴增添量/pitch轴增添量

int16_t leftLimit,rightLimit,upLimit,downLimit;       //上下左右限位位置变量

int16_t nowYaw,nowRoll,oldYaw;															//当前两个轴的位置   由can总线反馈的数据提供

//将现在Pitch轴的数据作为判断射速条件
extern int16_t nowPitch;


int16_t yawSpeed,pitchSpeed,rollSpeed,gimbal_speed=85;//speed=-42											//两轴速度

volatile float yaw_direction;

int16_t yaw_direction_flag;

Gimbal_PID_t gimbalPid_yaw,gimbalPid_pitch,gimbalPid_roll;           //定义云台pid数据结构

static float dErr_yaw = 0;														//yaw轴误差
static float dErr_pitch = 0;													//pitch轴误差

float multiple=0.17f;

unsigned char send_bytes[16];
RecData  rec_data;

int current_level;//记录当前机器人的等级



//发送视觉数据函数协议
void sendDatas(SendData stm32data)
{
	
	 unsigned char pitch_bit_,yaw_bit_;
    if(stm32data.pitch_data.f<0)
     {   pitch_bit_ = 0x00;
         stm32data.pitch_data.f=-stm32data.pitch_data.f;
    }
    else pitch_bit_ = 0x01;

    if(stm32data.yaw_data.f<0)
     {   yaw_bit_ = 0x00;
         stm32data.yaw_data.f=-stm32data.yaw_data.f;
		 }
    else yaw_bit_ = 0x01;

    send_bytes[0] = 0xAA;

    send_bytes[1] = stm32data.pitch_data.c[0];
    send_bytes[2] = stm32data.pitch_data.c[1]; 
    send_bytes[3] = stm32data.pitch_data.c[2];
    send_bytes[4] = stm32data.pitch_data.c[3];
    send_bytes[5] = pitch_bit_;


    send_bytes[6] = stm32data.yaw_data.c[0];
    send_bytes[7] = stm32data.yaw_data.c[1];
    send_bytes[8] = stm32data.yaw_data.c[2];
    send_bytes[9] = stm32data.yaw_data.c[3];
    send_bytes[10] = yaw_bit_;


    send_bytes[11]=stm32data.shoot_speed;
    send_bytes[12]=stm32data.IsBufMode;
    send_bytes[13] =0xBB;
		for(int i = 0;i<14;i++)
		{
				 USART_SendData( UART8, send_bytes[i] );  
	       while (USART_GetFlagStatus( UART8, USART_FLAG_TC ) == RESET);
		}
    
}



//小陀螺设置档位速度
void top_speed() 
{
	if(current_level == 1)
	{
		gimbal_speed = 85;
	}else if(current_level == 2)
	{
		gimbal_speed = 85;
		
	}else if(current_level == 3)
	{
		gimbal_speed = 85;
	}	
	
}

void test_keybord()
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)
	{
		ANflag=2;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)
	{
		ANflag=3;
	}
	else if(RC_Ctl.rc.s2 !=1)
	{
		ANflag=1;
	}
		
}


/*gimbal's signal data send by can1
**now yawMotor ID = 5   pitchMotor1 ID = 6   pitchMotor2 ID = 7*/
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3) //CAN1 发送云台数据   先存入，再发
{
    u8 gimebalMotorData[6];
	
		gimebalMotorData[2] = chassisMotor2 >> 8;			 //存入 高八位  电机 数据 	
    gimebalMotorData[3] = chassisMotor2 & 0xFF;    //存入 高八位  电机 数据
		CAN2_Send_Msg(0x2FF, gimebalMotorData, 0x04); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
	
		gimebalMotorData[0] = chassisMotor1 >> 8;      //存入 高八位  电机 数据    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    gimebalMotorData[1] = chassisMotor1 & 0xFF;    //存入 低八位  电机 数据                    0 1 2 3 4 5 6 7  
		CAN1_Send_Msg(0x2FF, gimebalMotorData, 0x02); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
}

//控制6020电机俯仰的角度，具体得看电机安放的位置来控制限位
void limitSet(void)//限位     0 ~ 8191   8190/360=22.75
{
	leftLimit=  5100;             //  水平 左限 
	rightLimit=	3100;							//  水平 右限 (5100-3100)/22.75=2000/22.72=88°C
	//(5100+3100)/2=4100 水平中轴   硬件连接决定
	upLimit=		3300;							//  垂直 下限
	downLimit=	4250;							//  垂直 上限 (7500-6830)/22.75=670/22.75=30°C 
	//(7500-6830)/2=7165 垂直中轴   硬件连接决定	
}

/*************************************************
角度换算
获取信道数据，将其转换成角度增加值
判断是否超过限位值 超过的话返回限位值
**************************************************/
//extern volatile float yaw_angle,pitch_angle,roll_angle,gg_x,gg_y,gg_z,ga_x,ga_y,ga_z,ga_y1,kg,ga_y3,kg3,ga_y22,ga_y33;
void angleConversion(void)           //角度 转换    遥控 输入 云台 水平角 俯仰角 数据   
{
	test_keybord();
  top_speed();
	
	TIM_Cmd(TIM2, ENABLE);
	
	//遥控模式下的灵敏度
	inputPitchData= RC_Ctl.rc.ch[3]*5.0 + RC_Ctl.mouse.y*8;

	inputYawData=(RC_Ctl.rc.ch[2]*0.1 + RC_Ctl.mouse.x*5);
	
	//电脑端控制灵敏度  - 值是因为英雄电机是反着装，所以为-值
	gyroYawData=-(RC_Ctl.rc.ch[2]*0.3 + RC_Ctl.mouse.x*5);
	
	

	addYaw=-inputYawData*0.1;                                         
	addPitch=inputPitchData*0.1;  //遥控灵敏度  
/********************************************************************/   //Pitch 6020
  if (inputPitchData != 0)               //垂直 上下限位
		{				
			if (setPitch+addPitch <= downLimit && setPitch+addPitch >= upLimit)  setPitch = setPitch + addPitch;
			else if (setPitch+addPitch > downLimit) setPitch = downLimit;
			else setPitch = upLimit;				
		}	
	
	
/********************************************************************/   //Yaw 6020		
	if ((RC_Ctl.rc.s2 == 3)||(ANflag==3))            //水平 复位
	{
		
		
		if(Yaw_flag==1)
		{
  		setYaw+=gimbal_speed+addYaw;
		}
		
		if(Yaw_flag==0)
		{
			yaw_direction_flag=1;
			inputYawData=0;
			setYaw = midYaw;    //midYaw 4100   //yaw轴  水平轴 中点位置 
		}			
			pid_ctrl(1);
	}
	else if((RC_Ctl.rc.s2 == 2)||(ANflag==2))               //水平 陀螺仪
	{		
	
		Yaw_flag=1;	
		setYaw+=gimbal_speed+gyroYawData*0.17;
		pid_ctrl(2);
	}else { 
			pid_ctrl(1);
	}

}

//获取feedback中得到的电机角度值
void angleFeedback(void)   ////角度 反馈    从 can1 接收 现在 云台 水平俯仰 角 数据
{	
	nowYaw=GM6020_Yaw.Mechanical_angle;      //现在 水平 角 //////////（获取的是角度，不是速度）
	nowPitch=GM6020_Pitch.Mechanical_angle;  //现在 俯仰 角
	nowRoll=GM6020_Roll.Mechanical_angle;		 //现在 偏航 角
  	
	
}


static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)//云台 pid 初始化 
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
static float gimbalPidContr0l(Gimbal_PID_t *pid, float get, float set, float dErr)// 云台 pid 控制    pid算法  
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
		return pid->out;                          //将要设置 云台 水平角度 俯仰 角 数据
}

void pid_ctrl(int pid_flag)
{
	switch(pid_flag)
	{
		case 1:
				YAW_PID_KP= 34.0f ;
				YAW_PID_KI= 0.00f;
				YAW_PID_KD= 10.00f;
				YAW_PID_MAX_OUT =20000.0f;
				YAW_PID_MAX_IOUT =3000.00f;
				gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
			break;
		case 2:
			
				YAW_PID_KP= 7.12f ;
				YAW_PID_KI= 0.00f;
				YAW_PID_KD= 1.22f;
				YAW_PID_MAX_OUT =20000.0f;
				YAW_PID_MAX_IOUT =3000.00f;
				gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
			break;
		
	}
}

//设定每个电机旋转输出的速度
void speedSet(void)         //云台 角度 设置    将要设置 云台 水平角 俯仰角 数据  （通过pid算法得出) 
{
	if(nowYaw>6100&&oldYaw<2100)
			{
				setYaw=setYaw+8191;
			}
			
	else if(nowYaw<2100&&oldYaw>6100)
			{
				setYaw=setYaw-8191;
			}
	yawSpeed = gimbalPidContr0l(&gimbalPid_yaw, nowYaw, setYaw, dErr_yaw);          //pid调节yaw轴输出      水平角度
	pitchSpeed = gimbalPidContr0l(&gimbalPid_pitch, nowPitch, setPitch, dErr_pitch);//pid调节pitch轴输出    俯仰角度
	

	gimbalSend(yawSpeed,pitchSpeed,rollSpeed); //max speed=30000 min speed=-30000  //can1 发送 云台 水平俯仰 偏转 数据
	oldYaw=nowYaw;
}


void gimbalInit(void) //云台 pid 初始化  gimbalPid_yaw 云台水平pid   gimbalPid_pitch 云台俯仰pid
{
				
	gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
	gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);

}


void gimbalTask(void)//云台任务 调用所有函数
{
	limitSet();        //限制 设置     //设置     云台 水平左右限位 垂直上下限位    限位：当电机转过限位角度后 ，再发送装动命令时，电机角度不变
	angleConversion(); //角度 转换     //读取     遥控器 云台 水平俯仰  偏转 设置 数据   
	angleFeedback();   //角度 反馈     //读取     云台 水平俯仰 偏转 数据          can1 接收 现在 云台 水平俯仰 偏转 数据
	speedSet();        //角度 设置     //将要设置 云台 水平偏转 俯仰偏转 数据      can1 发送 云台 水平俯仰 偏转 数据
}


void gimbal_Task(void *pvParameters)
{
	gimbalInit();
  	
	while(1)
	{
		gimbalTask();
		vTaskDelay(15);
	}
	
}

