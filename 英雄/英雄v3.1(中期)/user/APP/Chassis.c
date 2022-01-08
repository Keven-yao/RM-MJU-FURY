#include "stm32f4xx.h"
//#include "sys.h"


#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

int16_t inputVxData; 
int16_t inputVyData;
int16_t inputWzData;
int16_t Yaw_flag=0;

int16_t setVx,setVy,setWz;

int16_t addVx,addVy,addWz;

int16_t nowM1Speed,nowM2Speed,nowM3Speed,nowM4Speed;
int16_t setM1Speed,setM2Speed,setM3Speed,setM4Speed;

float speedControl;

Chassis_PID_t chassisPid;
static float dErr_M[5] = {0};



//向can总线发送器传输电机速度的控制数据
void chassisSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3, int16_t chassisMotor4) //底盘 发送

{
    u8 chassisMotorData[8];   // 底盘 电机 数据
    chassisMotorData[0] = chassisMotor1 >> 8;     //存入 高八位 底盘 电机 数据    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    chassisMotorData[1] = chassisMotor1 & 0xFF;   //存入 低八位 底盘 电机 数据                    0 1 2 3 4 5 6 7
    chassisMotorData[2] = chassisMotor2 >> 8;
    chassisMotorData[3] = chassisMotor2 & 0xFF;
    chassisMotorData[4] = chassisMotor3 >> 8;
    chassisMotorData[5] = chassisMotor3 & 0xFF;
    chassisMotorData[6] = chassisMotor4 >> 8;
    chassisMotorData[7] = chassisMotor4 & 0xFF;
    CAN1_Send_Msg(0x200, chassisMotorData, 0x08);   //can2发送底盘 四个 电机 数据
}


//////////////////////////////////////////////////////////////////////////////////
void chassisConversion(void)//底盘 转换  读取遥控器底盘设置数据
{
	if(Yaw_flag==1)
	{
		if((RC_Ctl.rc.s2 == 1)&&((GM6020_Yaw.Mechanical_angle<midYaw+122)&&(GM6020_Yaw.Mechanical_angle>midYaw-122)))
		{
			Yaw_flag=0;
		}
		setM1Speed=setM2Speed=setM3Speed=setM4Speed=3000;
	}
	else
	{
	inputVxData=RC_Ctl.rc.ch[1] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)*100 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)*100; //输入 x轴 数据
	inputVyData=RC_Ctl.rc.ch[0] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)*80 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)*80;   //输入 y轴 数据
	inputWzData=RC_Ctl.rc.ch[2] + RC_Ctl.mouse.x*20;                                                                     //输入 z轴 数据
//	if(RC_Ctl.rc.s2 == 3) inputWzData=0;
	
	addVx=inputVxData*15;
	addVy=inputVyData*15;
	addWz=inputWzData*4;
	
	speedControl = (1 + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)*0.5) * (1 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL)*0.5);
	
	setVx = addVx * speedControl; 
	setVy = addVy * speedControl;
	setWz = addWz * speedControl;
	
	
	//假设左前方为ID 1 顺时针排序。
	setM1Speed = + setVx + setVy + setWz;  //设置 M1电机 速度 （遥控器发送底盘速度）
	setM2Speed = - setVx + setVy + setWz;  //设置 M2电机 速度
	setM3Speed = - setVx - setVy + setWz;  //设置 M3电机 速度
	setM4Speed = + setVx - setVy + setWz;  //设置 M4电机 速度
	}
}
////////////////////////////////////////////////////////////////////////////////////
void chassisFeedback(void)          //底盘 反馈     读取can2接收底盘数据   读取底盘数据
{
	nowM1Speed=M3508_1.Rotor_speed;   //现在 M1电机速度   can2 总线接收M1电机速度 （底盘反馈的此时此刻速度）
	nowM2Speed=M3508_2.Rotor_speed;   //现在 M2电机速度   can2 总线接收M2电机速度 
	nowM3Speed=M3508_3.Rotor_speed;   //现在 M3电机速度   can2 总线接收M3电机速度 
	nowM4Speed=M3508_4.Rotor_speed;   //现在 M4电机速度   can2 总线接收M4电机速度 
}

//（chassisPid 存入pid各个参数）
static void chassisPidInit(Chassis_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)  //底盘 pid 初始化
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

static float chassisPidContr0l(Chassis_PID_t *pid, float get, float set, u8 i)    //底盘 pid 控制   （pid算法）
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
void chassisSpeedSet(void)    //   底盘速度设置    通过 CAN2 发送  （通过pid算法得出 底盘将要设置的速度）
{
	setM1Speed = chassisPidContr0l(&chassisPid, nowM1Speed, setM1Speed, 1);//底盘 pid 控制      设置 M1电机速度 速度 
	setM2Speed = chassisPidContr0l(&chassisPid, nowM2Speed, setM2Speed, 2);//底盘 pid 控制      设置 M2电机速度 速度
	setM3Speed = chassisPidContr0l(&chassisPid, nowM3Speed, setM3Speed, 3);//底盘 pid 控制      设置 M3电机速度 速度
	setM4Speed = chassisPidContr0l(&chassisPid, nowM4Speed, setM4Speed, 4);//底盘 pid 控制      设置 M4电机速度 速度
	
	
	chassisSend(setM1Speed,setM2Speed,setM3Speed,setM4Speed);                //底盘发送     通过 CAN2 发送   底盘 M1 M2 M3 M4电机 速度 数据
//	chassisSend(1000,-1000,-1000,1000);
}

void chassisInit(void) //底盘初始化   chassisPid 初始化
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	chassisPidInit(&chassisPid, M3508_PID_MAX_OUT, M3508_PID_MAX_IOUT, M3508_PID_KP, M3508_PID_KI, M3508_PID_KD);//底盘 pid 初始化
	
}


void chassisTask(void)//底盘 任务
{
	chassisConversion();  //底盘 转换             //读取遥控器底盘设置数据
	chassisFeedback();    //底盘 反馈             //读取底盘数据 通过  CAN2  接收
	chassisSpeedSet();    //底盘 速度 设置        //将要设置底盘数据 通过  CAN2  发送
}

void chassis_task(void *pvParameters)
{
	
	//底盘电机初始化
	//delay_ms(50);
   chassisInit();
	
	while(1)
	{
		chassisTask();  
		vTaskDelay(10);
		
	}

}



