/*****************************************************
name:gimbal.c
brief:��̨�����ļ�
function:ͨ���յ���ң���źš��������Լ������ź���������̨��ת��
author:CadenLei
date:V1.0:2019.4.20
date:V2.0:2019.4.30 CadenLei Implementing PID algorithm.
date:V2.1:2020.1.12 Caden Ϊ�°�Ӣ�ۻ�����������룬��Ҫ�޸ģ�
										1��ȡ��С���跢�����
										2������һ��6020��̨pitch����
										3����������Ϊ6020�������ٴ�һ��2006ǹ����λ��������

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
//#define midYaw 4100        //yaw���е�λ��

//#define midPitch 2700         //pitch���е�λ��

int16_t inputYawData;          //ң������ ˮƽ �� ����
int16_t inputPitchData;        //ң������ ���� �� ����

int16_t ANflag=0; //���¼��̰�����ʶλ
//int16_t remote_flag; //ң��������������ʶ
float remote_gread=0.1;//С���ݵȼ�ϵ��
int16_t remote_move_flag=0;//�ж��Ƿ�ΪС����״̬���ƶ�ϵ��

int16_t setYaw=midYaw,setPitch=midPitch,setRoll=6800;              //�趨��λ�ó�ʼΪ�е�

int16_t addYaw,addPitch;															//yaw��������/pitch��������

int16_t leftLimit,rightLimit,upLimit,downLimit;       //����������λλ�ñ���

int16_t nowYaw,nowPitch,nowRoll,oldYaw;															//��ǰ�������λ��   ��can���߷����������ṩ

int16_t yawSpeed,pitchSpeed,rollSpeed,addspeed=-40;//speed=-42											//�����ٶ�

volatile float yaw_direction;

int16_t yaw_direction_flag;

Gimbal_PID_t gimbalPid_yaw,gimbalPid_pitch,gimbalPid_roll;           //������̨pid���ݽṹ

static float dErr_yaw = 0;														//yaw�����
static float dErr_pitch = 0;													//pitch�����
static float dErr_roll = 0;													//roll�����
float multiple=0.17f;

void test_keybord()
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)
	{
		ANflag=2;
		remote_gread=0.1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)
	{
		ANflag=3;
		remote_gread=0.08;
	}
	else if(RC_Ctl.rc.s2 !=1)
	{
		ANflag=1;
		remote_gread=0.05;
	}
		
}

void judje_mote_move()
{
	if(ANflag==2)
	{
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
		{
			remote_move_flag=1;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
		{
			remote_move_flag=1;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
		{
			remote_move_flag=1;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)
		{
			remote_move_flag=1;
		}
		else
			remote_move_flag=0;
	}
	else 
	{
		remote_move_flag=0;
	}
}

void top_speed() //С�������õ�λ�ٶ�
{
	//judje_mote_move();
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_C )
	{
		addspeed=-40 ;
		//if(remote_move_flag==1)
//		{
//			addspeed+=10;
//		}
		
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)
	{
		addspeed=-44;
//		if(remote_move_flag==1)
//		{
//			addspeed+=20;
//		}
		
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)
	{
		addspeed=-48;
//		if(remote_move_flag==1)
//		{
//			addspeed+=30;
//		}
	}
}



/*gimbal's signal data send by can1
**now yawMotor ID = 5   pitchMotor1 ID = 6   pitchMotor2 ID = 7*/
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3) //CAN1 ������̨����   �ȴ��룬�ٷ�
{
    u8 gimebalMotorData[6];
    gimebalMotorData[0] = chassisMotor1 >> 8;      //���� �߰�λ  ��� ����    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    gimebalMotorData[1] = chassisMotor1 & 0xFF;    //���� �Ͱ�λ  ��� ����                    0 1 2 3 4 5 6 7  
		gimebalMotorData[2] = chassisMotor2 >> 8;			 //���� �߰�λ  ��� ���� 	
    gimebalMotorData[3] = chassisMotor2 & 0xFF;    //���� �߰�λ  ��� ���� dd
		gimebalMotorData[4] = chassisMotor3 >> 8;			 //���� �߰�λ  ��� ���� 	
    gimebalMotorData[5] = chassisMotor3 & 0xFF;    //���� �߰�λ  ��� ���� 
		CAN1_Send_Msg(0x2FF, gimebalMotorData, 0x06); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
}

//���ڸĳ��Զ����!!!
void limitSet(void)//��λ     0 ~ 8191   8190/360=22.75
{
	leftLimit=  5100;             //  ˮƽ ���� 
	rightLimit=	3100;							//  ˮƽ ���� (5100-3100)/22.75=2000/22.72=88��C
	//(5100+3100)/2=4100 ˮƽ����   Ӳ�����Ӿ���
	upLimit=		2200;							//  ��ֱ ����
	downLimit=	2850;			  				//  ��ֱ ���� (7500-6830)/22.75=670/22.75=30��C 
	//(7500-6830)/2=7165 ��ֱ����   Ӳ�����Ӿ���	
}

/*************************************************
�ǶȻ���
��ȡ�ŵ����ݣ�����ת���ɽǶ�����ֵ
�ж��Ƿ񳬹���λֵ �����Ļ�������λֵ
**************************************************/
//extern volatile float yaw_angle,pitch_angle,roll_angle,gg_x,gg_y,gg_z,ga_x,ga_y,ga_z,ga_y1,kg,ga_y3,kg3,ga_y22,ga_y33;
void angleConversion(void)           //�Ƕ� ת��    ң�� ���� ��̨ ˮƽ�� ������ ����   
{
	test_keybord();
	top_speed();
	
	//printf("%c\r\n",RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q);
	TIM_Cmd(TIM2, ENABLE);	
	inputYawData=(RC_Ctl.rc.ch[2]*0.1 + RC_Ctl.mouse.x*20*remote_gread);
	inputPitchData= RC_Ctl.rc.ch[3]*5.0 + RC_Ctl.mouse.y*7;

	addYaw=inputYawData*0.1;                                         
	addPitch=inputPitchData*0.1;  //ң��������  
//		if(inputPitchData>0&&inputPitchData<500)
//		{
//			addPitch=3;
//		}
//		else if(inputPitchData>-500&&inputPitchData<0)
//		{
//			addPitch=-3;
//		}
//				else if(inputPitchData>500)
//		{
//			addPitch=10;
//		}
//		else if(inputPitchData<-500)
//		{
//			addPitch=-10;
//		}
/********************************************************************/   //Pitch 6020
  if (inputPitchData != 0)               //��ֱ ������λ
		{				
			if (setPitch+addPitch <= downLimit && setPitch+addPitch >= upLimit)  setPitch = setPitch + addPitch;
			else if (setPitch+addPitch > downLimit) setPitch = downLimit;
			else setPitch = upLimit;				
		}	
//		if (addPitch != 0)               //��ֱ ������λ
//		{
//			
//			if (nowPitch+addPitch <= downLimit && nowPitch+addPitch >= upLimit)  setPitch = nowPitch + addPitch;
//			else if (nowPitch+addPitch > downLimit) setPitch = downLimit;
//			else setPitch = upLimit;
//			
//		}	
	
/********************************************************************/   //Yaw 6020		
	if ((RC_Ctl.rc.s2 == 3)||(ANflag==3))            //ˮƽ ��λ
	{
		
		
		if(Yaw_flag==1)
		{
				setYaw+=addspeed+addYaw;
		}
		
		else if(Yaw_flag==0)
		{
			setYaw = midYaw;    //midYaw 4100   //yaw��  ˮƽ�� �е�λ�� 
		}			
	}
	else if((RC_Ctl.rc.s2 == 2)||(ANflag==2))               //ˮƽ ������
	{		
		Yaw_flag=1;	
		setYaw+=addspeed+addYaw;

	}
/********************************************************************/   //Roll 6020
//			setRoll = setRoll-(int16_t)(angle[1]*22.75f*0.05f);

	
	
}

//��ȡfeedback�еõ��ĵ���Ƕ�ֵ
void angleFeedback(void)   ////�Ƕ� ����    �� can1 ���� ���� ��̨ ˮƽ���� �� ����
{	
	nowYaw=GM6020_Yaw.Mechanical_angle;      //���� ˮƽ �� //////////����ȡ���ǽǶȣ������ٶȣ�
	nowPitch=GM6020_Pitch.Mechanical_angle;  //���� ���� ��
	nowRoll=GM6020_Roll.Mechanical_angle;		 //���� ƫ�� ��
		
}


static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)//��̨ pid ��ʼ�� 
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
		
		//printf("��̨pid��ʼ�����\r\n");
}
static float gimbalPidContr0l(Gimbal_PID_t *pid, float get, float set, float dErr)// ��̨ pid ����    pid�㷨  
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
    pid->Dout = pid->kd * (pid->err - dErr); //����pid�������ԣ�����΢��ֱ��Ϊ�������-�ϴ����
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr=pid->err;
		return pid->out;                          //��Ҫ���� ��̨ ˮƽ�Ƕ� ���� �� ����
}

//�趨ÿ�������ת������ٶ�
void speedSet(void)         //��̨ �Ƕ� ����    ��Ҫ���� ��̨ ˮƽ�� ������ ����  ��ͨ��pid�㷨�ó�) 
{
	if(nowYaw>6100&&oldYaw<2100)
			{
				setYaw=setYaw+8191;
			}
			
	else if(nowYaw<2100&&oldYaw>6100)
			{
				setYaw=setYaw-8191;
			}
	yawSpeed = gimbalPidContr0l(&gimbalPid_yaw, nowYaw, setYaw, dErr_yaw);          //pid����yaw�����      ˮƽ�Ƕ�
	pitchSpeed = gimbalPidContr0l(&gimbalPid_pitch, nowPitch, setPitch, dErr_pitch);//pid����pitch�����    �����Ƕ�
	//pitchSpeed=pitchSpeed+20000;
	//USART6_TX_Byte(pitchSpeed);
			//printf(pitchSpeed);
	rollSpeed = gimbalPidContr0l(&gimbalPid_roll, nowRoll, setRoll, dErr_roll);
//  printf("%d =%d = %d\r\n",nowYaw,yawSpeed,setYaw);
//			if(Yaw_flag==1&&(nowYaw<=599&&nowYaw>595))
//			{
//				yawSpeed-=2000;
//			}
//			if(Yaw_flag==1&&(nowYaw<=595&&nowYaw>590))
//			{
//				yawSpeed-=4000;
//			}
//			if(Yaw_flag==1&&(nowYaw<=590&&nowYaw>585))
//			{
//				yawSpeed-=8000;
//			}
//			if(Yaw_flag==1&&(nowYaw<=5855&&nowYaw>580))
//			{
//				yawSpeed-=8000;
//			}
//			else if(Yaw_flag==1&&(nowYaw<=580&&nowYaw>570))
//			{
//				yawSpeed-=8000;
//			}
//			else	if(Yaw_flag==1&&(nowYaw<=570&&nowYaw>560))
//			{
//				yawSpeed-=8000;
//			}
//			else	if(Yaw_flag==1&&(nowYaw<=560&&nowYaw>500))
//			{
//				yawSpeed-=8000;
//			}
	gimbalSend(yawSpeed,pitchSpeed,rollSpeed); //max speed=30000 min speed=-30000  //can1 ���� ��̨ ˮƽ���� ƫת ����
	oldYaw=nowYaw;
}


void gimbalInit(void) //��̨ pid ��ʼ��  gimbalPid_yaw ��̨ˮƽpid   gimbalPid_pitch ��̨����pid
{
								// ��Gimbal_PID_t *pid �� maxOut ��maxIout ��P ��I ��D��
	gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
	gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
	gimbalPidInit(&gimbalPid_roll, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD);
}


void gimbalTask(void)//��̨���� �������к���
{
	limitSet();        //���� ����     //����     ��̨ ˮƽ������λ ��ֱ������λ    ��λ�������ת����λ�ǶȺ� ���ٷ���װ������ʱ������ǶȲ���
	angleConversion(); //�Ƕ� ת��     //��ȡ     ң���� ��̨ ˮƽ����  ƫת ���� ����   
	angleFeedback();   //�Ƕ� ����     //��ȡ     ��̨ ˮƽ���� ƫת ����          can1 ���� ���� ��̨ ˮƽ���� ƫת ����
	speedSet();        //�Ƕ� ����     //��Ҫ���� ��̨ ˮƽƫת ����ƫת ����      can1 ���� ��̨ ˮƽ���� ƫת ����
}



void gimbal_Task(void *pvParameters)
{
	gimbalInit();
  	
	while(1)
	{
		gimbalTask();
		vTaskDelay(10);
	}
	
}


