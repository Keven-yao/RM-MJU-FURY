#include "main.h"
#include "shoot.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
shoot_PID_t shootPid2006_1;                       //2006�������PID����

shoot_PID_t shootPid_5_6,shootPid_7;															//3508Ħ�����PID����

shoot_PID_t shootPid6020;													//6020�������PID����

int16_t inputShootData;														//����ķ����������

int16_t setShootSpeed=0;														//���ô���Ħ�����ٶ�

int16_t shootMotor2006_1,shootM4Speed;        		//�������ٶȺ͹������ٶ�

int16_t dErr_M3[3]={0};														//2��3508��1��6020 pid�е������

int16_t dErr_M2006; 															//2006����е�pid�������

int16_t nowM5Speed,nowM6Speed,nowM7Speed;										//2��3508��1��6020 pid�еĵ�ǰֵ��

int16_t nowGM4Speed;				

int16_t setM5Speed,setM6Speed,setM7Speed;										//��6020 pid�е��趨ֵ��

int16_t setGM4Speed;				
//int16_t nowAngle2006_1;														//2006����е�pid��ǰֵ��


//int16_t setAngle2006_1;														//2006����е�pid�趨ֵ��
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
	
void shootSend(u16 shootMotor1,u16 shootMotor2,u16 shootMotor3)
{
    u8 shootMotorData[6];

  	shootMotorData[0] = shootMotor1 >> 8;
    shootMotorData[1] = shootMotor1 & 0xFF;
    shootMotorData[2] = shootMotor2 >> 8;
    shootMotorData[3] = shootMotor2 & 0xFF;				//���Ƶ��idΪ 5��6  3508��� 
		shootMotorData[4] = shootMotor3 >> 8;
    shootMotorData[5] = shootMotor3 & 0xFF;
		CAN1_Send_Msg(0x1ff, shootMotorData, 0x06);    //���Ƶ��idΪ 1��2006 
}

void shootConversion(void)   //�����źŴ���ת��
{		
	setShootSpeed = 5000;  
//		if(RC_Ctl.rc.s1==1)     //
//	{		
//		
//		setShootSpeed = 5000;  
////		if (setShootSpeed>800) setShootSpeed -=66 ;   //Ħ���� ���� ����
////		else if (setShootSpeed<800) setShootSpeed =0; //Ħ���� �ٶ� ��Ϊ0	
//	}
//	else
//	{
//		
//	}	
	setM5Speed = setShootSpeed;   
	setM6Speed = -setShootSpeed;  //����Ħ����3508 ת����ͬ�������෴	
	
//	if(RC_Ctl.rc.ch[4]>100) setM7Speed = 6000;   //2006 ���� ��� ���� �ٶ� 																							 //˳ʱ ��Ϊ ��� ��ת
//	else setM7Speed = 0;
	if(RC_Ctl.rc.s1==2) 
	{
		setM7Speed = -1050;   //2006 ���� ��� ���� �ٶ� 																							 //˳ʱ ��Ϊ ��� ��ת
	}
	else if(RC_Ctl.rc.s1==1)
	{
		setM7Speed = 1000;
	}else {
		setM7Speed = 0;
	}
	
}

void shootFeedback(void)  //��������ٶȣ����ǽǶ�
{
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
		
//		printf("42Ħ����pid��ʼ�����\r\n");
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
    pid->Dout = pid->kd * (pid->err - dErr_M3[i]); //����pid�������ԣ�����΢��ֱ��Ϊ�������-�ϴ����
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
	setM5Speed = shootPidControl(&shootPid_5_6, nowM5Speed, setM5Speed, 1);//PID����M5������ 3508
	setM6Speed = shootPidControl(&shootPid_5_6, nowM6Speed, setM6Speed, 2);//PID����M6������ 3508
	setM7Speed = shootPidControl(&shootPid_7, nowM7Speed, setM7Speed, 3);
	shootSend(setM5Speed,setM6Speed,setM7Speed);
}



void shootInit(void)//�����ʼ��
{
	shootPidInit(&shootPid_5_6, SHOOT_M3508_5_6_PID_MAX_OUT, SHOOT_M3508_5_6_PID_MAX_IOUT, SHOOT_M3508_5_6_PID_KP, SHOOT_M3508_5_6_PID_KI, SHOOT_M3508_5_6_PID_KD);
	shootPidInit(&shootPid_7, SHOOT_M3508_7_PID_MAX_OUT, SHOOT_M3508_7_PID_MAX_IOUT, SHOOT_M3508_7_PID_KP, SHOOT_M3508_7_PID_KI, SHOOT_M3508_7_PID_KD);
}

void shootTask(void)//�������� �������к���
{
	shootFeedback();   //��� ����       //can1 ��ȡ����Ħ����3508�ٶ� can2 ��ȡ2006
	shootConversion(); //��� ת��       //s1���� ����3502��һ��6020������һ��2006���� ��ת��ͣ���Լ�ת�� 
	shootSpeedSet();   //��� �Ƕ� ����  //pid  
}
void shoot_task(void *pvParameters)
{
	shootInit();
	while(1)
	{
		
		shootTask();
		vTaskDelay(10);
		   
	}
	
	
}

