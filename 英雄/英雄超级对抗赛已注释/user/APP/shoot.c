/**************************************
   
	   2021.8.7 
		 
		���ٵĿ��Ʒ���������̨�ǶȲ�ͬĦ����ת�ٲ�ͬ���а������Զ�Ħ���ֽ��м��ټ��ٵĲ������Ե��pid���޸�Ҳ����ʹ������������ 
		
   ��ǰ�汾Ӣ�ۻ�е�Ĺ����Ͳ��������кܴ����⣬��Ҫ��е�����޸ģ����޸ĵĹ�����õ����ó������Ե�����Ħ���ֵĿ��жȣ������Ҫ��е�͵��һ����
   
	 Ӣ�ۻ������ڱ���ʱ���Զ�ѡ������ǵ��̹������ȣ������������ :������ΪӢ�ۻ������ڱ������ȵķ�Χ�ڣ����̫���ˡ�


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



shoot_PID_t shootPid_5_6,shootPid_7;							//3508Ħ�����PID����
int16_t setShootSpeed=0;														//���ô���Ħ�����ٶ�
int16_t shootMotor2006_1,shootM4Speed;        		//�������ٶȺ͹������ٶ�
int16_t dErr_M3[3]={0};														//2��3508��1��6020 pid�е������
int16_t nowM5Speed,nowM6Speed,nowM7Speed;										//2��3508��1��6020 pid�еĵ�ǰֵ��	
int16_t setM5Speed,setM6Speed,setM7Speed,nowangle;										//��6020 pid�е��趨ֵ��
int16_t setGM4Speed;				
int flag_shoot = 0;
//��̨Pitchֵ����
int16_t nowPitch;
//��̨��������ֵ
int16_t  oldpitch;
double linear;//Ħ�������Ա仯
	

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

void shootConversion(void)   
{	

  static int addshootspeed = 0;	
	static int flag_key = 0;
	//����C V���Խ���Ħ���ּ��ټ���   �ڱ�����ʱ��Ϊ�˹���������һ������������ʼ����X���Դ��Ħ����ת�ٿ���
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
	setM6Speed = -addshootspeed - setShootSpeed;  //����Ħ����3508 ת����ͬ�������෴	
	setM7Speed = 0;
	
	//���Զ�����������Ҽ��˵���ң�����ұ�������2 �м�3 ������1
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



//����1 16 2. 
//Ӣ��1.10 2.
//�����Զ��壬ͨ����̨����ֵ��Ħ���ֽ��б���,ԭ���ǵ�ͷת��һ������©��
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


void shootFeedback(void)  //��������ٶȣ����ǽǶ�
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


//�����ʼ��
void shootInit(void)
{
	shootPidInit(&shootPid_5_6, SHOOT_M3508_5_6_PID_MAX_OUT, SHOOT_M3508_5_6_PID_MAX_IOUT, SHOOT_M3508_5_6_PID_KP, SHOOT_M3508_5_6_PID_KI, SHOOT_M3508_5_6_PID_KD);
	shootPidInit(&shootPid_7, SHOOT_M3508_7_PID_MAX_OUT, SHOOT_M3508_7_PID_MAX_IOUT, SHOOT_M3508_7_PID_KP, SHOOT_M3508_7_PID_KI, SHOOT_M3508_7_PID_KD);
}
//�������� �������к���
void shootTask(void)
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
		vTaskDelay(15);
		   
	}
	
	
}

