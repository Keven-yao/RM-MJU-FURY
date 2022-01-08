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

int16_t setM5Speed,setM6Speed,setM7Speed,nowangle;										//��6020 pid�е��趨ֵ��

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
//��̨Pitchֵ����
int16_t nowPitch;
//��̨��������ֵ
int16_t  oldpitch;
double linear;//Ħ�������Ա仯
#define Dial_linear   0.05f//������

int yigeyige(int a,int16_t *Speed,int16_t now_angle)
{
	static int  clock=clock_num;
	static int  unclock=unclock_num;  //speed>0 ??����??   ��y?�̡䨮D?��?�䨮
	static int16_t old_angle=4000;
	
				if(unclock > 0)  
				{
					*Speed =  unclock_speed;		//??����??
				}			
				else if(clock>0) 
				{
					*Speed = -clock_speed;	//  ?3����??
				}
				else
				{
					a  = 1 ; 
					*Speed = 0 ;
					clock = clock_num;
					unclock = unclock_num;
				}				
				
				if(old_angle<2000&&now_angle>6000)    //?D?? ?3����?? ��??����a?����??��
				{
					clock--;
				}
				 if(old_angle>6000&&now_angle<2000)   //?D?? ??����?? ��??����a?����??��
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
    shootMotorData[3] = shootMotor2 & 0xFF;				//���Ƶ��idΪ 5��6  3508��� 
		shootMotorData[4] = shootMotor3 >> 8;
    shootMotorData[5] = shootMotor3 & 0xFF;
		CAN1_Send_Msg(0x1ff, shootMotorData, 0x06);    //���Ƶ��idΪ 1��2006 
}

void shootConversion(void)   //�����źŴ���ת��
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
	setM6Speed = -addshootspeed - setShootSpeed;  //����Ħ����3508 ת����ͬ�������෴	
	setM7Speed = 0;
	
	//4200��8.3����9.1 4420 ��8.9~9.8
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

//��������
void playing_speed(void)
{
	//	static int flag_flag=1;
//	if( RC_Ctl.mouse.press_l==1 && flag_flag == 1 )//������??-?������?��???����?o��?��?����y???a?T1??��???����?o��?��?����??T???a1?��??T??��������??��????T?��
//	{
//		flag_flag = 0 ;
//		setM7Speed= 0 ;
//	}
//	if(flag_flag == 0)
//	{
//		flag_flag=yigeyige(flag_flag,&setM7Speed,nowangle);  //speed>0 ??����??   ��y?�̡䨮D?��?�䨮	
//	}

//	if(flag_flag==1)
//	{
//		setM7Speed=0;
//	}
//	if((RC_Ctl.rc.s1==3)||(RC_Ctl.mouse.press_r==1))
//	{
//		setM7Speed = 1000;
//	}
		//�ټ�һ����������
}

//����1 16 2. 
//Ӣ��1.10 2.
//�����Զ��壬���ݵ�ǰ�����˵ȼ���Ŀǰ���ٱ��趨ֵ��ƫ����иı�
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
		vTaskDelay(15);
		   
	}
	
	
}

