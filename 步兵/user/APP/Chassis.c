#include "stm32f4xx.h"
//#include "sys.h"


#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


//С����ƽ��������ͽǶ�ƫ������
//#define chassisRotationProportion 0.75f
float chassisRotationProportion =0.75;
//#define chassisRotationProportion 1.0f
#define chassisAngleOffset -1.0f
#define PI 3.14159265358979f
int16_t chassisRotateSpeed = 0;

int16_t inputVxData; 
int16_t inputVyData;
int16_t inputWzData;
int16_t Yaw_flag=0;
float gears_flag=1;//�ٶȵ�λ����

float Chassis_flag=1;//���̳���������ϵ��

int16_t setVx,setVy,setWz;

int16_t setW_S_A_Dspeed =180;//���õ����ٶ�ϵ��

int16_t set_mouse_speed=10; //��������ƶ�ϵ��

int16_t addVx,addVy,addWz;

int16_t nowM1Speed,nowM2Speed,nowM3Speed,nowM4Speed;
int16_t setM1Speed,setM2Speed,setM3Speed,setM4Speed;

int16_t A_D_speed,W_S_speed;

float speedControl;

Chassis_PID_t chassisPid;
static float dErr_M[5] = {0};



//��can���߷������������ٶȵĿ�������
void chassisSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3, int16_t chassisMotor4) //���� ����
{
    u8 chassisMotorData[8];   // ���� ��� ����
    chassisMotorData[0] = chassisMotor1 >> 8;     //���� �߰�λ ���� ��� ����    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    chassisMotorData[1] = chassisMotor1 & 0xFF;   //���� �Ͱ�λ ���� ��� ����                    0 1 2 3 4 5 6 7
    chassisMotorData[2] = chassisMotor2 >> 8;
    chassisMotorData[3] = chassisMotor2 & 0xFF;
    chassisMotorData[4] = chassisMotor3 >> 8;
    chassisMotorData[5] = chassisMotor3 & 0xFF;
    chassisMotorData[6] = chassisMotor4 >> 8;
    chassisMotorData[7] = chassisMotor4 & 0xFF;
    CAN1_Send_Msg(0x200, chassisMotorData, 0x08);   //can2���͵��� �ĸ� ��� ����
}

void fix_speed()  //�������ó�ʼ�ٶ�
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)
	{
		A_D_speed=1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
	{
		A_D_speed=-1;
	}
	else A_D_speed=0;
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
	{
		W_S_speed=1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
	{
		W_S_speed=-1;
	}
	else W_S_speed=0;
		
}

void gears_speed() //�������õ�λ�ٶ�
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_C )
	{
		gears_flag=1;
		Chassis_flag=1;
		chassisRotationProportion=0.75;
		//chassisRotationProportion=0.85;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)
	{
		gears_flag=1.2;
		Chassis_flag=1.10;
		chassisRotationProportion=1.0;
		//chassisRotationProportion=1.1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)
	{
		gears_flag=1.3;
		Chassis_flag=1.2;
		chassisRotationProportion=1.2;
		//chassisRotationProportion=1.3;
	}
	
	
}
//////////////////////////////////////////////////////////////////////////////////
void chassisConversion(void)//���� ת��  ��ȡң����������������
{
	fix_speed();
	gears_speed();
	if (Yaw_flag == 1)
	{
		if (((RC_Ctl.rc.s2 == 3)||(ANflag==3)) && ((GM6020_Yaw.Mechanical_angle < midYaw + 122) && (GM6020_Yaw.Mechanical_angle > midYaw - 122)))
		{
			Yaw_flag = 0;
		}
		
		
		chassisRotateSpeed = setM1Speed = setM2Speed = setM3Speed = setM4Speed = 2822*Chassis_flag; //Ĭ�ϵ���ת

		float angleYaw = GM6020_Yaw.Mechanical_angle / 8192.0f;
		angleYaw = 1.0f + angleYaw + chassisAngleOffset; //��������ϵ�µ�ǰ����      ���̼�����ϵ�Ժ�Ϊ������˳ʱ�����
		float dir_x = RC_Ctl.rc.ch[0]+A_D_speed*setW_S_A_Dspeed*gears_flag;  //���뷽���������������������
		float dir_z = RC_Ctl.rc.ch[1]+W_S_speed*setW_S_A_Dspeed*gears_flag;//���뷽����������ǰ���������
		float dir_scale = fmaxf(fabsf(dir_x), fabsf(dir_z)) / sqrtf(dir_x * dir_x + dir_z * dir_z); //��׼��������뷽�������ĳ���
		if (isnan(dir_scale))
			dir_scale = 0.0f;
		float dir_angle = 0.25f - atan2f(dir_z, dir_x) / 2 / PI; //���뷽�������ķ����
		if (isnan(dir_angle))
			dir_angle = 0.0f;
		angleYaw += dir_angle; //ֱ�Ӱ�����ķ�����ӵ���������ϵ�µ�ǰ�����ϣ��õ���������ϵ��Ҫǰ���ķ���

		int16_t i, j;
		wheelRotate_t wheels[4] = {{0.375f, &setM1Speed}, {0.625f, &setM2Speed}, {0.875f, &setM3Speed}, {0.125f, &setM4Speed}}; //ID�ֱ�Ϊ1��2��3��4������
		for (i = 0; i < 4; i++) //�����������ϵ��ÿ�����ӵ�ʩ������
		{
			wheels[i].angle = wheels[i].angle - angleYaw + 1.5f;
			while (wheels[i].angle > 1.0f)
				wheels[i].angle -= 1.0f;
		}
		for (i = 0; i < 3; i++) //��ʩ��������������
		{
			for (j = 0; j < 3 - i; j++)
			{
				wheelRotate_t tmp;
				if (wheels[j].angle > wheels[j + 1].angle)
				{
					tmp = wheels[j];
					wheels[j] = wheels[j + 1];
					wheels[j + 1] = tmp;
				}
			}
		}

		float angleMin = wheels[0].angle, angleMin2Pi = angleMin * PI * 2; //����ÿ������Ҫ��/������
		float angleSin = sinf(angleMin2Pi), angleCos = cosf(angleMin2Pi);
		if (angleMin >= 0.125f)
		{
			float forceLarge = chassisRotationProportion * dir_scale / (angleSin + angleCos * angleCos / angleSin);
			float forceSmall = forceLarge / angleSin * angleCos;

			*(wheels[0].force) *= 1 + forceLarge; //���ӵ���ת���ϵõ���ת��ǰ�ĺ���
			*(wheels[1].force) *= 1 + forceSmall;
			*(wheels[2].force) *= 1 - forceLarge;
			*(wheels[3].force) *= 1 - forceSmall;
		}
		else
		{
			float forceLarge = chassisRotationProportion * dir_scale / (angleCos + angleSin * angleSin / angleCos);
			float forceSmall = forceLarge * angleSin / angleCos;

			*(wheels[0].force) *= 1 + forceSmall;
			*(wheels[1].force) *= 1 + forceLarge;
			*(wheels[2].force) *= 1 - forceSmall;
			*(wheels[3].force) *= 1 - forceLarge;
		}
	}
	else
	{
//	inputVxData=RC_Ctl.rc.ch[1] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)*100 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)*100; //���� x�� ����
//	inputVyData=RC_Ctl.rc.ch[0] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)*80 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)*80;   //���� y�� ����
	inputWzData=RC_Ctl.rc.ch[2] + RC_Ctl.mouse.x*set_mouse_speed;  		//���� z�� ����
	inputVxData=RC_Ctl.rc.ch[1]+W_S_speed*setW_S_A_Dspeed*gears_flag;
	inputVyData=RC_Ctl.rc.ch[0]+A_D_speed*setW_S_A_Dspeed*gears_flag;
	
		
	addVx=inputVxData*15;
	addVy=inputVyData*15;
	addWz=inputWzData*4;
	
	speedControl = (1 + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)*0.5) * (1 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL)*0.5);
	
	setVx = addVx * speedControl; 
	setVy = addVy * speedControl;
	setWz = addWz * speedControl;
	
	
	//������ǰ��ΪID 1 ˳ʱ������
	
	setM1Speed = + setVx + setVy + setWz;  //���� M1��� �ٶ� ��ң�������͵����ٶȣ�
	setM2Speed = - setVx + setVy + setWz;  //���� M2��� �ٶ�
	setM3Speed = - setVx - setVy + setWz;  //���� M3��� �ٶ�
	setM4Speed = + setVx - setVy + setWz;  //���� M4��� �ٶ�	
	}
}
////////////////////////////////////////////////////////////////////////////////////
void chassisFeedback(void)          //���� ����     ��ȡcan2���յ�������   ��ȡ��������
{
	nowM1Speed=M3508_1.Rotor_speed;   //���� M1����ٶ�   can2 ���߽���M1����ٶ� �����̷����Ĵ�ʱ�˿��ٶȣ�
	nowM2Speed=M3508_2.Rotor_speed;   //���� M2����ٶ�   can2 ���߽���M2����ٶ� 
	nowM3Speed=M3508_3.Rotor_speed;   //���� M3����ٶ�   can2 ���߽���M3����ٶ� 
	nowM4Speed=M3508_4.Rotor_speed;   //���� M4����ٶ�   can2 ���߽���M4����ٶ� 
}

//��chassisPid ����pid����������
static void chassisPidInit(Chassis_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)  //���� pid ��ʼ��
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
		
//		printf("����pid��ʼ�����\r\n");
}

static float chassisPidContr0l(Chassis_PID_t *pid, float get, float set, u8 i)    //���� pid ����   ��pid�㷨��
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
    pid->Dout = pid->kd * (pid->err - dErr_M[i]); //����pid�������ԣ�����΢��ֱ��Ϊ�������-�ϴ����
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M[i]=pid->err;
		return pid->out;
}

//�趨ÿ�������ת������ٶ�
void chassisSpeedSet(void)    //   �����ٶ�����    ͨ�� CAN2 ����  ��ͨ��pid�㷨�ó� ���̽�Ҫ���õ��ٶȣ�
{
	setM1Speed = chassisPidContr0l(&chassisPid, nowM1Speed, setM1Speed, 1);//���� pid ����      ���� M1����ٶ� �ٶ� 
	setM2Speed = chassisPidContr0l(&chassisPid, nowM2Speed, setM2Speed, 2);//���� pid ����      ���� M2����ٶ� �ٶ�
	setM3Speed = chassisPidContr0l(&chassisPid, nowM3Speed, setM3Speed, 3);//���� pid ����      ���� M3����ٶ� �ٶ�
	setM4Speed = chassisPidContr0l(&chassisPid, nowM4Speed, setM4Speed, 4);//���� pid ����      ���� M4����ٶ� �ٶ�
	
	
	chassisSend(setM1Speed,setM2Speed,setM3Speed,setM4Speed);                //���̷���     ͨ�� CAN2 ����   ���� M1 M2 M3 M4��� �ٶ� ����
//	chassisSend(1000,-1000,-1000,1000);
}

void chassisInit(void) //���̳�ʼ��   chassisPid ��ʼ��
{
								// ��Gimbal_PID_t *pid �� maxOut ��maxIout ��P ��I ��D��
	chassisPidInit(&chassisPid, M3508_PID_MAX_OUT, M3508_PID_MAX_IOUT, M3508_PID_KP, M3508_PID_KI, M3508_PID_KD);//���� pid ��ʼ��
	
}


void chassisTask(void)//���� ����
{
	chassisConversion();  //���� ת��             //��ȡң����������������
	chassisFeedback();    //���� ����             //��ȡ�������� ͨ��  CAN2  ����
	chassisSpeedSet();    //���� �ٶ� ����        //��Ҫ���õ������� ͨ��  CAN2  ����
}

void chassis_task(void *pvParameters)
{
	
	//���̵����ʼ��
	//delay_ms(50);
   chassisInit();
	
	while(1)
	{
		chassisTask();  
		vTaskDelay(10);
		
	}

}



