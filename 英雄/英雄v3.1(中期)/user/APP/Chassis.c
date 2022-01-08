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


//////////////////////////////////////////////////////////////////////////////////
void chassisConversion(void)//���� ת��  ��ȡң����������������
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
	inputVxData=RC_Ctl.rc.ch[1] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)*100 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)*100; //���� x�� ����
	inputVyData=RC_Ctl.rc.ch[0] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)*80 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)*80;   //���� y�� ����
	inputWzData=RC_Ctl.rc.ch[2] + RC_Ctl.mouse.x*20;                                                                     //���� z�� ����
//	if(RC_Ctl.rc.s2 == 3) inputWzData=0;
	
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



