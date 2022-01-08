/*****************************************************
   2021.8.7 
   ��̨������������Ҫ 1.���Զ���̨�������Ƶ������Ⱥ;��ȣ�����ƶ�����Ұ����׼ȷ�ȶ��ƶ�����Ӧ����
	                    2.С����ģʽ����̨���ȶ���  �����ַ�ʽ 1.���� ���������ã� 2. imu�ȶ���д�������ȶ�����Ҫ�Ľ���
                      ��ν���٣����ǵ����ڶ���ʱ�����̨һ����ֵ���е������̴����������ھ���pid��С����
											imu��ͨ�����ذ��ӵĽǶȣ�ͨ���Ƕȱ仯��������̨�ȶ�


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

//pitch �ǶȻ� PID�����Լ� PID���������������
float PITCH_PID_KP =22.0f;
float PITCH_PID_KI= 0.00f;
float PITCH_PID_KD =10.00f;
float PITCH_PID_MAX_OUT= 20000.0f;
float PITCH_PID_MAX_IOUT =3000.0f;




int16_t ANflag=0; //���¼��̰�����ʶλ
int16_t inputYawData;          //ң������ ˮƽ �� ����
int16_t gyroYawData;          //С���������ƶ����� ˮƽ �� ����
int16_t inputPitchData;        //ң������ ���� �� ����

int16_t setYaw=midYaw,setPitch=midPitch;              //�趨��λ�ó�ʼΪ�е�

int16_t addYaw,addPitch;															//yaw��������/pitch��������

int16_t leftLimit,rightLimit,upLimit,downLimit;       //����������λλ�ñ���

int16_t nowYaw,nowRoll,oldYaw;															//��ǰ�������λ��   ��can���߷����������ṩ

//������Pitch���������Ϊ�ж���������
extern int16_t nowPitch;


int16_t yawSpeed,pitchSpeed,rollSpeed,gimbal_speed=85;//speed=-42											//�����ٶ�

volatile float yaw_direction;

int16_t yaw_direction_flag;

Gimbal_PID_t gimbalPid_yaw,gimbalPid_pitch,gimbalPid_roll;           //������̨pid���ݽṹ

static float dErr_yaw = 0;														//yaw�����
static float dErr_pitch = 0;													//pitch�����

float multiple=0.17f;

unsigned char send_bytes[16];
RecData  rec_data;

int current_level;//��¼��ǰ�����˵ĵȼ�



//�����Ӿ����ݺ���Э��
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



//С�������õ�λ�ٶ�
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
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3) //CAN1 ������̨����   �ȴ��룬�ٷ�
{
    u8 gimebalMotorData[6];
	
		gimebalMotorData[2] = chassisMotor2 >> 8;			 //���� �߰�λ  ��� ���� 	
    gimebalMotorData[3] = chassisMotor2 & 0xFF;    //���� �߰�λ  ��� ����
		CAN2_Send_Msg(0x2FF, gimebalMotorData, 0x04); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
	
		gimebalMotorData[0] = chassisMotor1 >> 8;      //���� �߰�λ  ��� ����    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    gimebalMotorData[1] = chassisMotor1 & 0xFF;    //���� �Ͱ�λ  ��� ����                    0 1 2 3 4 5 6 7  
		CAN1_Send_Msg(0x2FF, gimebalMotorData, 0x02); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
}

//����6020��������ĽǶȣ�����ÿ�������ŵ�λ����������λ
void limitSet(void)//��λ     0 ~ 8191   8190/360=22.75
{
	leftLimit=  5100;             //  ˮƽ ���� 
	rightLimit=	3100;							//  ˮƽ ���� (5100-3100)/22.75=2000/22.72=88��C
	//(5100+3100)/2=4100 ˮƽ����   Ӳ�����Ӿ���
	upLimit=		3300;							//  ��ֱ ����
	downLimit=	4250;							//  ��ֱ ���� (7500-6830)/22.75=670/22.75=30��C 
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
	
	TIM_Cmd(TIM2, ENABLE);
	
	//ң��ģʽ�µ�������
	inputPitchData= RC_Ctl.rc.ch[3]*5.0 + RC_Ctl.mouse.y*8;

	inputYawData=(RC_Ctl.rc.ch[2]*0.1 + RC_Ctl.mouse.x*5);
	
	//���Զ˿���������  - ֵ����ΪӢ�۵���Ƿ���װ������Ϊ-ֵ
	gyroYawData=-(RC_Ctl.rc.ch[2]*0.3 + RC_Ctl.mouse.x*5);
	
	

	addYaw=-inputYawData*0.1;                                         
	addPitch=inputPitchData*0.1;  //ң��������  
/********************************************************************/   //Pitch 6020
  if (inputPitchData != 0)               //��ֱ ������λ
		{				
			if (setPitch+addPitch <= downLimit && setPitch+addPitch >= upLimit)  setPitch = setPitch + addPitch;
			else if (setPitch+addPitch > downLimit) setPitch = downLimit;
			else setPitch = upLimit;				
		}	
	
	
/********************************************************************/   //Yaw 6020		
	if ((RC_Ctl.rc.s2 == 3)||(ANflag==3))            //ˮƽ ��λ
	{
		
		
		if(Yaw_flag==1)
		{
  		setYaw+=gimbal_speed+addYaw;
		}
		
		if(Yaw_flag==0)
		{
			yaw_direction_flag=1;
			inputYawData=0;
			setYaw = midYaw;    //midYaw 4100   //yaw��  ˮƽ�� �е�λ�� 
		}			
			pid_ctrl(1);
	}
	else if((RC_Ctl.rc.s2 == 2)||(ANflag==2))               //ˮƽ ������
	{		
	
		Yaw_flag=1;	
		setYaw+=gimbal_speed+gyroYawData*0.17;
		pid_ctrl(2);
	}else { 
			pid_ctrl(1);
	}

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
	

	gimbalSend(yawSpeed,pitchSpeed,rollSpeed); //max speed=30000 min speed=-30000  //can1 ���� ��̨ ˮƽ���� ƫת ����
	oldYaw=nowYaw;
}


void gimbalInit(void) //��̨ pid ��ʼ��  gimbalPid_yaw ��̨ˮƽpid   gimbalPid_pitch ��̨����pid
{
				
	gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
	gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);

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
		vTaskDelay(15);
	}
	
}

