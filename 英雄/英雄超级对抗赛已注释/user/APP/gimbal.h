#ifndef _GIMBAL_H_
#define _GIMBAL_H_
#include "stm32f4xx.h"
#include "stdbool.h"
#include "main.h"
//��ʼ�����λ��
#define midYaw 2770         //yaw���е�λ��
#define midPitch 4100         //pitch���е�λ��


typedef struct
{
    float kp;
    float ki;
    float kd; 

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} Gimbal_PID_t;
//�������ݸ��Ӿ�

/*
*   @brief:�����ܹ�����С���Ĺ�����
*/
typedef union{
    float f;
    unsigned char c[4];
}float2uchar;

typedef union{
    int16_t d;
    unsigned char c[2];
}int16uchar;

/*
*   @brief:������̨���ݵĽṹ��
*   @param:pitch_bit_   pitch������
*          yaw_bit_     yaw������
*/
typedef struct{
    float2uchar pitch_data;
    float2uchar yaw_data;
	  int16_t shoot_speed;   
    bool  IsBufMode;

}SendData;

/*
*   @brief:������̨���ݵĽṹ��
*   @param:pitch_bit_   pitch������
*          yaw_bit_     yaw������
*    12λ
*/
typedef struct{
	
    float2uchar Rec_pitch_data;
    float2uchar Rec_yaw_data;
	  bool  Is_Identify;       //�Ƿ�ʶ��Ŀ��
	  unsigned char distance;  //��Ŀ�����
	  bool Is_shoot;           //�Ƿ����ӵ�

}RecData;


extern SendData s;

//��̨��������
void gimbal_Task(void *pvParameters);
//��̨����
void gimbalTask(void);
//��̨��ʼ��
void gimbalInit(void);
//��̨���ݷ���
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3);
//�ֵ�pid����
void pid_ctrl(int pid_flag);

extern volatile float angle_roll_pitch[3];
void sendDatas(SendData stm32data);
//���麯��
void Since_the_aim(RecData  rec);
static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd);//��̨ pid ��ʼ�� 
#endif
