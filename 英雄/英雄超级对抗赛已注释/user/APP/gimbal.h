#ifndef _GIMBAL_H_
#define _GIMBAL_H_
#include "stm32f4xx.h"
#include "stdbool.h"
#include "main.h"
//初始化电机位置
#define midYaw 2770         //yaw轴中点位置
#define midPitch 4100         //pitch轴中点位置


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
//发送数据给视觉

/*
*   @brief:创建能够发送小数的共用体
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
*   @brief:发送云台数据的结构体
*   @param:pitch_bit_   pitch轴数据
*          yaw_bit_     yaw轴数据
*/
typedef struct{
    float2uchar pitch_data;
    float2uchar yaw_data;
	  int16_t shoot_speed;   
    bool  IsBufMode;

}SendData;

/*
*   @brief:接收云台数据的结构体
*   @param:pitch_bit_   pitch轴数据
*          yaw_bit_     yaw轴数据
*    12位
*/
typedef struct{
	
    float2uchar Rec_pitch_data;
    float2uchar Rec_yaw_data;
	  bool  Is_Identify;       //是否识别目标
	  unsigned char distance;  //与目标距离
	  bool Is_shoot;           //是否发射子弹

}RecData;


extern SendData s;

//云台启动任务
void gimbal_Task(void *pvParameters);
//云台任务
void gimbalTask(void);
//云台初始化
void gimbalInit(void);
//云台数据反馈
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3);
//分档pid函数
void pid_ctrl(int pid_flag);

extern volatile float angle_roll_pitch[3];
void sendDatas(SendData stm32data);
//自瞄函数
void Since_the_aim(RecData  rec);
static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd);//云台 pid 初始化 
#endif
