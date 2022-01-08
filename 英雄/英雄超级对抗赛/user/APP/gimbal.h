#ifndef _GIMBAL_H_
#define _GIMBAL_H_
#include "stm32f4xx.h"
#include "stdbool.h"
//#include "sys.h"
#include "main.h"

void gimbal_Task(void *pvParameters);

void gimbalTask(void);
void gimbalInit(void);
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3);

void pid_ctrl(int pid_flag);




extern volatile float angle_roll_pitch[3];

#define midYaw 2770    



//roll 角度环 PID参数以及 PID最大输出，积分输出
#define ROLL_PID_KP 40.0f
#define ROLL_PID_KI 2.50f
#define ROLL_PID_KD 20.020f
#define ROLL_PID_MAX_OUT 20000.0f
#define ROLL_PID_MAX_IOUT 1000.0f

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

void sendDatas(SendData stm32data);
//自瞄函数
void Since_the_aim(RecData  rec);
static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd);//云台 pid 初始化 
#endif
