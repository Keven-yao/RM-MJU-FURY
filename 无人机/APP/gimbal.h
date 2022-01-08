#ifndef _GIMBAL_H_
#define _GIMBAL_H_
#include "stm32f4xx.h"
#include "sys.h"

void gimbalTask(void);
void gimbalInit(void);
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2);

//yaw 角度环 PID参数以及 PID最大输出，积分输出
#define YAW_PID_KP 20.0f
#define YAW_PID_KI 0.05f
#define YAW_PID_KD 0.000f
#define YAW_PID_MAX_OUT 20000.0f
#define YAW_PID_MAX_IOUT 8000.0f

//pitch 角度环 PID参数以及 PID最大输出，积分输出
#define PITCH_PID_KP 20.0f
#define PITCH_PID_KI 0.05f
#define PITCH_PID_KD 0.000f
#define PITCH_PID_MAX_OUT 20000.0f
#define PITCH_PID_MAX_IOUT 8000.0f


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
















#endif
