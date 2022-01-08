#ifndef __SHOOT_H
#define __SHOOT_H

#include "pwm.h"

#define SHOOT_L PWM_GPIOA_0_SET
#define SHOOT_R PWM_GPIOA_1_SET

//M3508 速度环 PID参数以及 PID最大输出，积分输出
#define SHOOT_M3508_5_6_PID_KP 10.0f
#define SHOOT_M3508_5_6_PID_KI 2.01f
#define SHOOT_M3508_5_6_PID_KD 0.0f
#define SHOOT_M3508_5_6_PID_MAX_OUT 16000.0f
#define SHOOT_M3508_5_6_PID_MAX_IOUT 1000.0f

//#define SHOOT_M3508_7_PID_KP 15.0f
//#define SHOOT_M3508_7_PID_KI 0.01f
//#define SHOOT_M3508_7_PID_KD 10.0f
//#define SHOOT_M3508_7_PID_MAX_OUT 30000.0f
//#define SHOOT_M3508_7_PID_MAX_IOUT 8000.0f


#define pi 3.141592653589793238462643383f

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
#define Half_ecd_range 4096
#define ecd_range 8191


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
} shoot_PID_t;



void shootInit(void);
void shootTask(void);
void shootSend(u16 shootMotor1,u16 shootMotor2,u16 shootMotor3);
void playing_speed(void);
void Rate_of_fire_adaptive(void);
void shoot_task(void *pvParameters);
#endif
