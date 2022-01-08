#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stm32f4xx.h"
//#include "sys.h"
#include "main.h"
#include "chassis.h"
extern int16_t Yaw_flag;
extern int16_t ANflag;
extern float gears_flag;
void chassis_task(void *pvParameters);
void chassisInit(void);
void chassisTask(void);
void chassisSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3, int16_t chassisMotor4);




typedef struct
{
	float angle;
	int16_t *force;
} wheelRotate_t;

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
} Chassis_PID_t;

static void chassisPidInit(Chassis_PID_t *chassisPid, float maxout, float max_iout, float kp, float ki, float kd);//‘∆Ã® pid ≥ı ºªØ 
void chassis_ctrl(int pid_flag);
#endif
