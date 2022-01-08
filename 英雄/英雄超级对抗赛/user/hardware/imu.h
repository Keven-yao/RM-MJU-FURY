#ifndef _IMU_H
#define _IMU_H

#include "stm32f4xx.h"

extern float angle_imu[3]; //pitch, roll, yaw

void IMU_Init(void);
void IMU_Updata(void);
void IMU_SetZero(void);

#endif

