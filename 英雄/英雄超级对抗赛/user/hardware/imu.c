#include "imu.h"
#include "mpu6500.h"
#include "inv_mpu.h"
#include "main.h"
float angle_imu[3] = {0};
static float angle_zero[3] = {0};

void IMU_Init(void)
{
    MPU_Init();
    mpu_dmp_init();
}
//angle_imu[2] YAW   angle_imu[1]pitch
void IMU_Updata(void)
{
    float angle[3];
    if (mpu_dmp_get_data(&angle[0], &angle[1], &angle[2]))
        return;
    angle_imu[0] = angle[0] - angle_zero[0];
    angle_imu[1] = angle[1] - angle_zero[1];
    angle_imu[2] = angle[2] - angle_zero[2];
		//UART7_SendChar( angle_imu[2]);
		//led_red_on();
	  //printf("%9.4f %9.4f %9.4f\r\n", angle_imu[0], angle_imu[1], angle_imu[2]);
}

void IMU_SetZero()
{
    angle_zero[0] = angle_imu[0];
    angle_zero[1] = angle_imu[1];
    angle_zero[2] = angle_imu[2];
}
