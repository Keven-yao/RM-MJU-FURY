#ifndef _MPU6500_H
#define _MPU6500_H

#include "stm32f4xx.h"

void MPU_Init(void);

void MPU_Read_Gyro(int16_t gyro[3]);
void MPU_Read_Accel(int16_t accel[3]);
void MPU_Read_Temp(int16_t *temp);

void MPU_Set_Gyro_Fsr(uint8_t gyro_range);
void MPU_Set_Accel_Fsr(uint8_t accel_range);
void MPU_Set_LPF(uint8_t dlpf_cfg);
void MPU_Set_Rate(uint8_t smplrt_div);
void MPU_Set_FIFO(uint8_t fifo_enable);

void MPU_SPI_NS_L(void);
void MPU_SPI_NS_H(void);
uint8_t MPU_Read_Byte(uint8_t reg);
void MPU_Write_Byte(uint8_t reg, uint8_t data);
void MPU_Read_Len(uint8_t reg, uint8_t *buf, uint8_t len);
void MPU_Write_Len(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
