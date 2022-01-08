#include "mpu6500.h"
#include "mpu6500reg.h"
#include "main.h"
#include "spi.h"


#define MPU_SPI_READ_BIT 0x80

static uint8_t MPU_SPI_read_write_byte(uint8_t TxData);

void MPU_Init(void)
{
	MPU_Write_Byte(MPU_PWR_MGMT_1, 0X80);
	delay_ms(50);
	MPU_Write_Byte(MPU_PWR_MGMT_1, 0X00);
	MPU_Set_Gyro_Fsr(MPU_GYRO_RANGE_2000);
	MPU_Set_Accel_Fsr(MPU_ACCEL_RANGE_2G);
	MPU_Set_LPF(MPU_DLPF_CFG_4_SET);
	MPU_Set_Rate(50);
	MPU_Set_FIFO(MPU_FIFO_ALL_DISABLE);
	MPU_Write_Byte(MPU_INT_ENABLE, 0X00); //关闭所有中断
	MPU_Write_Byte(MPU_INTBP_CFG, 0X80);  //INT引脚低电平有效
	MPU_Write_Byte(MPU_PWR_MGMT_1, 0X01); //设置时钟源为PLL x轴参考
	MPU_Write_Byte(MPU_PWR_MGMT_2, 0X00); //所有轴都不待机
    MPU_Set_Rate(50);                     //例程如此，原因未知
}

void MPU_Read_Gyro(int16_t gyro[3])
{
	uint8_t buffer[6];
	MPU_Read_Len(MPU_GYRO_XOUT_H, buffer, 6);

	gyro[0] = (int16_t)((buffer[0]) << 8) | buffer[1];
	gyro[1] = (int16_t)((buffer[2]) << 8) | buffer[3];
	gyro[2] = (int16_t)((buffer[4]) << 8) | buffer[5];
}

void MPU_Read_Accel(int16_t accel[3])
{
	uint8_t buffer[6];
	MPU_Read_Len(MPU_ACCEL_XOUT_H, buffer, 6);

	accel[0] = (int16_t)((buffer[0]) << 8) | buffer[1];
	accel[1] = (int16_t)((buffer[2]) << 8) | buffer[3];
	accel[2] = (int16_t)((buffer[4]) << 8) | buffer[5];
}

void MPU_Read_Temp(int16_t *temp)
{
	uint8_t buffer[2];
	MPU_Read_Len(MPU_TEMP_OUT_H, buffer, 2);

	*temp = (int16_t)((buffer[0]) << 8) | buffer[1];
}

void MPU_Set_Gyro_Fsr(uint8_t gyro_range)
{
	MPU_Write_Byte(MPU_GYRO_CONFIG, gyro_range);
}

void MPU_Set_Accel_Fsr(uint8_t accel_range)
{
	MPU_Write_Byte(MPU_ACCEL_CONFIG, accel_range);
}

void MPU_Set_LPF(uint8_t dlpf_cfg)
{
	MPU_Write_Byte(MPU_CONFIG, dlpf_cfg);
}

void MPU_Set_Rate(uint8_t smplrt_div)
{
	MPU_Write_Byte(MPU_SMPLRT_DIV, smplrt_div);
}

void MPU_Set_FIFO(uint8_t fifo_enable)
{
	MPU_Write_Byte(MPU_FIFO_EN, fifo_enable);
}

void MPU_SPI_CS_L(void)
{
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
}

void MPU_SPI_CS_H(void)
{
	GPIO_SetBits(GPIOF, GPIO_Pin_6);
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	MPU_SPI_CS_L();
	MPU_SPI_read_write_byte(reg | MPU_SPI_READ_BIT);
	res = MPU_SPI_read_write_byte(0xFF);
	MPU_SPI_CS_H();
	return res;
}

void MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	MPU_SPI_CS_L();
	MPU_SPI_read_write_byte(reg);
	MPU_SPI_read_write_byte(data);
	MPU_SPI_CS_H();
}

void MPU_Read_Len(uint8_t reg, uint8_t *buf, uint8_t len)
{
	MPU_SPI_CS_L();
	MPU_SPI_read_write_byte(reg | MPU_SPI_READ_BIT);
	if (len != 0)
	{
		uint8_t i;
		for (i = 0; i < len; i++)
		{
			*buf = MPU_SPI_read_write_byte(0xFF);
			buf++;
		}
	}
	MPU_SPI_CS_H();
}

void MPU_Write_Len(uint8_t reg, uint8_t *buf, uint8_t len)
{
	MPU_SPI_CS_L();
	MPU_SPI_read_write_byte(reg);
	if (len != 0)
	{
		uint8_t i;
		for (i = 0; i < len; i++)
		{
			MPU_SPI_read_write_byte(*buf);
			buf++;
		}
	}
	MPU_SPI_CS_H();
}

static uint8_t MPU_SPI_read_write_byte(uint8_t TxData)
{
	uint8_t retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if (retry > 200)
		{
			return 0;
		}
	}
	SPI_I2S_SendData(SPI5, TxData);

	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;
		if (retry > 200)
		{
			return 0;
		}
	}
	return SPI_I2S_ReceiveData(SPI5);
}
