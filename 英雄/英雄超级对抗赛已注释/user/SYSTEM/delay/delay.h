#ifndef __DELAY_H_
#define __DELAY_H_
#include "stm32f4xx.h"

void delay_Init(void);  //��ʱ��ʼ��
void delay_us(uint32_t nus);	//΢�뵥λ��ʱ
void delay_ms(uint16_t nms);	//���뵥λ��ʱ

#endif

