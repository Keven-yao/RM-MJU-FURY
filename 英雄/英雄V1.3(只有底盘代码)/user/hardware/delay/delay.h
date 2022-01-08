#ifndef DELAY_H
#define DELAY_H
#include "main.h"
#include "stm32f4xx.h"

void delay_Init(void);  //延时初始化
void delay_us(uint32_t nus);	//微秒单位延时
void delay_ms(uint16_t nms);	//毫秒单位延时
#endif

