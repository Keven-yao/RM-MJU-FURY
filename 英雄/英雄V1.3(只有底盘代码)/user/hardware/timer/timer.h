#ifndef __TIME_H
#define __TIME_H

#include "stm32f4xx.h"
#include "sys.h"
#include "time.h"

extern unsigned long long sysTime;      //ʱ��� ��λms

void TIM3_Int_Init(void);
void TIM4_Int_Init(void);

#endif
