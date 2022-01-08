#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "stm32f4xx.h"

void CAP_Ctrl(void);
void CAP_Send(int16_t temPower);

void super_task(void *pvParameters);
#endif


