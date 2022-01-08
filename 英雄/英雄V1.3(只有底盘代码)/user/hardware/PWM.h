#ifndef __PWM_H
#define __PWM_H

#include "stm32f4xx.h"
#include "sys.h"


#define Fric_UP 1250
#define Fric_DOWN 1150
#define Fric_OFF 1000

extern void PWM_Init(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif

