#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "sys.h"

void DR16_Init(void);
void USART1_TX_Byte(unsigned char data);
void usart2_Init(void);
void USART2_TX_Byte(unsigned char data);


#endif
