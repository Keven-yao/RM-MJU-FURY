#ifndef _USART4_H
#define _USART4_H

#include "main.h"



/* 接收缓存 */
#define    VISION_BUFFER_LEN           100   //稍微给大点

//extern uint8_t  Com4_Vision_Buffer[ VISION_BUFFER_LEN ];


void UART7_Init(void);
void DMA1_Stream3_IRQHandler(void);
void show(void);
void UART7_SendChar(uint16_t cData);
void UART7_DMA_Init(void);
#endif
