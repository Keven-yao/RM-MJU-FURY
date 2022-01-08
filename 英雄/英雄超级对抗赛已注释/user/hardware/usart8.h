#ifndef _USART5_H
#define _USART5_H

#include "main.h"

//#define    COM5_BUFFER_NUM           5
#define    JUDGE_BUFFER_LEN           200

//extern uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ];

void UART8_Init( void );
void UART8_DMA_Init( void );

//uint8_t *pucCom5ReadBuffer( void );
void UART8_SendChar( uint8_t cData );

//void UART5_UpdateMemoryAddr( DMA_InitTypeDef *xDMAInit, uint8_t *buffer );


#endif
