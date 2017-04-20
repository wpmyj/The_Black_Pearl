#ifndef __USART1_H
#define __USART1_H
#include "stm32f10x.h"
#include <stdio.h>

#define USART1_PRINTF


void USART_Configuration(void);
int fputc(int ch, FILE *f);
int GetKey(void);
void Uart1_PutChar(u8 ch);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
#endif