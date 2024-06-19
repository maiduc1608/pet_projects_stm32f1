#ifndef MYUART_H
#define MYUART_H

#include <stm32f10x.h>
void myUART_Config(void);
void uart_transmit_data(char*);
void USART1_IRQHandler(void);
#endif