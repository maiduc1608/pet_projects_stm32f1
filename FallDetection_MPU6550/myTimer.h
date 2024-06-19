#ifndef MYTIMER_H
#define MYTIMER_H
#include <stm32f10x.h>

void TIM2_Setup(uint16_t prescaler, uint16_t ARR);
void TIM3_Setup(uint16_t prescaler, uint16_t ARR);
void TIM2_SetInterrupt(uint8_t priority);
void TIM2_DisInterrupt(void);
void TIM2_RedState(uint8_t status);
void TIM3_SetInterrupt(uint8_t priority);
void TIM3_DisInterrupt(void);
void TIM3_GreenState(uint8_t status);

#endif