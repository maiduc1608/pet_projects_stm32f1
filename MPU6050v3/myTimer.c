#include "myTimer.h"

void TIM2_Setup(uint16_t prescaler, uint16_t ARR){
	RCC->APB1ENR |= 1UL<<0; //en clk tim2
	TIM2->PSC = prescaler-1;
	TIM2->ARR = ARR-1;
	TIM2->DIER |= 1; //interrupt en
}

void TIM2_SetInterrupt(uint8_t priority){
	NVIC_SetPriority(TIM2_IRQn,priority);
	NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_DisInterrupt(void){
	NVIC_DisableIRQ(TIM2_IRQn);
}

void TIM2_RedState(uint8_t status){
	if(status==0) TIM2->CR1 &= ~(1U<<0);
	else TIM2->CR1 |= 1;
}

void TIM3_Setup(uint16_t prescaler, uint16_t ARR){
	RCC->APB1ENR |= 1UL<<1; //en clk tim2
	TIM3->PSC = prescaler-1;
	TIM3->ARR = ARR-1;
	TIM3->DIER |= 1; //interrupt en
}

void TIM3_SetInterrupt(uint8_t priority){
	NVIC_SetPriority(TIM3_IRQn,priority);
	NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_DisInterrupt(void){
	NVIC_DisableIRQ(TIM3_IRQn);
}

void TIM3_GreenState(uint8_t status){
	if(status==0) TIM3->CR1 &= ~(1U<<0);
	else TIM3->CR1 |= 1;
}