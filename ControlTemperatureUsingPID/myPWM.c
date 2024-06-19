#include "myPWM.h"

void myPWM_Config(void){
	RCC->APB2ENR |= 0x04; //supply clock for GPIOA
	RCC->APB1ENR |= 0x02; //supply clock for TIM3

	//configure output A6
	GPIOA->CRL &= ~(0xFUL<<24);
	GPIOA->CRL |= 0xB<<24;
	
	//configure PWM.
	TIM3->PSC = 72-1;	//set prescaler.
	TIM3->ARR = 5000-1;	//set the period of auto reload register.
	TIM3->CCMR1 &= ~(0xFFUL<<0);
	TIM3->CCMR1 |= 0x6<<4; //PWM mode 1.
	TIM3->CCMR1 |= 0x8<<0;	//Output Compare 1 Preload En.

	TIM3->CR1 &= 0x01;
	TIM3->CR1 |= 0x01; //enable counter.

	TIM3->EGR |=0x01;
	TIM3->CCER |= 0x01; //enable OC1 of TIM3.
}

void setDutyCircle(uint16_t d){
		//set duty by percent of ARR value.
		TIM3->CCR1 = d;
	}