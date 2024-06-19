#include "myDelay.h"
void Delay_us(uint32_t d){
	uint32_t i;
	for(i=0;i<d;i++){
		SysTick->LOAD = 9-1;
		SysTick->VAL = 0;
		SysTick->CTRL |= 1<<0;
		while(!(SysTick->CTRL & 1<<16));
		SysTick->CTRL &= ~(1<<0);
	}
}

void Delay_ms(uint32_t d){
	uint32_t i;
	for(i=0;i<d;i++){
		SysTick->LOAD = 9000-1;
		SysTick->VAL = 0;
		SysTick->CTRL |= 1<<0;
		while(!(SysTick->CTRL & 1<<16));
		SysTick->CTRL &= ~(1<<0);
	}
}