#include "myDelay.h"
static volatile uint32_t timeTick = 0;

void Delay_us(uint32_t d) {
    timeTick = d;
    SysTick->LOAD = 9 - 1; // 1us delay
    SysTick->VAL = 0; 
    SysTick->CTRL |= 1ul<<1; //enable SysTick exception request.
		SysTick->CTRL |= 1ul<<0; //enable counter
    while(timeTick != 0);
    SysTick->CTRL = 0;
}

void Delay_ms(uint32_t d) {
    timeTick = d;
    SysTick->LOAD = 9000 - 1; // 1ms delay
    SysTick->VAL = 0;
    SysTick->CTRL |= 1ul<<1; //enable SysTick exception request.
		SysTick->CTRL |= 1ul<<0; //enable counter
    while(timeTick != 0);
    SysTick->CTRL = 0;
}

void SysTick_Handler(void) {
    if (timeTick > 0) {
        timeTick--;
    }
}