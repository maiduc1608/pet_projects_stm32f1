#include "myDHT.h"
#include "myDelay.h"

volatile GPIO_TypeDef *DHT_port;
static uint32_t DHT_pin;

static uint8_t humidity_integer;
static uint8_t humidity_decimal;
static uint8_t temperature_integer;
static uint8_t temperature_decimal;

static uint8_t checksum;
static double dht22_Data[2];
static double dht11_Data[2];

void DHT_setPort(char port){
	switch(port){
		case 'A':
			DHT_port = GPIOA;
			RCC->APB2ENR |= 1<<2; //en clock for GPIO A
			break;
		
		case 'B':
			DHT_port = GPIOB;
			RCC->APB2ENR |= 1<<3; //en clock for GPIO B
			break;
		case 'C':
			DHT_port = GPIOC;
			RCC->APB2ENR |= 1<<4; //en clock for GPIO C
			break;
	}
}

void DHT_setPin(uint8_t pin){
	DHT_pin = 1<<pin;
	if(pin<8){
		DHT_port->CRL &= ~0xF<<(pin*4);
		DHT_port->CRL |= 0x7<<(pin*4); //ouput open drain
		DHT_port->ODR |=DHT_pin;
	}
	else if(pin>=8){
		DHT_port->CRH &= ~0xF<<((pin-8)*4);
		DHT_port->CRH |= 0x7<<((pin-8)*4); //ouput open drain
		DHT_port->ODR |=DHT_pin;
	}
}

void SendStart(void){
	//send start signal and wait until DHT send data.
	DHT_port->ODR &= ~DHT_pin;
	Delay_ms(18);
	DHT_port->ODR |= DHT_pin;
	while ((DHT_port->IDR & DHT_pin) != 0);
	while ((DHT_port->IDR & DHT_pin) == 0);
  while ((DHT_port->IDR & DHT_pin) != 0);
}

uint8_t ReadDHT(void){
	uint8_t i;
	uint8_t data=0;

	for(i=0; i<8;i++){
		while((DHT_port->IDR & DHT_pin)==0); // wait until GPIO A high
		//use systick to determine the time of high level pinA.
		uint16_t tick_count=0;
		SysTick->LOAD = 9-1;
		SysTick->VAL = 0;
		SysTick->CTRL |= 1<<0;
		while((DHT_port->IDR & DHT_pin)!=0){
			if(SysTick->CTRL & 1<<16){
				tick_count++;
				SysTick->CTRL &= ~(1<<16);
			}
		}
		if(tick_count>60) data|=1<<(7-i);
	}
	return data;
}


double	*DHT22_Data(void){
	uint8_t i;
	SendStart();
	uint8_t dht_data[5];
	for(i=0;i<5;i++){
		dht_data[i] = ReadDHT();
	}
	humidity_integer = dht_data[0];
  	humidity_decimal = dht_data[1];
  	temperature_integer = dht_data[2];
  	temperature_decimal = dht_data[3];
  	checksum = dht_data[4];
		
//	if(checksum == humidity_integer + humidity_decimal + temperature_integer + temperature_decimal){
		dht22_Data[0] = (double)(((uint16_t)temperature_integer<<8) + (uint16_t)temperature_decimal)/10.0;
		dht22_Data[1] = (double)(((uint16_t)humidity_integer<<8) + (uint16_t)humidity_decimal)/10.0;
//	}
	return dht22_Data;
}

double	*DHT11_Data(void){
	uint8_t i;
	uint8_t dht_data[5];
	SendStart();
	for(i=0;i<5;i++){
		dht_data[i] = ReadDHT();
	}
	humidity_integer = dht_data[0];
  	humidity_decimal = dht_data[1];
  	temperature_integer = dht_data[2];
  	temperature_decimal = dht_data[3];
  	checksum = dht_data[4];
		
	if(checksum == humidity_integer + humidity_decimal + temperature_integer + temperature_decimal){
		dht11_Data[0] = (double)((uint16_t)temperature_integer) + (uint16_t)temperature_decimal/10.0;
		dht11_Data[1] = (double)((uint16_t)humidity_integer) + (uint16_t)humidity_decimal/10.0;
	}
	return dht11_Data;
}
