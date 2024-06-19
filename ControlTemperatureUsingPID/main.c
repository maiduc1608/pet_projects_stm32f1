#include<stm32f10x.h>
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include "mySystemClockConfig.h"
#include "myDelay.h"
#include "myUART.h"
#include "myPID.h"
#include "myLCD.h"

#define fanPin (1UL<<1)
#define mistPin (1UL<<2)

void PinConf(void);
void USART1_IRQHandler(void);

void SendStart(void);
uint8_t ReadDHT(void);

void convertToFormattedString(double*, char*);
void convertStringToDecimal(char*,double*);
void DHT_Display(void);
	
static uint8_t dht_data[5];
static uint8_t humidity_integer;
static uint8_t humidity_decimal;
static uint8_t temperature_integer;
static uint8_t temperature_decimal;
static double DHT_DATA[2];
static char DHT_format[9];
static double temp;
static double humid;
static uint8_t checksum;

static char uart_receive[5];
static uint8_t uart_index=0;
static double humid_setpoint = 76;
static double temp_setpoint = 30;
static uint8_t mode_control =0;
static char *t;
static char *h;
int main(void){
	SysClkConf_72MHz();
	PinConf();
	I2C1_Init();
	LCD_I2C_Init();
	LCD_I2C_Clear();
	myUART_Config();	
	myPWM_Config();
	setDutyCircle(1000);
	//wait 1s
	Delay_ms(1000);
	
	while(1){
		uint8_t i;
		SendStart();
		//collect raw data from DHT22.
		for(i=0;i<5;i++){
			dht_data[i] = ReadDHT();
		}
		humidity_integer = dht_data[0];
    humidity_decimal = dht_data[1];
    temperature_integer = dht_data[2];
    temperature_decimal = dht_data[3];
    checksum = dht_data[4];
		//handle raw data if checksum byte is valid.
		if(checksum == humidity_integer + humidity_decimal + temperature_integer + temperature_decimal){
				temp = (double)(((uint16_t)temperature_integer<<8) + (uint16_t)temperature_decimal)/10.0;
				humid = (double)(((uint16_t)humidity_integer<<8) + (uint16_t)humidity_decimal)/10.0;
			}
		DHT_DATA[0] = temp;
		DHT_DATA[1] = humid;
		DHT_Display();// show temp and humid on LCD
		
		if(mode_control==0){
			if(temp>temp_setpoint || humid>(humid_setpoint+2)) GPIOA->ODR |= fanPin;
			else GPIOA->ODR &= ~fanPin;
			if(humid<humid_setpoint-2) GPIOA->ODR |= mistPin;
			else GPIOA->ODR &= ~mistPin;
		}
		//send data in string form to labview via uart
		convertToFormattedString(DHT_DATA,DHT_format);
		uart_transmit_data(DHT_format);
		//control temperature.
		PID_Controller(temp_setpoint,temp); 
		//use led C13 to check when the system complete one loop
		GPIOC->ODR ^= 1<<13;
		Delay_ms(1000);
	}
	
}


void PinConf(void){
	RCC->APB2ENR |= 1<<2|1<<4; //en clock for GPIO A, C
	GPIOA->CRL &= ~0xFUL;
	GPIOA->CRL |= 0x7; //A0 ouput open drain to communicate with DHT22
	GPIOA->ODR |= 1<<0;
	GPIOA->CRL &= ~(0xFFUL<<4);
	GPIOA->CRL |= 0x33<<4; // A1,A2 output pp 50MHz to control fan and mist system
	GPIOC->CRH &= 0xF<<20;
	GPIOC->CRH |= 0x3<<20;
}

void SendStart(void){
	//send start signal and wait until DHT send data.
	GPIOA->ODR &= ~(1UL<<0);
	Delay_ms(20);
	GPIOA->ODR |= 1;
	while ((GPIOA->IDR & 1<<0) != 0);
	while ((GPIOA->IDR & 1<<0) == 0);
  while ((GPIOA->IDR & 1<<0) != 0);
}

uint8_t ReadDHT(void){
	uint8_t i;
	uint8_t data=0;

	for(i=0; i<8;i++){
		while((GPIOA->IDR & 1<<0)==0); // wait until GPIO A high
		//use systick to determine the time of high level pinA.
		uint16_t tick_count=0;
		SysTick->LOAD = 9-1;
		SysTick->VAL = 0;
		SysTick->CTRL |= 1<<0;
		while((GPIOA->IDR & 1<<0)!=0){
			if(SysTick->CTRL & 1<<16){
				tick_count++;
				SysTick->CTRL &= ~(1UL<<16);
			}
		}
		if(tick_count>50) data|=1<<(7-i);
	}
	return data;
}

void convertToFormattedString(double *Data, char *output) {
    int temperature = (int)(Data[0] * 10);
    int humidity = (int)(Data[1] * 10);
    sprintf(output, "%04d%04de", temperature, humidity);
}

void convertStringToDecimal(char* string, double* output){
	int te = atoi(string);
   *output = (double)te;
}

void USART1_IRQHandler(void){
	if(USART1->SR & 1<<5){
		char receive_char = USART1->DR;	
		uart_receive[uart_index]=receive_char;
		uart_index++;
		if(receive_char=='e'){
			uart_receive[4]=0;
			switch(uart_receive[0]){
				case 'h':
					convertStringToDecimal(uart_receive+1,&humid_setpoint);
					mode_control = 0;
					break;
				case 't':
					convertStringToDecimal(uart_receive+1,&temp_setpoint);
					mode_control=0;
					break;
				case 'g':
					if(uart_receive[3]=='f') GPIOA->ODR ^= fanPin;
					mode_control = 1;
					break;
				case 'm':
					if(uart_receive[3]=='f') GPIOA->ODR ^= mistPin;
					mode_control = 1;
					break;
			}
			uart_index=0;
		}
	}
}

void DHT_Display(void){
	LCD_I2C_Clear();
	//temp
	t = (char*)calloc(16,sizeof(char));
	strcpy(t,"Temp: ");
	char c_temp[4];
	sprintf(c_temp,"%.1f",temp);
	strcat(t,c_temp);
	strcat(t,"*C");
	LCD_I2C_Location(0,0);
	LCD_I2C_Write_String(t);
	free(t);
	//humid
	h = (char*)calloc(16,sizeof(char));
	strcpy(h,"Humid: ");
	char c_humid[4];
	sprintf(c_humid,"%.1f",humid);
	strcat(h,c_humid);
	strcat(h,"%");
	LCD_I2C_Location(1,0);
	LCD_I2C_Write_String(h);
	free(h);

}