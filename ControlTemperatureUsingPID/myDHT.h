#ifndef	MYDHT_H
#define	MYDHT_H 


#include <stm32f10x.h>
void SendStart(void);
void DHT_setPort(char);
void DHT_setPin(uint8_t);
uint8_t ReadDHT(void);
double	*DHT11_Data(void);
double 	*DHT22_Data(void);
extern volatile GPIO_TypeDef *DHT_port;

#endif 