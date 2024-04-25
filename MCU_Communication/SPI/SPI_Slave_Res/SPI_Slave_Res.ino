#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LEDpin 7
#define buttonpin 2

volatile bool received;
volatile uint8_t Slavereceived, Slavesend;

int buttonvalue;
int x;

void SPI_SlaveInit() {
    // Set MISO as output pin
    DDRB |= (1 << DDB4);
    
    // Enable SPI in Slave Mode
    SPCR |= (1 << SPE);
    
    // Enable SPI Interrupt
    SPCR |= (1 << SPIE);
}

ISR(SPI_STC_vect) {
    // SPI Interrupt Service Routine
    Slavereceived = SPDR;
    received = true;
}

void setup() {
    Serial.begin(9600);

    // Set button pin as input
    pinMode(buttonpin, INPUT);
    
    // Set LED pin as output
    pinMode(LEDpin, OUTPUT);
    
    SPI_SlaveInit();
}

void loop() {
    if (received) {
        if (Slavereceived == 1) {
            digitalWrite(LEDpin, HIGH);
            Serial.println("Slave LED ON");
        } else {
            digitalWrite(LEDpin, LOW);
            Serial.println("Slave LED OFF");
        }
        
        // Read button state
        buttonvalue = digitalRead(buttonpin);
        
        // Logic to set the value of x to send to master
        if (buttonvalue == HIGH) {
            x = 1;
        } else {
            x = 0;
        }
        
        // Send data to master via SPDR
        Slavesend = x;
        SPDR = Slavesend;
        
        // Reset received flag
        received = false;
        
        _delay_ms(1000);
    }
}
