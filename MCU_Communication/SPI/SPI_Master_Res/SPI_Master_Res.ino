#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN 7
#define BUTTON_PIN 2

void SPI_MasterInit() {
    // Set MOSI, SCK, and SS as output pins
    DDRB |= (1 << DDB3) | (1 << DDB5) | (1 << DDB2);
    
    // Enable SPI, Master mode, set clock rate fck/16
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void SPI_MasterTransmit(uint8_t data) {
    // Start transmission
    SPDR = data;
    
    // Wait for transmission complete
    while (!(SPSR & (1 << SPIF)));
}

void setup() {
    Serial.begin(9600);
    
    // Set button pin as input
    DDRD &= ~(1 << DDD2);
    
    // Set LED pin as output
    DDRD |= (1 << DDD7);
    
    SPI_MasterInit();
}

void loop() {
    uint8_t buttonValue, x;
    
    // Read button state
    buttonValue = PIND & (1 << PIND2);
    
    if (buttonValue) {
        x = 1;
    } else {
        x = 0;
    }
    
    // Start SPI communication
    PORTB &= ~(1 << PB2); // Set SS pin low
    
    // Send data to slave and receive data from slave
    SPI_MasterTransmit(x);
    uint8_t receivedData = SPDR;
    
    // End SPI communication
    PORTB |= (1 << PB2); // Set SS pin high
    
    if (receivedData == 1) {
        // Turn on LED
        PORTD |= (1 << PORTD7);
        Serial.println("Master LED ON");
    } else {
        // Turn off LED
        PORTD &= ~(1 << PORTD7);
        Serial.println("Master LED OFF");
    }
    
    _delay_ms(1000);
}
