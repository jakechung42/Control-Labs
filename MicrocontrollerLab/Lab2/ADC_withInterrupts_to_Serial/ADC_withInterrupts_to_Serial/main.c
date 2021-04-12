#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL // 8 MHz CPU Clock Frequency
#include <util/delay.h> // Include the built in Delay Function

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

// USART definitions
#define FOSC F_CPU  // CPU Clock Frequency must be set correctly for the USART to work
#define BAUD 14400
#define MYUBRR FOSC/16/BAUD-1

// USART declarations
static int    uart_putchar(char c, FILE *stream);
unsigned char uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//======================

ISR(ADC_vect)
{
	unsigned int adc_data;
	adc_data = ADCW;
    printf("%d\n", adc_data); //print to serial
	ADCSRA = ADCSRA | 0b01000000;
}

int main (void)
{
	DDRD = 0b00000010; //PORTD (RX on PD0 and TX on PD1)

	//USART Setup
	UBRR0H = MYUBRR >> 8;
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	stdout = &mystdout; //Required for printf init

    //ADC set up
    ADMUX  = 0b00000000; //Input on AD Channel 0
	ADCSRA = 0b11001111; // ADC on, /128 for a 16 MHz clock, interrupt on

    //Calling interrupts
	asm("sei");
	while(1)
	{
        ;
	}
}

// USART Functions

static int uart_putchar(char c, FILE *stream)
{
	if (c == '\n') uart_putchar('\r', stream);
	
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	
	return 0;
}

unsigned char uart_getchar(void)
{
	while( !(UCSR0A & (1<<RXC0)) );
	return(UDR0);
}

