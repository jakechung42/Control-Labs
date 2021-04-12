#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDs	PORTD
#define red		0b11011111
#define green	0b01111111
#define yellow	0b10111111

ISR(ADC_vect)
{
	unsigned int adc_data;

	adc_data = ADCW;

	if(adc_data > (3*1023)/5)
	{
		LEDs = red;
	}
	else if (adc_data < (2*1023)/5)
	{
		LEDs = yellow;
	}
	else
	{
		LEDs = green;
	}
	ADCSRA = ADCSRA | 0b01000000;
}

int main (void)
{
	DDRD   = 0b11100000; //Set bits 6, 7 and 8 as outputs
	ADMUX  = 0b00000000; //Input on AD Channel 0
	ADCSRA = 0b11001111; // ADC on, /128 for a 16 MHz clock, interrupt on

	asm("sei");

	while(1)
	{
		;
	}
}



