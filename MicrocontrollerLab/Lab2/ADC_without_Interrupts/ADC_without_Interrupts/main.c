#include <avr/io.h>

#define LEDs	PORTD
#define red		0b11011111
#define green	0b01111111
#define yellow	0b10111111


int main (void)
{
	unsigned int adc_data;

	DDRD   = 0b11100000; //Set bits 6, 7 and 8 as outputs
	ADMUX  = 0b00000000; //Input on AD Channel 0
	ADCSRA = 0b10000111; // ADC on, /128 for a 16 MHz clock, interrupt off


	while(1)
	{
		ADCSRA = ADCSRA | 0b01000000;  // Start AD conversion, this or gate only flips ADSC

		while ((ADCSRA & 0b01000000) == 0b01000000); // Wait while AD conversion is executed

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
	}

}



