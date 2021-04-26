//PWM_servo.c

#include <avr\io.h>
#define F_CPU 16000000UL // 16 MHz
#include <util/delay.h>

void _delay_ms ( double __ms ); // Delay milliseconds

int main(void)
{
	int ii;
	DDRB=0xFF; //Set PORTB1 pin as output

	//TOP=ICR1;
	//Output compare OC1A 8 bit non inverted PWM
	//Clear OC1A on Compare Match, set OC1A at TOP
	//Fast PWM
	//ICR1=40000 defines 50Hz PWM (clock frequency/prescaler)/(target frequency)
	//for a 16 MHz clock frequency, a prescaler of 8 and a target frequency of 50 Hz
	//Start timer with prescaler 8

	ICR1=40000; // PWM Count = Clock Speed / (Target Frequency * prescaler) = 16,000,000/(50*8) = 400000
	// Target Frequency = 50 Hz

	TCCR1A|=(0<<COM1A0)|(1<<COM1A1)|(0<<COM1B0)|(0<<COM1B1)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10);
	TCCR1B|=(0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);

	OCR1A=1200; // Starting Pulse Duration = (Clock Speed  / prescaler) * 0.6 ms = (16,000,000 / 8) * .0006 = 1200

	while(1)
	{
		OCR1A+=10;
		_delay_ms(10);   // 10ms delay between changes
		if(OCR1A > 8000) // Ending Pulse Duration = (Clock Speed  / prescaler) * 2.4 ms = (16,000,000 / 8) * .0024 = 4800
		{
			OCR1A = 2000;
			for ( ii = 0 ; ii < 100 ; ii++){_delay_ms(10);} // Long delay for servo to return to zero
		}
	}
	return(0);
}