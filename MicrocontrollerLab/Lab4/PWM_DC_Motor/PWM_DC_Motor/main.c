//PWM_DC_Motor

#include <avr\io.h>
#define F_CPU 16000000UL // 16 MHz
#include <util/delay.h>
void _delay_ms ( double __ms ); // Delay milliseconds

int main(void)
{
	//int ii;
	DDRB=0xFF; //Set PORTB1 pin as output
	DDRC=0b00100011; //Set PORTC pin as output


	//TOP=ICR1;
	//Output compare OC1A 8 bit non inverted PWM
	//Clear OC1A on Compare Match, set OC1A at TOP
	//Fast PWM
	//ICR1=2000 defines 1000Hz PWM (clock frequency/prescaler)/(target frequency)
	//for a 16 MHz clock frequency, a prescaler of 8 and a target frequency of 1000 Hz
	//Start timer with prescaler 8

	ICR1=2000; // PWM Count = Clock Speed / (Target Frequency * prescaler) = 16,000,000/(1000*8) = 2000
	// Target Frequency = 1000 Hz

	TCCR1A|=(0<<COM1A0)|(1<<COM1A1)|(0<<COM1B0)|(0<<COM1B1)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10);
	TCCR1B|=(0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);

	OCR1A=100; // Starting Duty length

	PORTC = 0b00000010;  // Set initial value of the direction pins (P0 and P1)

	while(1)
	{
		OCR1A+=2;
		_delay_ms(10);          // 10ms delay between changes
		PORTC ^= 0b00100000;    //blink LED
		if(OCR1A > 1500)
		{
			OCR1A = 100;
			PORTC ^= 0b00000011; // Reverse Direction using XOR, this toggles bits P0 and P1
			_delay_ms(10);       // 10ms delay between changes
		}
	}
	return(0);
}