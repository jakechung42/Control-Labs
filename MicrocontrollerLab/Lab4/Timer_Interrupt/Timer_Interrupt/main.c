//Timer_Interrupt.c

#include <avr/io.h>
#include <avr/interrupt.h>

int main (void)
{
	DDRC = (1 << 5); // Set Pin 5 on Port C as output

	TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode

	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt

	sei(); // Enable global interrupts

	// OCR1A = Target_Timer_Count = (Clock_Frequency / (Prescale * Target_Frequency)) - 1

	OCR1A = 1599;   //Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64

	//Prescaler Fcpu/1 to get 1kHz and 5kHz counter
	TCCR1B = TCCR1B | (1 << CS10);

	// Timer at Fcpu/256 prescaler to get the 1Hz clock
	//TCCR1B = TCCR1B | (1 << CS12); 

	// TCCR1B |= ((1 << CS10) | (1 << CS11)); // Start timer at Fcpu/64 

	while(1){}

}

ISR(TIMER1_COMPA_vect)
{
	// Place code here that should be executed when the interrupt occurs

	PORTC ^= (1 << 5); // Toggle Pin 5 on Port C
}





