// Blink Version 2
#include <avr/io.h>
#define F_CPU 1000000UL  // 1 MHz  CPU Clock Frequency
#include <util/delay.h>  // Include the built in Delay Function
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))
int main (void)
{
	//1 = output, 0 = input
	DDRD = 0b00000000; // Set Pin 5 as Output
	while(1)
	{
		sbi(PORTC, 6); // Set pin 5 on PORTC
		_delay_ms(500); // Delay 500 ms
		cbi(PORTC, 6); // Clear pin 5 on PORTC
		_delay_ms(500); // Delay 500 ms
	}
	return(0);
}