// Blink Version 2
#include <avr/io.h>
#define F_CPU 1000000UL  // 1 MHz  CPU Clock Frequency
#include <util/delay.h>  // Include the built in Delay Function
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask)) //set bit in I/O register
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask)) //clear bit in I/O register
int main (void)
{
	//1 = output, 0 = input
	DDRD = 0b01000000; // Set Pin 5 as Output
	while(1)
	{
		sbi(PORTD, 6); // Set pin 6 on PORTD
		_delay_ms(500); // Delay 500 ms
		cbi(PORTD, 6); // Clear pin 6 on PORTD
		_delay_ms(500); // Delay 500 ms
	}
	return(0);
}