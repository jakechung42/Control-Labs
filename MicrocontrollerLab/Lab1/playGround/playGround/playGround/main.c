// playground code, copied from blink 2 program
// this code take in signal from pin PD2 and broadcast the signal in pin 6 
// the signal will be analog
#include <avr/io.h>
#define F_CPU 1000000UL  // 1 MHz  CPU Clock Frequency
#include <util/delay.h>  // Include the built in Delay Function
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask)) //set bit in I/O register
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask)) //clear bit in I/O register
int main (void)
{
	//1 = output, 0 = input
	DDRD = 0b01000000; // Set Pin 6 as Output and pin 7 as input
	PORTD = 0;
	int x; //define x and y
	while(1)
	{
		x = PINB;
		//y = x>>2;
		PORTD = x;
	}
	return(0);
}