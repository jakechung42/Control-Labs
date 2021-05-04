// Velocity Control Example Proportional Control

#include <stdio.h>
#include <avr/io.h>
// Math functions
#include <math.h>
#include <stdlib.h>

#define F_CPU 16000000UL // 16 MHz CPU Clock Frequency

#include <util/delay.h>
void _delay_ms ( double __ms ); // Delay millaseconds

#define sbi(var, mask) ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask) ((var) &= (uint8_t)~(1 << mask))
#define SPIF 7

// USART definitions
#define FOSC F_CPU  // CPU Clock Frequency must be set correctly for the USART to work
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

// USART declarations
static int    uart_putchar(char c, FILE *stream);
unsigned char uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// SPI write read function
unsigned char spi_write_read(unsigned char spi_data)
{
	SPDR=spi_data;
	while ((SPSR & (1<<SPIF))==0); // Wait until the data transfer is complete
	return SPDR;
}
int main (void)
{
	unsigned int adc_input;
	float	Vel_Set_v;
	float	Control, Control_HB, Max_Voltage, Kp;
	float	adc_input_v;
	float	Error;

	//USART Setup
	UBRR0H = MYUBRR >> 8;
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	stdout = &mystdout; //Required for printf init


	// AD initialization
	ADMUX  = 0b00000100; //Input on AD Channel 4
	ADCSRA = 0b10000111; // ADC on, /128 for a 16 MHz clock, interrupt off

	DDRB=0b00101110; //Set Output Ports for the SPI Interface and PWM
	DDRD=0b10000010; //Set Output Ports for the Chip select and USART
	DDRC=0b00100011; //Set Output Ports for the LED and H-Bridge direction

	// PWM Initialization
	//Fast PWM
	//ICR1=1600 defines 10,000Hz PWM (clock frequency/prescaler)/(target frequency)
	//for a 16 MHz clock frequency, a prescaler of 1 and a target frequency of 10,000 Hz
	//Start timer with prescaler 1

	ICR1=1600; // PWM Count = Clock Speed / (Target Frequency * prescaler) = 16,000,000/(10000*1) = 1600

	TCCR1A|=(0<<COM1A0)|(1<<COM1A1)|(0<<COM1B0)|(0<<COM1B1)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10);
	TCCR1B|=(0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10);

	// SPI initialization
	SPCR=0b01010010;
	SPSR=0b00000000;

	Vel_Set_v = -3.0;

	Max_Voltage = 8.0;
	Kp          = 10.0; 	// Proportional control constant

	OCR1A=200; // Starting Duty length
	PORTC = 0b00000010;  // Set initial value of the direction pins (P0 and P1)

	while(1)
	{
		// The motor velocity voltage is cycled from -3 volts to +3 volts
		// The incriment needs to be very small so the velocity change is obsevable
		
		// Note if you have print statments active this will slow the control loop dramatically
		
		Vel_Set_v += .0001;
		if(Vel_Set_v >= 3.0) Vel_Set_v = -3.0;


		ADCSRA = ADCSRA | 0b01000000;  					// Start AD conversion
		while ((ADCSRA & 0b01000000) == 0b01000000); 	// Wait while AD conversion is executed

		adc_input = ADCW; 									// Read AD value
		adc_input_v = (float) adc_input*(20./1024.)- 10.0;	// Convert the adc_input digital value (0 to 1024) to a voltage
		// Note the input is bipolar +- 10.0 volts
		// The terms in the equation above are for a  10 volt bipolar range.
		// The (20.) term represents the bipolar range (+10 �(-10) = 20.) and
		// the (-10.) term is the bipolar range divided by 2.
		// Note that the (20./1024.) term needs the decimal point
		// or else it is interrupted as an integer and the result is zero
		
		//adc_input_v = (float) adc_input*(19.3/1024.)- 9.65; // These are the measured values for my circuit,
		// yours will be different.
		
		//printf("adc_input = %d   adc_input_v = %d\n",  // For Debugging only (slows the control loop down by a lot!)
		//        adc_input, (int) (adc_input_v*1000));  // Since adc_input_v is a float number it must be converted to an integer
		// and scaled by 1000 to get a meaningful output

		// Control Equation
		Error   = (Vel_Set_v - adc_input_v);			// Error (units are voltage +- 10 volts)
		Control = Kp * Error;  						    // Control (units are voltage  +- 10 volts)
		//Control = 5.0;

		if(fabs(Control) >= Max_Voltage)				// Check Maximum voltage
		Control = copysign(Max_Voltage,Control);

		Control_HB = (float) (Control/12.0*1600.0);   // Convert the Control voltage to a H-Bridge output value between o and 1600

		// Set the direction and value of the H-Bridge output

		if(Control_HB < 0)
		{
			cbi(PORTC, 0);  //Set Direction pins  (01)
			sbi(PORTC, 1);
			Control_HB = -Control_HB;

			if(Control_HB >= 1600.)
			{
				Control_HB   = 1600.;
			}
			OCR1A = (int)Control_HB;
		}
		else
		{
			sbi(PORTC, 0);  //Set Direction pins  (10)
			cbi(PORTC, 1);
			if(Control_HB >= 1600.)
			{
				Control_HB   = 1600.;
			}
			OCR1A = (int)Control_HB;
		}


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








