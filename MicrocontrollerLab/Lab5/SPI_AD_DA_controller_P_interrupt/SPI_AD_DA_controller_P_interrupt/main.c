// Velocity Control Example Proportional Control

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// Math functions
#include <math.h>
#include <stdlib.h>

#define F_CPU 16000000UL // 16 MHz CPU Clock Frequency

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

//Set up SPI and AD DA
unsigned char spi_data_0, spi_data_1, dummy_read;
unsigned int adc_output, adc_input;

//Set up Control variables
float	Vel_Set_v;
float	Control, Max_Voltage, Kp;
float	adc_input_v;
float	Error;

// SPI write read function
unsigned char spi_write_read(unsigned char spi_data)
{
	SPDR=spi_data;
	while ((SPSR & (1<<SPIF))==0); // Wait until the data transfer is complete
	return SPDR;
}

int main (void)
{
	float	Sawtooth, StepInput, Sawtooth_Amplitude, Step_Amplitude, Input_Increment;

	Vel_Set_v = -3.0;

	Max_Voltage = 6.0;
	Kp          = 1.0; 	// Proportional control constant
	// The motor velocity voltage is cycled from -3 volts to +3 volts
	// The incriment needs to be very small so the velocity change is obsevable
	
	// Note if you have print statments active this will slow the control loop dramatically

	//USART Setup
	UBRR0H = (MYUBRR >> 8);
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	stdout = &mystdout; //Required for printf init

	// AD initialization
	ADMUX  = 0b00000010; //Input on ADC2
	ADCSRA = 0b10000111; // ADC on, /128 for a 16 MHz clock, interrupt off

	DDRB=0b00101100; //Set Output Ports for the SPI Interface
	DDRD=0b10000010; //Set Output Ports for the Chip select and USART
	
	// SPI initialization
	SPCR=0b01010010;
	SPSR=0b00000000;

	//Interrupt counter set up
	// Configure timer 1 for CTC mode, clk/8
	TCCR1B |= (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10); 
	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
	
	sei(); // Enable global interrupt

	// OCR1A = Target_Timer_Count = (Clock_Frequency / (Prescale * Target_Frequency)) - 1

	OCR1A = 3999;   //Set CTC compare value to 200Hz at 16MHz AVR clock, with a prescaler of 8

	Sawtooth           = -1.0;			// Initial value
	Sawtooth_Amplitude = 5.0;			// 5 volts maximum
	Step_Amplitude     = 5.0;			// 5 volts maximum
	Input_Increment    = 0.02;		    // This variable is used to specify the desired frequency

	// Frequency = Input_Increment*SampleFrequency/2
	// SampleFrequency = 1/SampleTime
	// SampleTime is the physical time for one cycle

	while(1)
	{
		// Digitally generated Input wave form
		Sawtooth += Input_Increment;						// Input_Increment
		if(Sawtooth >= 1.0) Sawtooth = -1.0;                // Sawtooth Input Value (-1 to 1)
		if(Sawtooth <= 0.0) StepInput = 0.0;                  // Step Input Value     (0 to 1)
		if(Sawtooth > 0.0)  StepInput = 1.0;                  // Step Input Value		(0 to 1)
				
		//Vel_Set_v = Sawtooth*Sawtooth_Amplitude;            	// Set Velocity Set Point to either Sawtooth or Step Input Value
		Vel_Set_v = StepInput * Step_Amplitude;					// Set Velocity Set Point to either Sawtooth or Step Input Value
																	// Note the Velocity Set Point is in Control Voltage Units (+- 10 volts)
		// Vel_Set_v += 0.0005;
		// if(Vel_Set_v >= 3.0) Vel_Set_v = -3.0;
		//printf("Print value: %d\n", StepInput);
	}
}

ISR(TIMER1_COMPA_vect)
{
	// Begin sampling for control system
	ADCSRA = ADCSRA | 0b01000000;  					// Start AD conversion
	while ((ADCSRA & 0b01000000) == 0b01000000); 	// Wait while AD conversion is executed

	adc_input = ADCW; 									// Read AD value
	adc_input_v = (float) adc_input*(20./1024.)- 10.0;	// Convert the adc_input digital value (0 to 1024) to a voltage
	// Note the input is bipolar +- 10 volts
	// Note that the (10./1024.) term needs the decimal point
	// or else it is interrupted as an integer and the result is zero
	// Control Equation

	Error   = (Vel_Set_v - adc_input_v);			// Error (units are voltage +- 5 volts)
	Control = Kp * Error;  						    // Control (units are voltage  +- 5 volts)

	if(fabs(Control) >= Max_Voltage)				// Check Maximum voltage
	Control = copysign(Max_Voltage,Control);

	adc_output = floor((Control + 10.)*4096./20.);  			// Convert control voltage to a digital number for output
	// Note the output is +- 5 Volts  which corresponds to 0 to 4095
		
	// printf("Error, vel_Set_v, adc_input, adc_output %d    %d    %d    %d\n", (int) Error,(int) Vel_Set_v,adc_input,adc_output );
		
	// Output adc_output to DAC
	spi_data_0 = 0x00; 								// Zero spi_data_0
	spi_data_0 = (adc_output & 0x0F00) >> 8; 		// Set up the first byte to write by mapping bits 8-11
	// to the lower 4 bit positions and
	spi_data_0 = spi_data_0 + 0b00110000; 			// Adding the upper 4 DA control bits
	spi_data_1 = (adc_output & 0xFF); 				// Set up the second byte to write by mapping
	// bits 0-7 to the lower 8 bit positions
	cbi(PORTD,7); 									// Activate the chip - set chip select to zero
	dummy_read = spi_write_read(spi_data_0); 		// Write/Read first byte
	dummy_read = spi_write_read(spi_data_1); 		// Write/Read second byte
	sbi(PORTD,7); 	
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


