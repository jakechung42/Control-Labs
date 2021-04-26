//AD_DA with interrupt at 1kHz

#include <avr/io.h>
#include <avr/interrupt.h>

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))
#define SPIF 7

//Set up SPI and AD DA
unsigned char spi_data_0, spi_data_1, dummy_read;
unsigned int adc_output;

// SPI write read function
unsigned char spi_write_read(unsigned char spi_data)
{
	SPDR=spi_data;
	while ((SPSR & (1<<SPIF))==0);	// Wait until the data transfer is complete
	return SPDR;
}

int main (void)
{
	DDRB=0b00101100;	//Set Output Ports for the SPI Interface
	DDRD=0b10000000;	//Set Output Ports for the Chip select
	// SPI initialization
	SPCR=0b01010010;
	SPSR=0b00000001;
	//ADC set up
	ADMUX  = 0b00000000; //Input on AD Channel 0
	ADCSRA = 0b10000011; // ADC on, /8 for a 16 MHz clock, interrupt off

	//Interrupt counter set up
	TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt

	sei(); // Enable global interrupts

	// OCR1A = Target_Timer_Count = (Clock_Frequency / (Prescale * Target_Frequency)) - 1

	OCR1A = 7999;   //Set CTC compare value to 1kHz at 8MHz AVR clock, with a prescaler of 1

	//Prescaler Fcpu/1 to get 1kHz 
	TCCR1B = TCCR1B | (1 << CS10);

	while(1)
	{
		// Does nothing and wait for counter	
	}
}

ISR(TIMER1_COMPA_vect)
{
	//Start the AD DA process on interrupt counter

	//ADC process
	ADCSRA = ADCSRA | 0b01000000;  // Start AD conversion, this or gate only flips ADSC
	while ((ADCSRA & 0b01000000) == 0b01000000); // Wait while AD conversion is executed
	adc_output = ADCW;
	adc_output = adc_output << 2;

	//SPI process
	spi_data_0 = 0x00;
	spi_data_0 = (adc_output & 0x0F00) >> 8;  //Set up first byte to write
	spi_data_0 = spi_data_0 + 0b00110000;
	spi_data_1 = (adc_output & 0xFF);  //Set up second byte to write

	cbi(PORTD,7);								// Activate the chip - set chip select to zero
	dummy_read = spi_write_read(spi_data_0);	// Write/Read first byte
	dummy_read = spi_write_read(spi_data_1);  	// Write/Read second byte
	sbi(PORTD,7);	
}





