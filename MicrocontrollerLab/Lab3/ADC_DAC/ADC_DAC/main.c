
// SPI_test_WIN_Avr_ATmega8.c

#include <avr/io.h>
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))
#define SPIF 7

// SPI write read function
unsigned char spi_write_read(unsigned char spi_data)
{
	SPDR=spi_data;
	while ((SPSR & (1<<SPIF))==0);	// Wait until the data transfer is complete
	return SPDR;
}

int main (void)
{
	unsigned char   spi_data_0;
	unsigned char   spi_data_1;
	unsigned char   dummy_read;
	unsigned int	adc_output;

	DDRB=0b00101100;	//Set Output Ports for the SPI Interface
	DDRD=0b10000000;	//Set Output Ports for the Chip select

	// SPI initialization
	SPCR=0b01010010;
	SPSR=0b00000001;

	//ADC set up
	ADMUX  = 0b00000000; //Input on AD Channel 0
	ADCSRA = 0b10000111; // ADC on, /128 for a 16 MHz clock, interrupt off

	while(1)
	{
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
		sbi(PORTD,7);								// Release the chip  - set chip select to one
	}
}
