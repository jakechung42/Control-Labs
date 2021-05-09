// Blink Version 2

#include <avr/io.h>
#include <stdio.h>

#define F_CPU 32000000UL  // 32 MHz  CPU Clock Frequency
#include <util/delay.h>  // Include the built in Delay Function

#define sbi(var, bit)   ((var) |= _BV(bit))
#define cbi(var, bit)   ((var) &= ~_BV(bit))

// macros for baudrate
#define BAUD 921600
#define f_PER 32000000
#define BSCALE 	-6
#define BSEL 	75

//Define functions
//======================
void delay_ms(uint16_t x); 	//General purpose delay
void clk_init(void);		// Initialize the system clock to 32 MHz
void usart_init(void);
static int put_char(char c, FILE *stream);
//======================

int main (void)
{

	unsigned int x = 0;

	clk_init();    // Initialize the system clock to 32 MHz
	usart_init();  // Initialize the serial port
	
	//1 = output, 0 = input
	PORTC.DIR = 0b00100000;  // Set Pin 5 as Output

	while(1)
	{
		x++;
		
		printf("Test it! x = %d\n", x);

		sbi(PORTC_OUT, 5);  // Set pin 5 on PORTC
		_delay_ms(50); // Delay 500 ms

		cbi(PORTC_OUT, 5);  // Clear pin 5 on PORTC
		_delay_ms(50); // Delay 500 ms
	}
	return(0);
}


void clk_init(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm;					//enable 32Mhz RC Osc
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));		//wait for Osc to be stable
	CCP = CCP_IOREG_gc;							//enable access to system clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;			//set 32Mhz RC Osc as system clock
}


void usart_init(void)
{

	//Set TxD as output RxD as input
	PORTD.DIRSET = (1<<7);
	PORTD.DIRCLR = (1<<6);

	//Set mode, baud rate and frame format
	USARTD1.CTRLC |= USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc;
	USARTD1.BAUDCTRLA = (uint8_t)BSEL;
	USARTD1.BAUDCTRLB = (BSCALE<<USART_BSCALE0_bp) | (BSEL>>8);

	//enable Tx and Rx
	USARTD1.CTRLB |= USART_TXEN_bm;

	// setup printf to use serial port
	fdevopen(&put_char,NULL);

}


static int put_char(char c, FILE *stream)
{
	if (c == '\n') put_char('\r',stream);	//add return to newline character for term

	while(!(USARTD1.STATUS & USART_DREIF_bm)); //loop until Tx is ready
	USARTD1.DATA = c;
	return 0;
}