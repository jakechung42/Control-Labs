//  Xmega_Test_AD_DA_64pin_chip_4

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#define F_CPU 32000000UL  // 32 MHz
#include <util/delay.h>
#define sbi(var, bit)   ((var) |= _BV(bit))   
#define cbi(var, bit)   ((var) &= ~_BV(bit))


//AD Macros
#define ADC_ConvMode_and_Resolution_Config(_adc, _signedMode, _resolution)     \
	((_adc)->CTRLB = ((_adc)->CTRLB & (~(ADC_RESOLUTION_gm|ADC_CONMODE_bm)))|  \
		(_resolution| ( _signedMode? ADC_CONMODE_bm : 0)))
#define ADC_Ch_InputMux_Config(_adc_ch, _posInput, _negInput)                  \
	((_adc_ch)->MUXCTRL = (uint8_t) _posInput | _negInput)
#define ADC_Referance_Config(_adc, _convRef)                                   \
	((_adc)->REFCTRL = ((_adc)->REFCTRL & ~(ADC_REFSEL_gm)) | _convRef)
#define ADC_Ch_InputMode_and_Gain_Config(_adc_ch, _inputMode, _gain)       \
	(_adc_ch)->CTRL = ((_adc_ch)->CTRL &                                   \
	                  (~(ADC_CH_INPUTMODE_gm|ADC_CH_GAIN_gm))) |        \
	                  ((uint8_t) _inputMode|_gain)
#define ADC_Prescaler_Config(_adc, _div)                                       \
	((_adc)->PRESCALER = ((_adc)->PRESCALER & (~ADC_PRESCALER_gm)) | _div)
#define ADC_Enable(_adc) ((_adc)->CTRLA |= ADC_ENABLE_bm)


// macros for serial baudrate
#define BAUD 921600
#define f_PER 32000000
#define BSCALE 	-6
#define BSEL 	75


//Define functions
//======================
void ioinit(void);      //Initializes IO
//AD
void ADC_CalibrationValues_Set(ADC_t * adc);
uint8_t SP_ReadCalibrationByte( uint8_t index );
//DA
void DAC_SingleChannel_Enable( volatile DAC_t * dac, DAC_REFSEL_t convRef, bool leftAdjust );
// Usart
void usart_init(void);
static int put_char(char c, FILE *stream);


int main (void)
{
    uint16_t AD_value;


	ioinit();       //Setup IO pins and defaults
	usart_init();  // Initialize the serial port


    while(1)
    {

	//	AD
		ADCA.CTRLA        = ADCA.CTRLA | ADC_CH0START_bm;       // Start Conversion
		while(((ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm) == 0x00));   // Is the conversion is complete ?
		
	  	ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;					   	// Clear interup flag by writing a one
		AD_value = ADCA.CH0.RES;								// Read AD Value
	
	//	DA
		while ( (DACB.STATUS & DAC_CH0DRE_bm) == false );  // Wait for the DA regester to be empty
		DACB.CH0DATA = AD_value;                           // write the DAC Value
 
		//printf("AD_value = %d\n",AD_value);
		
    }
   
    return(0);
}

void ioinit (void)
{

	PORTB_DIR = 0b00000100;  // DACB DAC0 Set as Output
		
	// Set 32MHz clock
	OSC.CTRL = OSC_RC32MEN_bm; 				//enable 32MHz oscillator 
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));	//wait for stability 
    CCP = CCP_IOREG_gc; 					//secured access 
    CLK.CTRL = 0x01; 						//choose this osc source as clk 

	// AD

	// Move stored calibration values to ADC A.
	ADC_CalibrationValues_Set(&ADCA);

	// Set up ADC A to have signed (true) or Unsigned (false) conversion mode and 12 bit resolution. 
	ADC_ConvMode_and_Resolution_Config(&ADCA, false, ADC_RESOLUTION_12BIT_gc);

	// Set referance voltage on ADC A to be Internal 1 volt
	ADC_Referance_Config(&ADCA, ADC_REFSEL_INT1V_gc);

	// Sample rate is CPUFREQ/256. Allow time for storing data.
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV16_gc);

	// Setup channel 2  to have singleended input. 
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
									 ADC_CH_INPUTMODE_SINGLEENDED_gc,
	                                 ADC_CH_GAIN_1X_gc);

	// Set input to the channels in ADC A to be PIN 3
	ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN3_gc, ADC_CH_MUXNEG_PIN2_gc);

	// Enable Enable AD Conversion in ADC A
	ADC_Enable(&ADCA);


	// DA

	// Setup DAC channel B with the DA reference set to the Analog supply voltage and DA data left adjust false 
	DAC_SingleChannel_Enable( &DACB, DAC_REFSEL_AVCC_gc, false);

}



void ADC_CalibrationValues_Set(ADC_t * adc)
{
	if(&ADCA == adc){
		 /* Get ADCCAL0 from byte address 0x20 (Word address 0x10. */
		adc->CAL = SP_ReadCalibrationByte(0x20);
	}else {
		/* Get ADCCAL0 from byte address 0x24 (Word address 0x12. */
		adc->CAL = SP_ReadCalibrationByte(0x24);
	}
}

uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
 	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
 	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
}


void DAC_SingleChannel_Enable( volatile DAC_t * dac, DAC_REFSEL_t convRef, bool leftAdjust )
{
	dac->CTRLB = ( dac->CTRLB & ~DAC_CHSEL_gm ) | DAC_CHSEL_SINGLE_gc;
	dac->CTRLC = ( dac->CTRLC & ~(DAC_REFSEL_gm | DAC_LEFTADJ_bm) ) |
	             convRef | ( leftAdjust ? DAC_LEFTADJ_bm : 0x00 );
	dac->CTRLA = ( dac->CTRLA & ~DAC_CH1EN_bm ) |
	             DAC_CH0EN_bm | DAC_ENABLE_bm;
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
