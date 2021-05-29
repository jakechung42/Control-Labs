/************************************************************************/
/* Feedback Control for HDD                                             */
/************************************************************************/

#define F_CPU 32000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>

// macros for baudrate
#define BAUD 460800
#define f_PER 32000000
#define BSCALE 	-5
#define BSEL 	107

/************************************************************************/
/* Function Prototypes                                                  */
/************************************************************************/
void clk_init(void);
void usart_init(void);
static int put_char(char c, FILE *stream);
void encoder_init(void);
void timer_init(uint16_t topCount);
void dac_init(void);
void adc_init(void);
void ADC_CalibrationValues_Set(ADC_t * adc);
void DAC_CalibrationValues_Set(DAC_t * dac);
uint8_t SP_ReadCalibrationByte( uint8_t index );


// All variables that are declared outside the interupt function and are referenced or changed
// inside the interupt function must be declared as volatile.

/***** Parameters to Change ******/
volatile float Kp = 15.0;				//Proportional Gain
volatile float 	Ts = 0.00012500;		//Sample Time set by period of TCC1. Max of 0.00819 with Clk/4
/*********************************/

volatile uint16_t	topCount = 0;		//TOP value for interrupt timer
volatile int 	encCount = 0;			//Encoder count at beginning of control loop
volatile float 	position = 0;			//Position of arm
volatile float 	positionDAC = 0;		//Position converted to DAC value (0-4095)
volatile float 	error = 0;				//E(k)
volatile uint16_t	adcIn = 0;			//ADC value for setpoint
volatile float	setPoint = 0;			//Desired setpoint
volatile float 	ctrlOut = 0;			//Vm(k)
volatile float 	ctrlDAC = 0;			//Control signal converted to DAC value (0-4095)
volatile float 	maxVoltage = 12.0;		//Maximum Voltage available from power supply
volatile float 	ctrlCorrection = 0;		//Control signal Correction to adjust zero value of the Single ended to Bipolar circuit

/**********	Precompute some Coefficients ******************/
volatile float	dacConst = 0;			//H-Bridge constant for voltage to PWM conversion


int main(void)
{
	
	clk_init();
	encoder_init();
	usart_init();
	dac_init();
	adc_init();
	
	PORTD.DIRSET = (1<<7);	//set D7 as output for timing pin
	dacConst = 4095./(2.0*maxVoltage);		//Conversion factor for Control voltage to DAC value
	topCount = (uint16_t)(Ts*8000000.);		//Computed TOP value for TCC1
	
	timer_init(topCount);
	
	//enable intterupts all levels
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
	
	while(1)
	{
		//These are not the droids you are looking for...
	}
}


/************************************************************************/
/* Interrupt Service Routine for TCC1 overflow.  This is where the		*/
/* control loop runs.                                                   */
/************************************************************************/
ISR(TCC1_OVF_vect)
{
	
	adcIn = ADCA.CH0.RES;				//read setpoint from ADC
	// setPoint = (adcIn/4095.);		//convert to voltage.0.
	setPoint = (adcIn/4095.)-0.030;		//convert to voltage
	// setPoint = 2;                   //Set the setpoint to midrange for initial testing of the controller
	
	encCount = TCC0.CNT;			//read encoder
	if(encCount < 0) encCount = 0;	//out of bounds check  (encoder count should not be below zero)
	
	position = encCount/700.;		//encoder range of 0-700 mapped to 0-1V
	positionDAC = (position*4095.);	//convert arm postition to DAC output
	// printf("positionDAC = %d, position = %d, encCount = %d\n", (int)(positionDAC), (int)(position*100), (int)encCount);
	// printf("encCount = %d\n", (int)encCount);
	if(positionDAC < 0) positionDAC = 0;	//rollover check
	// positionDAC = 2048;
	// position = 0.0;

	error = setPoint - position;			//compute error
	
	/****** Control Equation ******/
	
	ctrlCorrection = 0.4;  // Initial value
	//ctrlCorrection = 0.495;  // Control Correction to Correct the Single ended to Bipolar circuit zero offset
	// For my circuit the output of the Single ended to Bipolar circuit equaled -0.495 volts
	// when the control input (ctrlOut = 0.0) equalled 0.
	
	
	ctrlOut = Kp * error;
	// ctrlOut = 0.0;        // Set ctrlOut=0 and read the value of the voltage offset at the output of
	// the Single ended to Bipolar circuit.  In a perfect world this value would be zero.
	// This value is used to calculate ctrlCorrection.  That is ctrlCorrection = -output value.
	
	/******************************/
	
	if (fabs(ctrlOut) >= maxVoltage)			//Saturation check
	ctrlOut = copysign(maxVoltage, ctrlOut);
	
	ctrlDAC = ((ctrlOut+ ctrlCorrection)*dacConst)+2048.;	// Convert ctrl output voltage (including ctrlCorrection) to DAC value
	if(ctrlDAC < 0) ctrlDAC=0;								// Check to make sure ctrlDAC is not less than zero
	if(ctrlDAC > 4095) ctrlDAC=4095;						// Check to make sure ctrlDAC is not greater than the maximum value
	
	//ctrlDAC = 0;											// Used to check the voltage of the DAC

	
	// while((DACB.STATUS & DAC_CH0DRE_bm)==0);
	// DACB.CH0DATA = (int)ctrlDAC;				//Write ctrl signal to DACB channel 0
	// while((DACB.STATUS & DAC_CH1DRE_bm)==0);
	// DACB.CH1DATA = (int)positionDAC;					//Write arm position to DACB channel 1

	// flip the order to check channel 0 and channel 1
	// printf("positionDAC = %d\n", (int)(positionDAC));
	while((DACB.STATUS & DAC_CH0DRE_bm)==0);
	DACB.CH0DATA = (int)positionDAC;				//Write arm position to DACB channel 0
	while((DACB.STATUS & DAC_CH1DRE_bm)==0);
	DACB.CH1DATA = (int)ctrlDAC;					//Write ctrl signal to DACB channel 1

	
	// printf("Error = %d, ctrlOut = %d, Position = %d, ctrlDac = %d, PositionDac = %d\n", (int)(error*100), (int)(ctrlOut*100), (int)(position*100), (int)ctrlDAC, (int)positionDAC);
	
	PORTD.OUTTGL = (1<<7);	//Toggle Pin D7 for timing
}


/************************************************************************/
/*Initialization Functions for Hardware                                 */
/************************************************************************/

void clk_init(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm;					//enable 32Mhz RC Osc
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));		//wait for 32MHz Osc to be stable
	CCP = CCP_IOREG_gc;							//enable access to system clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;			//set 32Mhz RC Osc as system clock
}

void usart_init(void)
{

	//Set TxD as output RxD as input
	PORTC.DIRSET = (1<<3);
	PORTC.DIRCLR = (1<<2);

	//Set mode, baud rate and frame format
	USARTC0.CTRLC |= USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc;
	USARTC0.BAUDCTRLA = (uint8_t)BSEL;
	USARTC0.BAUDCTRLB = (BSCALE<<USART_BSCALE0_bp) | (BSEL>>8);

	//enable Tx and Rx
	USARTC0.CTRLB |= USART_TXEN_bm;

	// setup printf to use serial port
	fdevopen(&put_char,NULL);

}

static int put_char(char c, FILE *stream)
{
	if (c == '\n') put_char('\r',stream);		//add return to newline character for term

	while(!(USARTC0.STATUS & USART_DREIF_bm)); //loop until Tx is ready
	USARTC0.DATA = c;
	return 0;
}


void encoder_init(void)
{
	// set encoder pins as input
	PORTD.DIRCLR = (1<<0) | (1<<1);
	PORTD.PIN0CTRL |= PORT_ISC_LEVEL_gc;		//set Pin 0 to level sensing

	//setup event system
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN0_gc;	//route PIND0 to Event channel 0
	EVSYS.CH0CTRL |= EVSYS_QDEN_bm 				//enable quadrature decode
	| EVSYS_DIGFILT_2SAMPLES_gc;					//set digital filter to 1 sample

	//setup timer
	TCC0.CTRLD |= TC_EVACT_QDEC_gc				//set timer to quadrature decode
	| TC_EVSEL_CH0_gc;							//set EVCH0 as source
	TCC0.CTRLA |= TC_CLKSEL_DIV1_gc;			//enables timer
}


void dac_init(void)
{
	DAC_CalibrationValues_Set(&DACB);
	DACB.CTRLB |= DAC_CHSEL_DUAL_gc;
	DACB.CTRLC |= DAC_REFSEL_INT1V_gc;	// 1 volt internal reference. bug in xmega makes 0-.75v output noisy when using reference above 2v
	// DACB.TIMCTRL |= DAC_CONINTVAL_32CLK_gc;
	DACB.CH0DATAH = 0x00;
	DACB.CH1DATAH = 0x00;
	DACB.CTRLA |= DAC_ENABLE_bm | DAC_CH0EN_bm | DAC_CH1EN_bm;
	
	DAC_CalibrationValues_Set(&DACA);
	DACA.CTRLB |= DAC_CHSEL_DUAL_gc;
	DACA.CTRLC |= DAC_REFSEL_INT1V_gc;	// 1 volt internal reference. bug in xmega makes 0-.75v output noisy when using reference above 2v
	//DACA.TIMCTRL |= DAC_CONINTVAL_32CLK_gc;
	DACA.CH0DATAH = 0x00;
	DACA.CH1DATAH = 0x00;
	DACA.CTRLA |= DAC_ENABLE_bm | DAC_CH0EN_bm | DAC_CH1EN_bm;
}

void adc_init(void)
{
	ADC_CalibrationValues_Set(&ADCA);
	ADCA.CTRLB |= ADC_FREERUN_bm;
	ADCA.REFCTRL |= ADC_BANDGAP_bm;				//enable bandgap for internal vref
	ADCA.CH0.CTRL |= ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.PRESCALER |= ADC_PRESCALER_DIV16_gc;	//maximum ADC clock of 2 MHz
	ADCA.CTRLA |= ADC_ENABLE_bm;				//enable ADCA
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

void DAC_CalibrationValues_Set(DAC_t * dac)
{
	if(&DACA == dac){
		/* Get DACA0OFFCAL from byte address 0x30 */
		dac->CH0OFFSETCAL = SP_ReadCalibrationByte(0x30);
		/* Get DACA0GAINCAL from byte address 0x31 */
		dac->CH0GAINCAL = SP_ReadCalibrationByte(0x31);
		/* Get DACA1OFFCAL from byte address 0x34 */
		dac->CH1OFFSETCAL = SP_ReadCalibrationByte(0x34);
		/* Get DACA1GAINCAL from byte address 0x35 */
		dac->CH1GAINCAL = SP_ReadCalibrationByte(0x35);
		}else {
		/* Get DACB0OFFCAL from byte address 0x32 */
		dac->CH0OFFSETCAL = SP_ReadCalibrationByte(0x32);
		/* Get DACB0GAINCAL from byte address 0x33 */
		dac->CH0GAINCAL = SP_ReadCalibrationByte(0x33);
		/* Get DACB1OFFCAL from byte address 0x36 */
		dac->CH1OFFSETCAL = SP_ReadCalibrationByte(0x36);
		/* Get DACB1GAINCAL from byte address 0x37 */
		dac->CH1GAINCAL = SP_ReadCalibrationByte(0x37);
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

void timer_init(uint16_t topCount)
{
	TCC1.CTRLB |= TC_WGMODE_NORMAL_gc;		//Normal mode, Output Compare pins disconnected
	TCC1.INTCTRLA |= TC_OVFINTLVL_LO_gc;	//Enable overflow interrrupt
	TCC1.PER = topCount;					//Set Period
	TCC1.CTRLA |= TC_CLKSEL_DIV4_gc;		//Start at Clk/4
}