/*
 *
 * Xmega code to run Lag Controller AD DA 
 *
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#define F_CPU 32000000UL  // 32 MHz
#include <util/delay.h>
#define sbi(var, bit)   ((var) |= _BV(bit))   
#define cbi(var, bit)   ((var) &= ~_BV(bit))

// macros for serial baudrate
#define BAUD 9600
#define f_PER 32000000
#define BSCALE 	-6
#define BSEL 	75

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


//Define functions
//======================
void timer_init(uint16_t topCount);
void ioinit(void);      //Initializes IO
//AD
void ADC_CalibrationValues_Set(ADC_t * adc);
uint8_t SP_ReadCalibrationByte( uint8_t index );
//DA
void DAC_CalibrationValues_Set(DAC_t * dac);

// Usart
void usart_init(void);
static int put_char(char c, FILE *stream);

// Define variables
volatile uint16_t AD_value, DAC_output;
float Sawtooth, StepInput, Tach_val_V;
float Sawtooth_Amplitude, Step_Amplitude, Input_Increment;

volatile float Error, Error_m1, Error_m2;
volatile float Control, Control_m1, Control_m2;
volatile float Vel_Set_v;

float Max_Voltage, Kp;


// main set up
int main (void)
{
	unsigned int topCount;
	float Ts;
	Max_Voltage = 6.0;
	Vel_Set_v = -3.0;
	Kp = 1.0;
	//float InputTime, DeltaInputTime, SinInput, del_theta, theta, pi, DAC_output, SinAmplitude, StepAmplitude;
	//float InputData[100];
	//int ii, NumInputPoints;
	
	ioinit();       //Setup IO pins and defaults
	usart_init();  // Initialize the serial port

    Ts = 0.001; //Start with 1k Hz for now seeing if the code works 
	topCount = (uint16_t)(Ts*8000000.);		//Computed TOP value for TCC1

	timer_init(topCount);
	
	//enable intterupts all levels
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	// Digitally generated Input wave form
	
	// The sawtooth increment needs to be very small so the velocity change is observable
	// The sawtooth increment value is increased to decrease the period of the resulting sawtooth or step input value
		
	// Note if you have print statements active this will slow the control loop dramatically
		
	// Sawtooth and Step Input are in Control Voltage Units (+- 10 volts)
		
    Sawtooth           = -1.0;			// Initial value
	Sawtooth_Amplitude = 5.0;
	Step_Amplitude     = 5.0;
	Input_Increment = .001;		    // This variable is used to specify the desired frequency
										// Frequency = Input_Increment*SampleFrequency/2
										// Approximately 1HZ output frequency for Input_Increment    = 0.0001
										// Approximately 10HZ output frequency for Input_Increment   = 0.001
										// Approximately 100HZ output frequency for Input_Increment  = 0.01
										// Approximately 1000HZ output frequency for Input_Increment = 0.1
										// SampleFrequency = 1/SampleTime
										// SampleTime is the physical time for one cycle measured using the toggle pin P0
										// Note when you add additional code to the control loop the SampleTime will change and the values above will change
										// The SampleFrequency for this code and this chip is approximately 21.2 kHz
										// The digitally generated input is output to DACB.CH1 and can be viewed on the oscilloscope

	
	while(1)
	{
		// Wait for interrupt to run
	}
	
	return(0);
}


// Control code to be run during interrupt subroutine
ISR(TCC1_OVF_vect)
{
	// Digitally generated Input wave form
	Sawtooth += Input_Increment;						// Input_Increment
	if(Sawtooth >= 1.0) Sawtooth = -1.0;                // Sawtooth Input Value (-1 to 1)
	if(Sawtooth <= 0.0) StepInput = 0;                  // Step Input Value     (0 to 1)             
	if(Sawtooth > 0.0)  StepInput = 1;                  // Step Input Value		(0 to 1)
		
	// Vel_Set_v = Sawtooth*Sawtooth_Amplitude;             // Set Velocity Set Point to either Sawtooth or Step Input Value
	Vel_Set_v = StepInput*Step_Amplitude;                // Set Velocity Set Point to either Sawtooth or Step Input Value
		                                                     // Note the Velocity Set Point is in Control Voltage Units (+- 10 volts)

	//	Read tachometer sensor value
	ADCA.CTRLA = ADCA.CTRLA | ADC_CH0START_bm;       			// Start Conversion
	while(((ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm) == 0x00));   	// Is the conversion is complete ?
	
	ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;					   		// Clear interrupt flag by writing a one
	AD_value = ADCA.CH0.RES;									// Read AD Value
	
	Tach_val_V = (float) AD_value*(16.0/4095.0)-8.0;		// Convert the AD bit to Tach voltage
															// Tach voltage is between +/- 8

	//Calculate the control parameters
	Error = (Vel_Set_v - Tach_val_V);

	// Get ready! Here comes the Lag Control Equation!!!!!!
    Control = (425.6*Error + 2.0*Error_m1 - 2.0*Control_m1 - 423.6*Error_m2 + 750.6*Control_m2)/752.6;
	// Control = Kp*Error;

    Error_m1 = Error;
    Error_m2 = Error_m1;

    Control_m1 = Control;
    Control_m2 = Control_m1;

	if(fabs(Control) >= Max_Voltage)				// Check Maximum voltage
	Control = copysign(Max_Voltage, Control);

	DAC_output = floor((Control + 11.)*4095./20.); 
	//	DA
	while ( (DACB.STATUS & DAC_CH0DRE_bm) == false );  // Wait for the DA register to be empty
	DACB.CH0DATA = DAC_output;                           // write the DAC Value

	// DAC_output = floor(Vel_Set_v/5.0*4095.0);  	// Convert control voltage to a digital number for output
													// Note the output is +- 10 Volts  which corresponds to 0 to 4095
	// DACB.CH1DATA = DAC_output;                     // Write the DAC Value
		
	PORTC_OUT ^= (1 << 0);								// Toggle P0 on port C to check timing
														// Note the frequency of the displayed square wave 
														// is one half of the actual cycle frequency
														// because the cycle frequency is the time the signal
														// is on or off not the entire cycle
	
	// printf("Vel set v = %d\n", Vel_Set_v*1000);
}

void ioinit (void)
{

	PORTB_DIRSET = 0b00001100;  // DACB DAC0 and DACB DAC1 Set as Output
	PORTC_DIRSET = 0b00000001;  // PORT C P0 Set as Output for timing pin toggle
	
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

	// Set reference voltage on ADC A to be Internal 1 volt
	ADC_Referance_Config(&ADCA, ADC_REFSEL_INT1V_gc);

	// Sample rate is CPUFREQ/256. Allow time for storing data.
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV16_gc);

	// Setup channel 2  to have single ended input.
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	ADC_CH_INPUTMODE_SINGLEENDED_gc,
	ADC_CH_GAIN_1X_gc);

	// Set input to the channels in ADC A to be PIN 3
	ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN3_gc, ADC_CH_MUXNEG_PIN2_gc);

	// Enable Enable AD Conversion in ADC A
	ADC_Enable(&ADCA);


	// DA

	// Setup DAC channel B with the DA reference set to the Internal 1 volt supply voltage and DA data left adjust false
	
		DAC_CalibrationValues_Set(&DACB);
		DACB.CTRLB |= DAC_CHSEL_DUAL_gc;
//		DACB.CTRLC |= DAC_REFSEL_INT1V_gc;	// 1 volt internal reference. 
		DACB.CTRLC |= DAC_REFSEL_AVCC_gc;	// 3.3 volt internal reference.		DACB.CH0DATAH = 0x00;
		DACB.CH1DATAH = 0x00;
		DACB.CTRLA |= DAC_ENABLE_bm | DAC_CH0EN_bm | DAC_CH1EN_bm;
	
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


void timer_init(uint16_t topCount)
{
	TCC1.CTRLB |= TC_WGMODE_NORMAL_gc;		//Normal mode, Output Compare pins disconnected
	TCC1.INTCTRLA |= TC_OVFINTLVL_LO_gc;	//Enable overflow interrrupt
	TCC1.PER = topCount;					//Set Period
	TCC1.CTRLA |= TC_CLKSEL_DIV4_gc;		//Start at Clk/4
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