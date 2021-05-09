//  test interrupt with AD and DA
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
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

void timer_init(uint16_t topCount);
void clk_init(void);

// main set up
int main(void)
{
    unsigned int topCount;
    float Ts;
	clk_init();
	// dac_init();
	// adc_init();

    PORTD.DIRSET = (1<<7); //set PD7 as output for timing pin

    Ts = 0.001; //Start with 1k Hz for now seeing if the code works 
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

ISR(TCC1_OVF_vect)
{
    PORTD.OUTTGL = (1<<7);	//Toggle Pin D7 for timing
}

void clk_init(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm;					//enable 32Mhz RC Osc
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));		//wait for 32MHz Osc to be stable
	CCP = CCP_IOREG_gc;							//enable access to system clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;			//set 32Mhz RC Osc as system clock
}

void timer_init(uint16_t topCount)
{
	TCC1.CTRLB |= TC_WGMODE_NORMAL_gc;		//Normal mode, Output Compare pins disconnected
	TCC1.INTCTRLA |= TC_OVFINTLVL_LO_gc;	//Enable overflow interrrupt
	TCC1.PER = topCount;					//Set Period
	TCC1.CTRLA |= TC_CLKSEL_DIV4_gc;		//Start at Clk/4
}
