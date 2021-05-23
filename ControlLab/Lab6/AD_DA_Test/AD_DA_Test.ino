//  AD_DA_Test.ino

// ADC 16 bits 0-65535 and 0-3.3 volts
// DAC 12 bits 0-4095  and 0-3.3 volts (conversion factor between DAC and ADC = 16)

#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC(); // adc object

void setup()
{
  Serial.begin(38400);
  pinMode(6, OUTPUT); // Set Pin 6 as an Output Pin
    
   ///// ADC0 ////
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 or ADC_REFERENCE::REF_EXT.
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 

  adc->adc0->setResolution(16); // set bits of resolution

  // ADC_CONVERSION_SPEED  =  VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
	// ADC_SAMPLING_SPEED    =  VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED

  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // Set the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);     // Set the sampling speed

  adc->adc0->recalibrate(); // Recalibrate adc0

  pinMode(A0,  INPUT);  // Set pin A0 as an input (AD)
  pinMode(A21, OUTPUT); // Set pin A21 as output (DA)
	 
  analogWriteResolution(12); // Set DA resolution (12 bits)
}

int val;

void loop()
{
  
	  val = adc->adc0->analogRead(A0);          // Read Analog value (AD)
	  analogWrite(A21,val/16);                  // Write digital value (DA)
    digitalWriteFast(6, !digitalReadFast(6)); // Toggle output pin 6 for cycle timming 

}