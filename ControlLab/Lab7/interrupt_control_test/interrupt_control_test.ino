//  AD_DA_Test.ino
// implement interrupt to do the proportional control

// ADC 16 bits 0-65535 and 0-3.3 volts
// DAC 12 bits 0-4095  and 0-3.3 volts (conversion factor between DAC and ADC = 16)

#include <ADC.h>
#include <ADC_util.h>

IntervalTimer myTimer; //create interrupt function

ADC *adc = new ADC(); // adc object

unsigned int System_Input, Sensor_Input, Control_Output;
float Max_Voltage, Kp, System_Input_v, Sensor_Input_v, Error, Control, Control_correction;
float Ts, samplingFreq, Ts_micro;

void setup()
{
  Serial.begin(38400);
  pinMode(6, OUTPUT); // Set Pin 6 as an Output Pin
    
   ///// ADC0 ////
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 or ADC_REFERENCE::REF_EXT.
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 

  adc->adc0->setResolution(16); // set bits of ADC resolution

  // ADC_CONVERSION_SPEED  =  VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // ADC_SAMPLING_SPEED    =  VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED

  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // Set the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);     // Set the sampling speed

  adc->adc0->recalibrate(); // Recalibrate adc0

  pinMode(A0,  INPUT);  // Set pin A0 as an input (AD)
  pinMode(A1,  INPUT);  // Set pin A1 as an input (AD)
  pinMode(A21, OUTPUT); // Set pin A21 as output (DA)
  pinMode(A22, OUTPUT); // Set pin A22 as output (DA)
	 
  analogWriteResolution(12); // Set DA resolution (12 bits)

  samplingFreq = 500; //define sampling frequency in Hz
  Ts = 1/samplingFreq; //sampling in seconds
  Ts_micro = Ts * 1000000.0; //sampling in microseconds

  Max_Voltage = 5.0;   // Maximum Voltage = 5 volts because of the gain of 2 of the power amplifier
  Kp          = 0.5;   // Proportional control constant
  Control_correction = 0.22; // Control correction to center the response
  myTimer.begin(ControlLoop, Ts_micro); //control loop to run at 1kHz
}

void loop(){
  //does nothing waiting for interrupt timer
}

void ControlLoop()
{
  
    System_Input = adc->adc0->analogRead(A0);          				  // Read Analog value channel A0 (AD) - Function Generator Input (0-3.3 volts) after the B2SE circuit
	
	  System_Input_v = (float) System_Input*(20./65535.)- 10.0;		// Convert the System_Input digital value (0-65535) to a voltage
																	                              // Note the input (0-65535) digital range is mapped to a bipolar +- 10 volts range
																	                              // Note that the (20./65535.) term needs the decimal point
																	                              // or else it is interrupted as an integer and the result is zero
																	                              // The underscore v (_v) notation denotes +- 10 volt units
		
    Sensor_Input = adc->adc0->analogRead(A1);          				  // Read Analog value channel A1 (AD) - Sensor Input (0-3.3 volts) after the B2SE circuit
	  Sensor_Input_v = (float) Sensor_Input*(20./65535.)- 10.0;		// Convert the Sensor_Input digital value (0-65535) to a voltage
																	                              // Note the input (0-65535) digital range is mapped to a bipolar +- 10 volts range
																	                              // Note that the (20./65535.) term needs the decimal point
																	                              // or else it is interrupted as an integer and the result is zero
																	                              // The underscore v (_v) notation denotes +- 10 volt units	
	
    
	  Error   = (System_Input_v - Sensor_Input_v);					      // Error (units are voltage +- 10 volts)     This equation impliments the Feedback
	  Control = Kp * Error;  						    				              // Control (units are voltage  +- 10 volts)  This equation impliments the control equation

    Control -= Control_correction;    // shift the control signal to center the signal
	  if(fabs(Control) >= Max_Voltage)								            // Check Maximum voltage
	  Control = copysign(Max_Voltage,Control);

	  Control_Output = floor((Control + 10.)*4095./20.);  			  // Convert control voltage to a digital number for output
																	                              // Note the output bipolar range +- 10 Volts is mapped to the digital range 0 to 4095
		
	  analogWrite(A21,Control_Output);                  				  // Write digital value (DA)

    digitalWriteFast(6, !digitalReadFast(6));     					    // Toggle output pin 6 for cycle timming 

}