//  AD_DA_Test.ino
// implement interrupt to do the proportional control

// ADC 16 bits 0-65535 and 0-3.3 volts
// DAC 12 bits 0-4095  and 0-3.3 volts (conversion factor between DAC and ADC = 16)

#include <ADC.h>
#include <ADC_util.h>

IntervalTimer myTimer; //create interrupt function

ADC *adc = new ADC(); // adc object

volatile unsigned int System_Input, Sensor_Input, Control_Output;
volatile float Max_Voltage, System_Input_v, Sensor_Input_v, Error, Control, Control_correction;
float Ts, samplingFreq, Ts_micro;

//State space variables
volatile float C_k;
volatile float C_km1 = 0.0;
volatile float x1_k;
volatile float x1_km1 = 0.0;
volatile float x2_k;
volatile float x2_km1 = 0.0;
volatile float x3_k;
volatile float x3_km1 = 0.0;
volatile float xI_k;
volatile float xI_km1 = 0.0;
volatile float rk_m1;

volatile float A1_11 = 0.8965;
volatile float A1_12 = 0.00019985;
volatile float A1_13 = 0.000000019262;
volatile float A1_21 = -13.8959;
volatile float A1_22 = 0.9994;
volatile float A1_23 = 0.00019437;
volatile float A1_31 = -198.4238;
volatile float A1_32 = -5.5727;
volatile float A1_33 = 0.9439;

volatile float B_1 = 0.0;
volatile float B_2 = 0.0011;
volatile float B_3 = 11.4151;

volatile float K_i = 0.0479;

volatile float K_1 = 19.3998;
volatile float K_2 = 0.4447;
volatile float K_3 = 0.0036;

volatile float D_1 = 1.0;
volatile float D_2 = 0.0;
volatile float D_3 = 0.0;

volatile float L_1 = 0.1035;
volatile float L_2 = 13.8737;
volatile float L_3 = -23.0279;


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

  samplingFreq = 5000.0; //define sampling frequency in Hz
  Ts = 1/samplingFreq; //sampling in seconds
  Ts_micro = Ts * 1000000.0; //sampling in microseconds

  Max_Voltage = 5.0;   // Maximum Voltage = 5 volts because of the gain of 2 of the power amplifier
  Control_correction = 0.0; // Control correction to center the response
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
	
	// Error   = (System_Input_v - Sensor_Input_v);					      // Error (units are voltage +- 10 volts)     This equation impliments the Feedback
	Control = K_i*xI_km1 - (K_1*x1_km1 + K_2*x2_km1 + K_3*x3_km1);   // Control (units are voltage  +- 10 volts)  This equation impliments the control equation

    Control -= Control_correction;    // shift the control signal to center the signal
	if(fabs(Control) >= Max_Voltage){
        Control = copysign(Max_Voltage,Control); // Check Maximum voltage
    }								            
	Control_Output = floor((Control + 10.)*4095./20.);  // Convert control voltage to a digital number for output
	                                                    // Note the output bipolar range +- 10 Volts is mapped to the digital range 0 to 4095
	analogWrite(A21,Control_Output);                    // Write digital value (DA)

    C_k = Sensor_Input_v; //Pot data

    x1_k = (A1_11*x1_km1 + A1_12*x2_km1 + A1_13*x3_km1) + B_1*K_i*xI_km1 + L_1*C_km1;
    x2_k = (A1_21*x1_km1 + A1_22*x2_km1 + A1_23*x3_km1) + B_2*K_i*xI_km1 + L_2*C_km1;
    x3_k = (A1_31*x1_km1 + A1_32*x2_km1 + A1_33*x3_km1) + B_3*K_i*xI_km1 + L_3*C_km1;

    xI_k = xI_km1 + (System_Input_v - C_k);

    //Update variables
    x1_km1 = x1_k;
    x2_km1 = x2_k;
    x3_km1 = x3_k;

    xI_km1 = xI_k;

    C_km1 = C_k;    

    digitalWriteFast(6, !digitalReadFast(6));     					    // Toggle output pin 6 for cycle timming 

    // Serial.print("x1_km1: ");
    // Serial.print(x1_km1);
    // Serial.print(", x2_km1: ");
    // Serial.print(x2_km1);   
    // Serial.print(", x3_km1: ");
    // Serial.print(x3_km1); 

    // Serial.print(", Sensor_Input_v ");
    // Serial.print(Sensor_Input_v);

    // Serial.print(", Sensor_Input ");
    // Serial.print(Sensor_Input);

    // Serial.print(", Control ");
    // Serial.print(Control);

    // Serial.print(", System_Input_v ");
    // Serial.print(System_Input_v);

    // Serial.print(", xI_km1 ");
    // Serial.println(xI_km1);
}