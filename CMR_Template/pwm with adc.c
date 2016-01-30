/*********************************************************************
 *
 *  This example reads samples from the ADC and copies them
 * to the Vref DAC using a timer ISR
 *
 * AN4 (pin 6 on the 28-PDIP) is connected thru a 1k resistor to a signal gen
 * Vref DAC output (pin 25) is connected to the scope
 *********************************************************************
 * Bruce Land, Cornell University
 * May 30, 2014
 ********************************************************************/
// all peripheral library includes
#include <plib.h>
#include <math.h>

// Configuration Bit settings
// SYSCLK = 40 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//
#pragma config FNOSC = FRCPLL, POSCMOD = HS, FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF
// frequency we're running at
#define	SYS_FREQ 40000000

// volatiles for the stuff used in the ISR
volatile unsigned int DAC_value; // Vref output
volatile int CVRCON_setup; // stores the voltage ref config register after it is set up
volatile unsigned int channel4;	// conversion result as read from result buffer
volatile unsigned int dutycycle;

// Timer 2 interrupt handler ///////
// ipl2 means "interrupt priority level 2"
// ASM output is 47 instructions for the ISR
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // clear the interrupt flag
    mT2ClearIntFlag();
    // read the ADC
    // read the first buffer position
    channel4 = ReadADC10(0);   // read the result of channel 4 conversion from the idle buffer
    dutycycle = channel4 * PR3 / 1024;
    
    AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below
    // DAC output
    DAC_value = channel4>>6 ; //10-bit to 4 bit
    CVRCON = CVRCON_setup | DAC_value ;
}

// main ////////////////////////////
int main(void)
{

	// Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    
    //pwm-------------------------------------------------
    RPA0R = 0x05;   
    
    //mPORTAClearBits(BIT_0);	
    mPORTASetPinsDigitalOut(BIT_0);

    OpenOC1( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 4000);
    
    //SetDCOC1PWM(dutycycle);  
    //end pwm---------------------------------------------
    
    // the ADC ///////////////////////////////////////
    // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
        // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
        // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
        // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
        #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF //

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
        //
	// Define setup parameters for OpenADC10
        // use peripherial bus clock | set sample time | set ADC clock divider
        // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
        // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
        #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// define setup parameters for OpenADC10
	// set AN4 and  as analog inputs
	#define PARAM4	ENABLE_AN4_ANA 

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL

	// use ground as neg ref for A | use AN4 for input A     
	// configure to sample AN4 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4 ); // configure to sample AN4 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC

        // Vref DAC /////////////////////////////////
        // set up the Vref pin and use as a DAC
        // enable module| eanble output | use low range output | use internal reference | desired step
        CVREFOpen( CVREF_ENABLE | CVREF_OUTPUT_ENABLE | CVREF_RANGE_LOW | CVREF_SOURCE_AVDD | CVREF_STEP_0 );
        // And read back setup from CVRCON for speed later
        // 0x8060 is enabled with output enabled, Vdd ref, and 0-0.6(Vdd) range
        CVRCON_setup = CVRCON; //CVRCON = 0x8060 from Tahmid http://tahmidmc.blogspot.com/

        // timer interrupt //////////////////////////
        // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
        // 500 kHz i2 80 cycles
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 80);

        // set up the timer interrupt with a priority of 2
        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
        mT2ClearIntFlag(); // and clear the interrupt flag

        // setup system wide interrupts  ///
        INTEnableSystemMultiVectoredInt();

        // set up i/o port pin 
        mPORTBClearBits(BIT_0);		//Clear bits to ensure light is off.
        mPORTBSetPinsDigitalOut(BIT_0);    //Set port as output

	while(1)
	{		
        // toggle a bit for ISR perfromance measure
        mPORTBToggleBits(BIT_0);
        SetDCOC1PWM(dutycycle);  
 	}
	return 0;
}






