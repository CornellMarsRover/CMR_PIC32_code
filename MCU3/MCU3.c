/*
 * File:        MCU3
 * Author:      Matthew Filipek
 * Target PIC:  PIC32MX250F128B
 */



////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
#include "pt_cornell_1_2.h"
#include <plib.h>

////////////////////////////////////
// graphics libraries
#include "tft_gfx.h"
#include "tft_master.h"
#include "Adafruit_BNO055.h"
#include "I2C_Reg_defs.h"
#include "IR_Sensor.h"


char buffer[60]; // string buffer
//static int speedTarget; // target fan speed


#define I2CBAUD 10000 //clock operating at 10kHz
#define BRG_VAL  ((PBCLK/2/I2CBAUD)-2)
//#define I2CAddress 0x1E



static char I2CDataIn;
static unsigned char config1 = 0x88;
static unsigned char Status1 = 0xFF;
static unsigned char Status2;
static unsigned int M2_PWM;
static unsigned int M1_PWM;
static char M2_Dir;
static char M1_Dir;
static unsigned int M1_Pot;
static unsigned int M2_Pot;
static unsigned int M3_Pot;

static unsigned int CUR_Sense;
//BIT7(MSB) - Motor2, BIT6(MSB) - Motor1






static unsigned char Bat_Cutoff; //pin 14
//// command array
static char cmd[30];
static unsigned int IMUData[8];
static int value;
static int count;
#define IMUADD 0x28 //base
#define IMUADD2 0x29 //wrist

#define MIN_SERVO_DUTY       3000
#define MAX_SERVO_DUTY      50000
#define NEUTRAL_SERVO2_DUTY 3750
#define MAX_SERVO1_DUTY 2850
static int Servo1_PWM = 200;
static signed char Servo2_PWM;
static unsigned char servo1_ON; //1 if high, 0 if low
static unsigned char servo2_ON;
#define SERVO2_PER_UNIT 16 //15.6; 500 = 0.1 ms
#define SERVO1_PER_UNIT 15 //

static struct pt pt_ADC, pt_CSense, pt_I2C;


void InitI2C2(void) {
    CloseI2C2();
    OpenI2C2(I2C_EN | I2C_SLW_DIS | I2C_7BIT_ADD | I2C_SM_EN | I2C_RESTART_EN, BRG_VAL);

}

static unsigned int rcv;
float cels;
static unsigned char addr = 0x5A;
static unsigned int placeholder;

void SendIMUData(int reg_addr, int data, char address) {
    StartI2C2(); //Send the Start Bit
    IdleI2C2(); //Wait to complete

    MasterWriteI2C2(address << 1); //Sends the slave address over the I2C line.                                               
    IdleI2C2(); //Wait to complete

    MasterWriteI2C2(reg_addr); //Sends data byte over I2C line
    IdleI2C2(); //Wait to complete

    MasterWriteI2C2(data);
    IdleI2C2(); //wait for ack
    StopI2C2(); //Send the Stop condition
    IdleI2C2(); //Wait to complete
    return;
}

int RcvIMUData(unsigned int reg_addr, char IMUAddress) {
    //	//see page 92 of https://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
    rcv = 0;
    StartI2C2(); //Send line start condition
    IdleI2C2(); //Wait to complete
    MasterWriteI2C2(IMUAddress << 1); //Write slave addr WRITE (OR 0)
    IdleI2C2();
    MasterWriteI2C2(reg_addr); //BNO055_QUATERNION_DATA_W_LSB_ADDR
    IdleI2C2();

    RestartI2C2();
    IdleI2C2();

    MasterWriteI2C2((IMUAddress << 1) | 0x01); //Write the slave address, READ (OR 1)
    IdleI2C2();

    rcv |= MasterReadI2C2(); //Read in LSB
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    rcv |= MasterReadI2C2() << 8; //Read in MSB
    NotAckI2C2();
    IdleI2C2();
    StopI2C2(); //Send line stop condition
    IdleI2C2();
    return rcv;

}
static int temp;

static short IR_reading;


char checkI2CConnectivity(unsigned char Address) {
    char connected;
    StartI2C2(); //Send line start condition
    IdleI2C2(); //Wait to complete
    MasterWriteI2C2(Address << 1); //Write slave addr WRITE (OR 0)
    IdleI2C2();
    if (I2C2STATbits.ACKSTAT) {
        connected = 1;
    } else {
        connected = 0;
    }
    StopI2C2();
    return connected;
}

static PT_THREAD(protothread_I2C(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        IR_reading = RcvIRTemp();
        PT_YIELD_TIME_msec(100);

        IMUData[0] = RcvIMUData(BNO055_QUATERNION_DATA_W_LSB_ADDR, IMUADD);
        IMUData[1] = RcvIMUData(BNO055_QUATERNION_DATA_X_LSB_ADDR, IMUADD);
        IMUData[2] = RcvIMUData(BNO055_QUATERNION_DATA_Y_LSB_ADDR, IMUADD);
        IMUData[3] = RcvIMUData(BNO055_QUATERNION_DATA_Z_LSB_ADDR, IMUADD);
        PT_YIELD_TIME_msec(50);
        IMUData[4] = RcvIMUData(BNO055_QUATERNION_DATA_W_LSB_ADDR, IMUADD2);
        IMUData[5] = RcvIMUData(BNO055_QUATERNION_DATA_X_LSB_ADDR, IMUADD2);
        IMUData[6] = RcvIMUData(BNO055_QUATERNION_DATA_Y_LSB_ADDR, IMUADD2);
        IMUData[7] = RcvIMUData(BNO055_QUATERNION_DATA_Z_LSB_ADDR, IMUADD2);
        PT_YIELD_TIME_msec(50);

    }
    PT_END(pt);
}

static unsigned int POT1;
static unsigned int POT2;
static unsigned int MAG; //magnetic sensor
static unsigned int POT1In;
static unsigned int POT2In;
static unsigned int MAGIn; //magnetic sensor
#define ADC_PERIOD 20
#define ALPHA ((double)ADC_PERIOD)/500.0


static PT_THREAD(protothread_ADC(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {

        MAG = (1 - ALPHA) * MAG + ALPHA * ReadADC10(0);
        POT2 = (1 - ALPHA) * POT2 + ALPHA * ReadADC10(1);
        POT1 = (1 - ALPHA) * POT1 + ALPHA * ReadADC10(2);
        PT_YIELD_TIME_msec(ADC_PERIOD);
    }
    PT_END(pt);
}

static char M1_LastDir;
static char M2_LastDir;

static PT_THREAD(protothread_CSense(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        if (config1 & CUR_SENSE_EN) {
            if (Status2 & M1_CUR) {
                if (M1_LastDir != M1_Dir)
                    Status2 &= !M1_CUR;
            }
            if (Status2 & M2_CUR) {
                if (M2_LastDir != M2_Dir)
                    Status2 &= !M2_CUR;
            }
            if (mPORTBReadBits(BIT_7)) {
                M1_PWM = 0;
                SetDCOC1PWM(M1_PWM);
                Status2 |= M1_CUR;
                M1_LastDir = M1_Dir;
            }
            if (mPORTBReadBits(BIT_10)) {
                M2_PWM = 0;
                SetDCOC3PWM(M2_PWM);
                Status2 |= M2_CUR;
                M2_LastDir = M2_Dir;
            }
        } else {
            Status2 = 0;
        }
        PT_YIELD_TIME_msec(10);

    }
    PT_END(pt);

}

//void __ISR(_EXTERNAL_0_VECTOR, ipl1) INT0Interrupt(void) {
//    //tft_fillScreen(ILI9340_BLACK); //240x320 vertical display
//    count++;
//
//    // clear interrupt flag
//    mINT0ClearIntFlag();
//}

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void) { //empty ISR
    mT2ClearIntFlag(); //clear interrupt flag, if you forget to do this, the microcontroller will interrupt continuously
}

static int I2Cstate = 0;
static unsigned char I2C_request;

///////////////////////////////////////////////////////////////////
//
// Slave I2C interrupt handler
// This handler is called when a qualifying I2C events occurs
// this means that as well as Slave events
// Master and Bus Collision events will also trigger this handler.
//
///////////////////////////////////////////////////////////////////
static char writepointer = 0;
static unsigned char WDTCount = 0;

void __ISR(_I2C_1_VECTOR, ipl3) _SlaveI2CHandler(void) {
    unsigned char temp;
    static unsigned int dIndex;

    // check for MASTER and Bus events and respond accordingly
    if (IFS1bits.I2C1MIF) {//master interrupt flag
        mI2C1MClearIntFlag();
        return;
    }
    if (IFS1bits.I2C1BIF) {//bus collision interrupt
        I2Cstate = 0;
        I2C_request = 0;
        mI2C1BClearIntFlag();
        return;
    }

    // handle the incoming message
    if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0)) {
        // reset any state variables needed by a message sequence
        // perform a dummy read
        temp = SlaveReadI2C1();
        I2C1CONbits.SCLREL = 1; // release the clock
        I2Cstate = 0;
    } else if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1)) {//data received, input to slave
        WDTCount = 0; //reset watchdog
        WriteTimer1(0); //feed the watchdog

        // writing data to our module
        I2CDataIn = SlaveReadI2C1();
        I2C1CONbits.SCLREL = 1; // release clock stretch bit

        if (I2Cstate == 0 && I2CDataIn != ADDR_CLR_I2C_STATE) {
            I2Cstate = 1;
            I2C_request = I2CDataIn;
        } else if (I2Cstate == 1) {
            switch (I2C_request) {
                case ADDR_M1_PWM:
<<<<<<< HEAD
                    M1_Dir = I2CDataIn >> 7 & 0x01; //take top bit                    
=======
                    M1_Dir = I2CDataIn >> 7 & 0x01;//take top bit                    
>>>>>>> 0bf8f8db958bf40d6be8a98095a3e35310bfc7e7
                    if (((Status2 & M1_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M1_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_7);
                            M1_PWM = (((int) ((~I2CDataIn) + 1)) << 4);

                        } else {
                            mPORTBSetBits(BIT_7);
                            M1_PWM = ((int) I2CDataIn) << 4;

                        }
                    }
                    SetDCOC1PWM(M1_PWM);
                    break;
                case ADDR_M2_PWM:
<<<<<<< HEAD
                    M2_Dir = I2CDataIn >> 7 & 0x01; //take top bit
=======
                    M2_Dir = I2CDataIn >> 7 & 0x01;//take top bit
>>>>>>> 0bf8f8db958bf40d6be8a98095a3e35310bfc7e7
                    if (((Status2 & M2_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M2_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_10);
                            M2_PWM = (((int) ((~I2CDataIn) + 1)) << 4);
                        } else {
                            mPORTBSetBits(BIT_10);
                            M2_PWM = ((int) I2CDataIn) << 4;
                        }
                    }
                    SetDCOC3PWM(M2_PWM);

                    break;
                case ADDR_Config1:
                    config1 = I2CDataIn;
                    if (config1 & 0x20) {
                        SetDCOC1PWM(0);
                        SetDCOC3PWM(0);
                    } else {
                        SetDCOC1PWM(M1_PWM);
                        SetDCOC3PWM(M2_PWM);
                    }
                    break;
                case ADDR_Status2:
                    Status2 = I2CDataIn;
                    break;
                case ADDR_SERVO1_PWM:
                    Servo1_PWM = *(unsigned char*) (&I2CDataIn);
                    if (Servo1_PWM > 200) {
                        Servo1_PWM = 200;
                    }
                    break;
                case ADDR_SERVO2_PWM:
                    if (I2CDataIn & 0x80) {
                        Servo2_PWM = -1 * ((int) ((~I2CDataIn) + 1));
                    } else {
                        Servo2_PWM = (I2CDataIn);
                    }
                    break;
                default:
                    break;

            }
            I2Cstate = 0;
        }


    } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0)) {
        // read of the slave device, read the address
        temp = SlaveReadI2C1();
        if (I2Cstate == 1) {
            I2Cstate = 0;
            switch (I2C_request) {
                case ADDR_POT1:
                    SlaveWriteI2C1(POT1 >> 8);
                    break;
                case ADDR_POT2:
                    SlaveWriteI2C1(POT2 >> 8);
                    break;
                case ADDR_MAG:
                    SlaveWriteI2C1(MAG >> 8);
                    break;
                case ADDR_Status1:
                    SlaveWriteI2C1(Status1);
                    break;
                case ADDR_Status2:
                    SlaveWriteI2C1(Status2);
                    break;
                case QUATERNION_DATA_W:
                    SlaveWriteI2C1(IMUData[0] >> 8);
                    break;
                case QUATERNION_DATA_X:
                    SlaveWriteI2C1(IMUData[1] >> 8);
                    break;
                case QUATERNION_DATA_Y:
                    SlaveWriteI2C1(IMUData[2] >> 8);
                    break;
                case QUATERNION_DATA_Z:
                    SlaveWriteI2C1(IMUData[3] >> 8);
                    break;
                case QUATERNION_DATA2_W:
                    SlaveWriteI2C1(IMUData[4] >> 8);
                    break;
                case QUATERNION_DATA2_X:
                    SlaveWriteI2C1(IMUData[5] >> 8);
                    break;
                case QUATERNION_DATA2_Y:
                    SlaveWriteI2C1(IMUData[6] >> 8);
                    break;
                case QUATERNION_DATA2_Z:
                    SlaveWriteI2C1(IMUData[7] >> 8);
                    break;
                case ADDR_TEMP_SENSOR:
                    SlaveWriteI2C1(IR_reading >> 8);
                    break;
                default:
                    SlaveWriteI2C1(0);
                    I2C_request = 0;
                    break;
            }
        }
        dIndex = 0;
    } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 1)) {
        // output the data until the MASTER terminates the
        // transfer with a NACK, continuing reads return 0
        switch (I2C_request) {
            case ADDR_POT1:
                SlaveWriteI2C1(POT1 & 255);
                break;
            case ADDR_POT2:
                SlaveWriteI2C1(POT2 & 255);
                break;
            case ADDR_MAG:
                SlaveWriteI2C1(MAG & 255);
                break;
            case QUATERNION_DATA_W:
                SlaveWriteI2C1(IMUData[0] & 255);
                break;
            case QUATERNION_DATA_X:
                SlaveWriteI2C1(IMUData[1] & 255);
                break;
            case QUATERNION_DATA_Y:
                SlaveWriteI2C1(IMUData[2] & 255);
                break;
            case QUATERNION_DATA_Z:
                SlaveWriteI2C1(IMUData[3] & 255);
                break;
            case QUATERNION_DATA2_W:
                SlaveWriteI2C1(IMUData[4] & 255);
                break;
            case QUATERNION_DATA2_X:
                SlaveWriteI2C1(IMUData[5] & 255);
                break;
            case QUATERNION_DATA2_Y:
                SlaveWriteI2C1(IMUData[6] & 255);
                break;
            case QUATERNION_DATA2_Z:
                SlaveWriteI2C1(IMUData[7] & 255);
                break;
            case ADDR_TEMP_SENSOR:
                SlaveWriteI2C1(IR_reading);
                break;
            default:
                SlaveWriteI2C1(0);
                I2C_request = 0;
                break;
        }
    }
    mI2C1SClearIntFlag();
}

void InitI2C(void) {
    CloseI2C1();

    // Enable the I2C module with clock stretching enabled
    // define a Master BRG of 400kHz although this is not used by the slave
    OpenI2C1(I2C_SLW_DIS | I2C_ON | I2C_7BIT_ADD | I2C_STR_EN | I2C_SM_DIS, BRG_VAL); //48);
    //    I2C1BRG = BRG_VAL;
    //    I2C1CON = 0xD040;
    //    I2C1CON &= !I2C_SM_EN;
    //    I2C1CON |= 0x40;
    // set the address of the slave module
    I2C1ADD = I2CAddress;
    I2C1MSK = 0;
    // I2C1CON |= 0x8000;
    // configure the interrupt priority for the I2C peripheral
    mI2C1SetIntPriority(I2C_INT_PRI_3 | I2C_INT_SLAVE);

    // clear pending interrupts and enable I2C interrupts
    mI2C1SClearIntFlag();
    EnableIntSI2C1;
}

<<<<<<< HEAD
=======
void InitI2C2(void) {

    OpenI2C2(I2C_EN | I2C_SLW_DIS | I2C_7BIT_ADD | I2C_SM_EN | I2C_RESTART_EN, BRG_VAL);

}
>>>>>>> 0bf8f8db958bf40d6be8a98095a3e35310bfc7e7


//== Servo ========================================================

void __ISR(_TIMER_3_VECTOR, ipl3) T3Interrupt(void) {
    //signed int per = Servo1_PWM * SERVO_PER_UNIT;

    if (servo1_ON) {
        //int x = MAX_SERVO_DUTY  - (NEUTRAL_SERVO_DUTY + per);
        WritePeriod3(MAX_SERVO_DUTY); //- (NEUTRAL_SERVO_DUTY + Servo1_PWM * SERVO_PER_UNIT));
        servo1_ON = 0;
        mPORTAClearBits(BIT_3);
    } else {
        WritePeriod3(MAX_SERVO1_DUTY - (Servo1_PWM << 3));
        servo1_ON = 1;
        mPORTASetBits(BIT_3);
    }
    mT3ClearIntFlag();
} // T3 Interrupt

void __ISR(_TIMER_4_VECTOR, ipl3) T4Interrupt(void) {
    //signed int per = Servo2_PWM * SERVO_PER_UNIT;
    if (servo2_ON) {
        WritePeriod4(MAX_SERVO_DUTY); //  - (NEUTRAL_SERVO_DUTY + Servo2_PWM * SERVO_PER_UNIT));
        servo2_ON = 0;
        mPORTBClearBits(BIT_4);
    } else {
        WritePeriod4(NEUTRAL_SERVO2_DUTY + Servo2_PWM * SERVO2_PER_UNIT);
        servo2_ON = 1;
        mPORTBSetBits(BIT_4);
    }
    mT4ClearIntFlag();
} // T4 Interrupt

inline void setWDT(void) {
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 60000);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);


}

<<<<<<< HEAD
=======
void __ISR(_TIMER_1_VECTOR, ipl1) WatchdogInt(void) {
    WDTCount++;
    if (WDTCount > 3) {//~1sec
        WDTCount = 3; //prevent overflow
        SetDCOC1PWM(0);
        SetDCOC3PWM(0);
        Servo1_PWM = 200;
        Servo2_PWM = 0;
    }
    mT1ClearIntFlag();
}
>>>>>>> 0bf8f8db958bf40d6be8a98095a3e35310bfc7e7

// === Main  ======================================================

void main(void) {

    SYSTEMConfigPerformance(sys_clock); //clock CPU at 40MHz

    // === I2C Init ================
    InitI2C(); //configure as slave on bus 1
    InitI2C2(); //configure as master on bus 2

    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
    AD1CON2 = 0;
    mJTAGPortEnable(0);


    PT_setup();
    RPA0R = 0;
    RPA1R = 0;
    CM2CON = 0;
    CVRCON = 0x0000;
    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
    count = 0;

    RPB15R = 0x05; //output compare 1, pin 26, M1_PWM
    RPB14R = 0x05; //output compare 3, pin 25, M2_PWM



    //mPORTAClearBits(BIT_0);

    //    RB13, RB11
    mPORTBSetPinsDigitalIn(BIT_13 | BIT_11);

    //This pin is externally shorted to AN0 (pin2), set as input for high impedance
    mPORTASetPinsDigitalIn(BIT_2);

    mPORTASetPinsDigitalOut(BIT_3); //
    mPORTBSetPinsDigitalOut(BIT_15 | BIT_14 | BIT_4 | BIT_7 | BIT_10); //

    //B7 is DIR1
    //B10 is DIR2

    // external interrupt0:
    mINT0IntEnable(TRUE);
    mINT0SetIntPriority(INT_PRIORITY_LEVEL_1);

    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_4, 2049); //~20KHz
    ConfigIntTimer2(T2_INT_OFF | T2_INT_PRIOR_2);

    //    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 60000);//operating freq of servos is ~666 Hz
    //    ConfigIntTimer3(T3_INT_OFF| T3_INT_PRIOR_3);

    // set up OC for PWM
    ConfigIntOC1(OC_INT_PRIOR_5 | EXT_INT_SUB_PRI_2);
    ConfigIntOC3(OC_INT_PRIOR_7 | EXT_INT_SUB_PRI_3);


    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);



    // the ADC ///////////////////////////////////////
    // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
#define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

    // define setup parameters for OpenADC10
    // ADC ref external  | disable offset test | enable scan mode | do 1 sample | use single buf | alternate mode off
    //note samples per int MUST be at least equal to the number of ADC channels you are reading from
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_3 | ADC_BUF_16 | ADC_ALT_INPUT_OFF
    //
    // Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
#define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_30 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

    // define setup parameters for OpenADC10
    // set AN4 and  as analog inputs
#define PARAM4	ENABLE_AN0_ANA|ENABLE_AN1_ANA | ENABLE_AN2_ANA


    // define setup parameters for OpenADC10
    // do not assign channels to scan
#define PARAM5	SKIP_SCAN_AN3|SKIP_SCAN_AN4|SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8| SKIP_SCAN_AN9|SKIP_SCAN_AN10|SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // use ground as neg ref for A | use AN4 for input A     
    // configure to sample AN4 
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF); // configure to sample AN4 
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC 



    INTEnableSystemMultiVectoredInt();
    while (!mAD1GetIntFlag()) {
    } // wait for the first conversion to complete so there will be valid data in ADC result registers

    OpenTimer3(T3_ON | T3_PS_1_16 | T3_SOURCE_INT, MAX_SERVO_DUTY);
    ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);
    mT3IntEnable(1); // enable timer3 interrupts


    //Timer, servo 2 pwm
    OpenTimer4(T4_ON | T4_PS_1_16 | T4_SOURCE_INT, MAX_SERVO_DUTY);
    ConfigIntTimer4(T4_INT_OFF | T4_INT_PRIOR_3);
    mT4IntEnable(1);

    //round robin thread schedule
    PT_INIT(&pt_ADC);
<<<<<<< HEAD
//    PT_INIT(&pt_CSense);
=======
    PT_INIT(&pt_CSense);
>>>>>>> 0bf8f8db958bf40d6be8a98095a3e35310bfc7e7
    PT_INIT(&pt_I2C);
    unsigned int OPR_CODE = 0x3D;
    SendIMUData(OPR_CODE, 0x00, IMUADD); //sets IMU mode to CONFIG    
    delay_ms(10);
    SendIMUData(OPR_CODE, 0x0C, IMUADD); //sets IMU mode to NDOF
    delay_ms(50);
    SendIMUData(OPR_CODE, 0x00, IMUADD2); //sets IMU mode to CONFIG    
    delay_ms(10);
    SendIMUData(OPR_CODE, 0x0C, IMUADD2); //sets IMU mode to NDOF
    delay_ms(10);
    IMUData[0] = 1;
    IMUData[4] = 1;
    //    setWDT();
<<<<<<< HEAD

    while (1) {
        PT_SCHEDULE(protothread_I2C(&pt_I2C));
        PT_SCHEDULE(protothread_ADC(&pt_ADC));
        
        //        PT_SCHEDULE(protothread_CSense(&pt_CSense));
=======
    while (1) {
        PT_SCHEDULE(protothread_ADC(&pt_ADC));
        PT_SCHEDULE(protothread_I2C(&pt_I2C));
        PT_SCHEDULE(protothread_CSense(&pt_CSense));
        //        PT_SCHEDULE(protothread_anim(&pt_anim));
>>>>>>> 0bf8f8db958bf40d6be8a98095a3e35310bfc7e7
        I2C1CONbits.SCLREL = 1; // release the clock
    }
} // main


// === end  ======================================================
