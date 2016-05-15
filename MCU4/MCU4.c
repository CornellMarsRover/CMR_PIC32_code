/*
 * File:        MCU4
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
#include "I2C_Reg_defs.h"


char buffer[60]; // string buffer
//static int speedTarget; // target fan speed


//#define I2CBAUD 10000 //clock operating at 10kHz
//#define BRG_VAL  ((PBCLK/2/I2CBAUD)-2)
//#define I2CAddress 0x1F;



static char I2CDataIn;
static unsigned char config1 = 0x88;
static unsigned char Status1 = 0xFF;
static unsigned char Status2 = 0x00;
static unsigned int M7_PWM; //pin 2
static unsigned int M6_PWM; //pin 3
static unsigned int M5_PWM; //pin 4
static unsigned int M4_PWM; //pin 9
static unsigned int M3_PWM; //pin 6
static char M7_Dir;
static char M6_Dir;
static char M5_Dir;
static char M4_Dir;
static char M3_Dir;


static unsigned int CUR_SENSE;
//BIT7(MSB)= M7, BIT6 = M6
//BIT5 = M5, BIT4 = M4
//BIT3 = M3

static unsigned int Bat_Cutoff;


//// command array
static char cmd[30];
static int value;
static int count;

static struct pt pt_CSense;
static char M3_LastDir;
static char M4_LastDir;
static char M5_LastDir;
static char M6_LastDir;
static char M7_LastDir;
static PT_THREAD(protothread_CSense(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {

        if (config1 & CUR_SENSE_EN) {
            if(Status2 & M7_CUR){
                if(M7_LastDir != M7_Dir)//if direction has changed, lower CS flag, which enables PWM writing
                    Status2 &= !M7_CUR; 
            }
            if(Status2 & M6_CUR){
                if(M6_LastDir != M6_Dir)
                    Status2 &= !M6_CUR; 
            }
            if(Status2 & M6_CUR){
                if(M6_LastDir != M6_Dir)
                    Status2 &= !M6_CUR; 
            }
            if(Status2 & M5_CUR){
                if(M5_LastDir != M5_Dir)
                    Status2 &= !M5_CUR; 
            }
            if(Status2 & M4_CUR){
                if(M4_LastDir != M4_Dir)
                    Status2 &= !M4_CUR; 
            }
            if(Status2 & M3_CUR){
                if(M3_LastDir != M3_Dir)
                    Status2 &= !M3_CUR; 
            }            
            if (mPORTBReadBits(BIT_3)) {//CS triggered!
                M7_PWM = 0;
                SetDCOC1PWM(M7_PWM);//Halt motor
                Status2 |= M7_CUR;//set CS flag
                M7_LastDir = M7_Dir;//save direction
                
            }
            if (mPORTBReadBits(BIT_1)) {
                M6_PWM = 0;
                SetDCOC2PWM(M6_PWM);
                Status2 |= M6_CUR;
                M6_LastDir = M6_Dir;
                
            }
            if (mPORTAReadBits(BIT_3)) {
                M5_PWM = 0;
                SetDCOC3PWM(M5_PWM);
                Status2 |= M5_CUR;
                M5_LastDir = M5_Dir;
                
            }
            if (mPORTBReadBits(BIT_4)) {
                M4_PWM = 0;
                SetDCOC5PWM(M4_PWM);
                Status2 |= M4_CUR;
                M4_LastDir = M4_Dir;
                
            }
            if (mPORTAReadBits(BIT_4)) {
                M3_PWM = 0;
                SetDCOC4PWM(M3_PWM);
                Status2 |= M3_CUR;
                M3_LastDir = M3_Dir;
            }
        }
        PT_YIELD_TIME_msec(10);

    }
    PT_END(pt);

}

// external interrupt

void __ISR(_EXTERNAL_0_VECTOR, ipl1) INT0Interrupt(void) {
    //tft_fillScreen(ILI9340_BLACK); //240x320 vertical display
//w    count++;

    // clear interrupt flag
    mINT0ClearIntFlag();
}

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
static unsigned char WDTCount = 0;
void __ISR(_I2C_1_VECTOR, ipl3) _SlaveI2CHandler(void) {
    unsigned char temp;
    static unsigned int dIndex;

    // check for MASTER and Bus events and respond accordingly
    if (IFS1bits.I2C1MIF) {
        mI2C1MClearIntFlag();
        return;
    }
    if (IFS1bits.I2C1BIF) {//bus collision, reset I2C state machine
        I2Cstate = 0;
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
        WDTCount = 0;
        WriteTimer1(0);
        
        // writing data to our module
        I2CDataIn = SlaveReadI2C1();
        I2C1CONbits.SCLREL = 1; // release clock stretch bit

        if (I2Cstate == 0 && I2CDataIn != ADDR_CLR_I2C_STATE) {
            I2Cstate = 1;
            I2C_request = I2CDataIn;
        } else if (I2Cstate == 1) {
            switch (I2C_request) {
                case ADDR_M3_PWM:
                    M3_Dir = I2CDataIn >> 7 & 0x01;//take top bit                    
                    
                    if (((Status2 & M3_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M3_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_13);
                                M3_PWM = (((int) ((~I2CDataIn) + 1)) << 4);
                            
                        } else {
                            
                            mPORTBSetBits(BIT_13);
                            
                            M3_PWM = ((int) I2CDataIn) << 4;
                        }
                    }
                    SetDCOC4PWM(M3_PWM);
                    break;
                case ADDR_M4_PWM:
                    M4_Dir = I2CDataIn >> 7 & 0x01;//take top bit                    
                    
                    if (((Status2 & M4_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M4_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_14);
                            M4_PWM = (((int) ((~I2CDataIn) + 1)) << 4);

                        } else {
                            mPORTBSetBits(BIT_14);
                            M4_PWM = ((int) I2CDataIn) << 4;
                        }
                    }
                    SetDCOC5PWM(M4_PWM);
                    break;
                case ADDR_M5_PWM:
                    M5_Dir = I2CDataIn >> 7 & 0x01;//take top bit                    
                    
                    if (((Status2 & M5_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M5_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_15);
                            M5_PWM = (((int) ((~I2CDataIn) + 1)) << 4);

                        } else {
                            mPORTBSetBits(BIT_15);
                            M5_PWM = ((int) I2CDataIn) << 4;
                        }
                    }
                    SetDCOC3PWM(M5_PWM);
                    break;
                case ADDR_M6_PWM:
                    M6_Dir = I2CDataIn >> 7 & 0x01;//take top bit                    
                    
                    if (((Status2 & M6_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M6_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_11);
                            M6_PWM = (((int) ((~I2CDataIn) + 1)) << 4);

                        } else {
                            mPORTBSetBits(BIT_11);
                            M6_PWM = ((int) I2CDataIn) << 4;
                        }
                    }
                    SetDCOC2PWM(M6_PWM);
                    break;
                case ADDR_M7_PWM:
                    M7_Dir = I2CDataIn >> 7 & 0x01;//take top bit                    
                    
                    if (((Status2 & M7_CUR) && (config1 & CUR_SENSE_EN)) || (config1 & MOTOR_HALT)) {
                        M7_PWM = 0;
                    } else {
                        if (I2CDataIn & 0x80) {
                            mPORTBClearBits(BIT_7);
                            M7_PWM = (((int) ((~I2CDataIn) + 1)) << 4);

                        } else {
                            mPORTBSetBits(BIT_7);
                            M7_PWM = ((int) I2CDataIn) << 4;
                        }
                    }
                    SetDCOC1PWM(M7_PWM);
                    break;
                case ADDR_Config1:
                    config1 = I2CDataIn;
                    if (config1 & MOTOR_HALT) {
                        SetDCOC1PWM(0);
                        SetDCOC2PWM(0);
                        SetDCOC3PWM(0);
                        SetDCOC4PWM(0);
                        SetDCOC5PWM(0);
                    } else {
                        SetDCOC1PWM(M7_PWM);
                        SetDCOC2PWM(M6_PWM);
                        SetDCOC3PWM(M5_PWM);
                        SetDCOC4PWM(M3_PWM);
                        SetDCOC5PWM(M4_PWM);
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
        dIndex = 0;
        SlaveWriteI2C1(0xF6);
    } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 1)) {
        // output the data until the MASTER terminates the
        // transfer with a NACK, continuing reads return 0
        if (dIndex == 0) {
            SlaveWriteI2C1(0x66);
            dIndex++;
        } else
            SlaveWriteI2C1(0);

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

inline void setWDT(void){
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 60000);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);
    
    
}

void __ISR(_TIMER_1_VECTOR, ipl1) WatchdogInt(void)
{
    WDTCount ++;
    if(WDTCount > 3){//~1sec
        WDTCount = 3; //prevent overflow
        SetDCOC1PWM(0);
        SetDCOC2PWM(0);
        SetDCOC3PWM(0);
        SetDCOC4PWM(0);
        SetDCOC5PWM(0);
    }
    mT1ClearIntFlag();  
}

// === Main  ======================================================

void main(void) {
    SYSTEMConfigPerformance(sys_clock); //clock CPU at 40MHz

    // === I2C Init ================
    InitI2C();
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

    RPA0R = 0x05; //output compare 1
    RPA1R = 0x05; //output compare 2
    RPB0R = 0x05; //output compare 3
    RPB2R = 0x05; //output compare 4
    RPA2R = 0x06; //output compare 5


    //mPORTAClearBits(BIT_0);
    mPORTASetPinsDigitalIn(BIT_3 | BIT_4); //CS5, CS3
    mPORTBSetPinsDigitalIn(BIT_1 | BIT_3 | BIT_4); //CS6, CS7, CS4

    mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2); //PWMs
    mPORTBSetPinsDigitalOut(BIT_0 | BIT_2 | BIT_7 | BIT_11 | BIT_15 | BIT_14 | BIT_13); //PWMs & DIR

    // external interrupt0:
    mINT0IntEnable(TRUE);
    mINT0SetIntPriority(INT_PRIORITY_LEVEL_1);

    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_4, 2049);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);

    // set up OC for PWM
    ConfigIntOC1(OC_INT_PRIOR_0 | EXT_INT_SUB_PRI_2);
    ConfigIntOC2(OC_INT_PRIOR_1 | EXT_INT_SUB_PRI_2);
    ConfigIntOC3(OC_INT_PRIOR_2 | EXT_INT_SUB_PRI_2);
    ConfigIntOC4(OC_INT_PRIOR_3 | EXT_INT_SUB_PRI_2);
    ConfigIntOC5(OC_INT_PRIOR_4 | EXT_INT_SUB_PRI_2);



    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC5(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);


    // init the threads

//    setWDT();
    INTEnableSystemMultiVectoredInt();


    //round robin thread schedule
    while (1) {
//        PT_SCHEDULE(protothread_CSense(&pt_CSense));
        I2C1CONbits.SCLREL = 1; // release the clock
    }
} // main


// === end  ======================================================
