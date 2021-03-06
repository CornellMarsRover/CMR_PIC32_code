/*
 * File:        MCU2
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
#include "PWM_logic.h"
#include "I2C_Reg_defs.h"


#define DRIVE
char buffer[60]; // string buffer
static int IntThresh = 0;

#define I2CBAUD 100000 //clock operating at 10kHz
#define BRG_VAL  ((PBCLK/2/I2CBAUD)-2)
//#define I2CAddress 0x1D;



static char I2CDataIn;
static unsigned char byteReceved;
static unsigned char config1 = 0x0E;
static unsigned char Status1 = 0xFF;
static int RF_PWM;
static int RF_P = 23;
static int RF_I = 5;
static int RF_D = 5;
static int RM_PWM;
static int RM_P = 23;
static int RM_I = 5;
static int RM_D = 5;
static int RB_PWM;
static int RB_P = 23;
static int RB_I = 5;
static int RB_D = 5;
static unsigned char halted;

static int RF_Speed;
static int RM_Speed;
static int RB_Speed;
static unsigned char Bat_lvl;
static unsigned int Hum_sense;


////---I2C Register Addresses---
//
//
////Config
//#define ADDR_CLR_I2C_STATE 0xFF
//#define ADDR_Config1  0x00 
//
///* BIT7(MSB) = PID_EN
// * 
// * PID_EN - If enabled, PWM addresses represent target speeds, if
// * disabled, they will be interpreted as duty cycles. Note: in all cases, motor
// * speed/effort is set with a signed 8-bit number
// * 
// * BIT6 = LowPowerShutoff_EN
// * 
// * LowPowerShutoff_EN - MCU will halt all motors if low battery signal received
// * 
// * BIT5 = MOTOR_HALT
// * 
// * MOTOR_HALT - MCU will halt all motors
// * 
// * BIT4 = RAMP_DOWN_EN
// * 
// * RAMP_DOWN_EN - MCU will set a maximum speed delta; any speed changes
// * that exceed this delta will be done gradually rather than
// * instantaneously
// * 
// * BIT3 = MCU_TIMEOUT_EN
// * 
// * MCU_TIMEOUT - MCU will set all motor efforts to 0 if no command has been received 
// * for 2 seconds
// * 
// * BIT2 = ENCODER_DISCONECT_BEHAVIOR
// * 
// * ENCODER_DISCONECT_BEHAVIOR - When bit is set: If the encoder for a given 
// * motor is disconnected, * MCU will set  effort level of motor to average 
// * effort level of motors on the same bank. When bit is cleared: all motors on 
// * bank will halt (note: only half of the rover will stop)
// * 
// * BIT1 = 6_WHEEL_EN
// * 
// * 6_WHEEL_EN - when bit is set: 6 wheel drive is activated, otherwise 4 wheel 
// * drive is used
// * 
// * 
//
//Default state of config1 = 0x8C
// */
//#define PID_EN 0x80
//#define SIX_WHEEL_EN 0x02
//
//
////Read-only
//#define ADDR_Status1  0x01
///*
// * BIT7(MSB) = RF_ENC_STAT
// * BIT6 = RM_ENC_STAT
// * BIT5 = RB_ENC_STAT
// * 
// * XX_ENC_STAT - Bit indicates if the encoder for the given motor is
// * connected, 1 for connected, 0 for disconnected
// * 
// * BIT4 = RF_STALL
// * BIT3 = RM_STALL
// * BIT2 = RB_STALL
// * 
// * XX_STALL - Bit indicates if the given motor has stalled, i.e. it is
// * being given a sufficiently high PWM duty cycle, but is still not
// * moving. This may also indicate an improper connection to the motor. 0
// * if stalled, 1 if nominal
// * 
// * BIT1 = BAT_CUTOFF
// * 
// * BAT_CUTOFF - Bit indicates the rover's battery is running low, 1 if
// * nominal, 0 if low
//
//Default state of register = 0xFF
// */
#define RF_ENC_STAT 0x80
#define RM_ENC_STAT 0x40
#define RB_ENC_STAT 0x20


////Motor addresses
//#define ADDR_RF_PWM 0x10 //NOTE, motor speeds are signed, 2s compliment numbers
//#define ADDR_RF_P 0x11
//#define ADDR_RF_I 0x12
//#define ADDR_RF_D 0x13
//#define ADDR_RM_PWM 0x14
//#define ADDR_RM_P 0x15
//#define ADDR_RM_I 0x16
//#define ADDR_RM_D 0x17
//#define ADDR_RB_PWM 0x18
//#define ADDR_RB_P 0x19
//#define ADDR_RB_I 0x1A
//#define ADDR_RB_D 0x1B
//
////Motor Bank Addresses
//#define ADDR_ALL_PWM 0x20 //NOTE, motor speeds are signed, 2s compliment numbers
//#define ADDR_ALL_P  0x21
//#define ADDR_ALL_I  0x22
//#define ADDR_ALL_D  0x23
//#define ADDR_HALT  0x2F
//
//
////----Read-only addresses----
//
//#define ADDR_RF_SPEED 0x30 //NOTE, motor speeds are signed, 2s compliment numbers
//#define ADDR_RM_SPEED 0x31
//#define ADDR_RB_SPEED 0x32
//#define ADDR_BAT_LVL 0x33
//#define ADDR_HUM_SENSE 0x34

//// command array
static char cmd[30];
static int value;
static int count_RF;
static int count_RM;
static int count_RB;

// === the fixed point macros ========================================
typedef signed int fix16;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b))))
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a))))
#define absfix16(a) abs(a)

// UART parameters
#define BAUDRATE 9600 // must match PC end
#define PB_DIVISOR (1 << OSCCONbits.PBDIV) // read the peripheral bus divider, FPBDIV
#define PB_FREQ SYS_FREQ/PB_DIVISOR // periperhal bus frequency

// useful ASCII/VT100 macros for PuTTY
#define clrscr() printf( "\x1b[2J")
#define home()   printf( "\x1b[H")
#define pcr()    printf( '\r')
#define crlf     putchar(0x0a); putchar(0x0d);
#define max_chars 50 // for input buffer

#define FOSC 60E6
#define PB_DIV 8
#define PRESCALE 256
#define T1_TICK (FOSC/PB_DIV/PRESCALE*64)

#define CONFIG (CN_ON | CN_IDLE_CON)
#define PINS (CN15_ENABLE)
#define PULLUPS (CN_PULLUP_DISABLE_ALL)


static int RF_error;
static int RF_integral;
static int RF_effort;
static int last_RF_effort;
static int RF_lastSpeed;
static int RM_error;
static int RM_integral;
static int RM_effort;
static int last_RM_effort;
static int RM_lastSpeed;
static int RB_error;
static int RB_integral;
static int RB_effort;
static int last_RB_effort;
static int RB_lastSpeed;

#define STOP_SPEED 2300

#define MIN_SERVO_DUTY       3000
#define MAX_SERVO_DUTY      50000
#define MAX_SERVO_DUTY 2950
static int Servo_PWM = 200;
static unsigned char servo_ON; //1 if high, 0 if low
#define SERVO2_PER_UNIT 16 //15.6; 500 = 0.1 ms
#define SERVO1_PER_UNIT 6 //

//// receive function prototype (see below for code)
//// int GetDataBuffer(char *buffer, int max_size);
//
//// === thread structures ============================================
//// thread control structs
static struct pt pt_pid, pt_input, pt_output, pt_DMA_output, pt_ADC, pt_ENC;

static PT_THREAD(protothread_ADC(struct pt *pt)){
    PT_BEGIN(pt);
    while(1){
        Hum_sense = ReadADC10(0);
//        tft_setCursor(0,0);
//        tft_fillRect(0,0,240, 50, ILI9340_BLACK);
//        tft_setTextColor(ILI9340_WHITE);
//        tft_setTextSize(2);
//        sprintf(buffer, "%d %d", RB_effort, RB_PWM);
//        tft_writeString(buffer);
        
        PT_YIELD_TIME_msec(30);
    }
    PT_END(pt);
}

static int ENC_TEST_THRESHOLD = 1000;

static PT_THREAD(protothread_ENC(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        if (!(Status1 & RF_ENC_STAT)) {//if encoder already unplugged
            if ((Status1 & RB_ENC_STAT) && ((config1 & SIX_WHEEL_EN) && (Status1 & RM_ENC_STAT))) {// 
                RF_effort = (RM_effort + RB_effort) >> 1; //set speed to average of remaining motors
            } else if ((config1 & SIX_WHEEL_EN) && (Status1 & RM_ENC_STAT)) {
                RF_effort = RM_effort;
            } else if ((Status1 & RB_ENC_STAT)) {
                RF_effort = RB_effort;
            } else {
                RF_effort = STOP_SPEED;
            }

            CNPUB |= 0x02; //pull-up enable
            CNPDB &= !0x02;
            PT_YIELD_TIME_msec(1);
            if (!(PORTB & 0x02)) {//if not pulled high
                Status1 |= 0x80;
                CNPUB &= !0x02;
            } else {
                CNPDB |= 0x02; //pull-down
                CNPUB &= !0x02;
                PT_YIELD_TIME_msec(1);
                if (PORTB & 0x02) {
                    Status1 |= 0x80;
                    CNPDB &= !0x02;
                }

            }
        }
        if (!(Status1 & 0x40)) {//if encoder already unplugged
            if ((Status1 & 0x20) && ((Status1 & 0x80))) {//if 
                RM_effort = (RM_effort + RB_effort) >> 1; //set speed to average of remaining motors
            } else if ((Status1 & 0x20)) {
                RM_effort = RB_effort;
            } else if ((Status1 & 0x80)) {
                RM_effort = RF_effort;

            } else {
                RM_effort = 2000;
            }

            CNPUB |= 0x04; //pull-up enable
            CNPDB &= !0x04;
            PT_YIELD_TIME_msec(1);
            if (!(PORTB & 0x04)) {//if not pulled high
                Status1 |= 0x40;
                CNPUB &= !0x04;
            } else {
                CNPDB |= 0x04; //pull-down
                CNPUB &= !0x04;
                PT_YIELD_TIME_msec(1);
                if (PORTB & 0x04) {
                    Status1 |= 0x40;
                    CNPDB &= !0x04;

                }

            }
        }
        if (!(Status1 & 0x20)) {//if encoder already unplugged
            if ((Status1 & 0x80) && ((Status1 & 0x40) && (config1 & 0x02))) {//if 
                RB_effort = (RM_effort + RF_effort) >> 1; //set speed to average of remaining motors
            } else if ((Status1 & 0x40)&&(config1 & 0x02)) {
                RB_effort = RM_effort;
            } else if ((Status1 & 0x80)) {
                RB_effort = RF_effort;

            } else {
                RB_effort = 2000;
            }

            CNPUB |= 0x01; //pull-up enable
            CNPDB &= !0x01;
            PT_YIELD_TIME_msec(1);
            if (!(PORTB & 0x01)) {//if not pulled high
                Status1 |= 0x20;
                CNPUB &= !0x01;
            } else {
                CNPDB |= 0x01; //pull-down
                CNPUB &= !0x01;
                PT_YIELD_TIME_msec(1);
                if (PORTB & 0x01) {
                    Status1 |= 0x20;
                    CNPDB &= !0x01;

                }

            }
        }

        if ((RF_Speed == 0) && ((RF_effort > (2000 + ENC_TEST_THRESHOLD)) || (RF_effort < (2000 - ENC_TEST_THRESHOLD)))) {
            CNPUB |= 0x02; //pull-up enable
            CNPDB &= !0x02;
            PT_YIELD_TIME_msec(1);
            if (PORTB & 0x02) {//if pin pulled high
                CNPUB &= !0x02; //pull-up disable
                CNPDB |= 0x02; //pull-down enable
                PT_YIELD_TIME_msec(1); //leave time for value to settle
                if (!(PORTB & 0x02)) {//if pin pulled low
                    Status1 &= !0x80;
                    if ((Status1 & 0x20) && ((config1 & 0x02) && (Status1 & 0x40))) {//if 
                        RF_effort = (RM_effort + RB_effort) >> 1; //set speed to average of remaining motors
                    } else if ((config1 & 0x02) && (Status1 & 0x40)) {
                        RF_effort = RM_effort;
                    } else if ((Status1 & 0x20)) {
                        RF_effort = RB_effort;
                    } else {
                        RF_effort = 2000;
                    }
                }
            }

            CNPUB &= !0x02; //
            CNPDB &= !0x02; //            
        }
        if ((RM_Speed == 0) && ((RM_effort > (2000 + ENC_TEST_THRESHOLD)) || (RM_effort < (2000 - ENC_TEST_THRESHOLD)) && (config1 & 0x02))) {
            CNPUB |= 0x04; //pull-up enable
            CNPDB &= !0x04; //pull-up enable
            PT_YIELD_TIME_msec(1); //let pin value settle
            if (PORTB & 0x04) {//if pin pulled high
                CNPUB &= !0x04; //pull-up disable
                CNPDB |= 0x04; //pull-down enable
                PT_YIELD_TIME_msec(1); //leave time for value to settle
                if (!(PORTB & 0x04)) {//if pin pulled low
                    Status1 &= !0x40;
                    if ((Status1 & 0x20) && ((Status1 & 0x80))) {//if 
                        RM_effort = (RM_effort + RB_effort) >> 1; //set speed to average of remaining motors
                    } else if ((Status1 & 0x20)) {
                        RM_effort = RB_effort;
                    } else if ((Status1 & 0x80)) {
                        RM_effort = RF_effort;

                    } else {
                        RM_effort = 2000;
                    }
                }
            }

            CNPUB &= !0x04; //
            CNPDB &= !0x04; //            
        }

        if ((RB_Speed == 0) && ((RB_effort > (2000 + ENC_TEST_THRESHOLD)) || (RB_effort < (2000 - ENC_TEST_THRESHOLD)))) {
            CNPUB |= 0x01; //pull-up enable
            CNPDB &= !0x01;
            PT_YIELD_TIME_msec(1); //let pin value settle
            if (PORTB & 0x01) {//if pin pulled high
                CNPUB &= !0x01; //pull-up disable
                CNPDB |= 0x01; //pull-down enable
                PT_YIELD_TIME_msec(1); //leave time for value to settle
                if (!(PORTB & 0x01)) {//if pin pulled low
                    Status1 &= !0x20;
                    if ((Status1 & 0x80) && ((Status1 & 0x40) && (config1 & 0x02))) {//if 
                        RB_effort = (RM_effort + RF_effort) >> 1; //set speed to average of remaining motors
                    } else if ((Status1 & 0x40)&&(config1 & 0x02)) {
                        RB_effort = RM_effort;
                    } else if ((Status1 & 0x80)) {
                        RB_effort = RF_effort;

                    } else {
                        RB_effort = 2000;
                    }
                }
            }

            CNPUB &= !0x01; //
            CNPDB &= !0x01; //  
        }





        SetDCOC2PWM(RM_effort);
        SetDCOC1PWM(RB_effort);
        SetDCOC3PWM(RF_effort);
        PT_YIELD_TIME_msec(20);
    }
    PT_END(pt);
}

static int RF_dir;
static int RM_dir;
static int RB_dir;
static int RF_reverse;
static int RM_reverse;
static int RB_reverse;
static int last_last_RB_effort;
static int last_last_RM_effort;
static int last_last_RF_effort;
signed short RB_speed_out;
#define MIN_STOP_SPEED 2100
#define MAX_STOP_SPEED 2500

static PT_THREAD(protothread_pid(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {

        if (Status1 & RF_ENC_STAT) {
            
            if(RF_effort > MIN_STOP_SPEED && RF_effort < MAX_STOP_SPEED){
                if(last_RF_effort != RF_effort){//check for divide by zero condition
                    RF_dir = (RF_Speed- RF_lastSpeed)/(RF_effort - last_RF_effort) ;//+((RB_Speed- RB_lastSpeed)/(RB_effort - last_RB_effort)- (RB_lastSpeed- RB_last_last_speed)/(last_RB_effort - last_last_RB_effort))/(RB_effort-last_last_RB_effort);// ds/de gives direction information    
                }
            }
            else if (RF_effort <= MIN_STOP_SPEED){//effort very small, speed is certainly negative
                RF_dir = -1;
            }
            else if (RF_effort >= MAX_STOP_SPEED){//effort very large, positive speed
                RF_dir = 1;
            }
            else{
                RF_dir = RF_reverse;
            }
            
            if(RF_dir < 0) {
                if(RF_Speed > 0){
                    RF_Speed *= -1;
                }
                RF_reverse = -1;//save direction
            }
            else if(RF_dir == 0 && (RF_reverse==-1)){
                if(RF_Speed>0){
                    RF_Speed *= -1;
                }//in special case when speed did not change, latch last direction
            }
            else{
                if(RF_Speed < 0){
                    RF_Speed *= -1; 
                }
                RF_reverse = 1; //save direction
            }            
            
            
            
            RF_error = RF_PWM - RF_Speed; // get error from difference

            if (abs(RF_error) > IntThresh) {
                RF_integral += RF_error;
            } else {
                RF_integral *= 0.9;
            }
            if (RF_integral > 400) {
                RF_integral = 400;
            } else if (RF_integral < -400) {
                RF_integral = -400;
            }
            RF_effort += ((RF_error * RF_P) /16.0) +((RF_integral * RF_I) /64.0) + (((RF_lastSpeed - RF_Speed) * RF_D)/32.0);
            
            if(RF_effort > 3999){
                RF_effort = 3999;
            }
            else if(RF_effort < 1){
                RF_effort = 1;
            }
            
        } 


        if (Status1 & RM_ENC_STAT && config1 & SIX_WHEEL_EN) {
            if(RM_effort > MIN_STOP_SPEED && RM_effort < MAX_STOP_SPEED){
                if(last_RM_effort != RM_effort ){//check for divide by zero condition
                    RM_dir = (RM_Speed- RM_lastSpeed)/(RM_effort - last_RM_effort) ;// ds/de gives direction information    
                }
            }
            else if (RM_effort <= MIN_STOP_SPEED){//effort very small, speed is certainly negative
                RM_dir = -1;
            }
            else if (RM_effort >= MAX_STOP_SPEED){//effort very large, positive speed
                RM_dir = 1;
            }
            else{
                RM_dir = RM_reverse;
            }
            
            if(RM_dir < 0) {
                if(RM_Speed > 0){
                    RM_Speed *= -1;
                }
                RM_reverse = -1;//save direction
            }
            else if(RM_dir == 0 && (RM_reverse==-1)){
                if(RM_Speed>0){
                    RM_Speed *= -1;
                }//in special case when speed did not change, latch last direction
            }
            else{
                if(RM_Speed < 0){
                    RM_Speed *= -1; 
                }
                RM_reverse = 1; //save direction
            }            
            
            
            
            RM_error = RM_PWM - RM_Speed; // get error from difference

            if (abs(RM_error) > IntThresh) {
                RM_integral += RM_error;
            } else {
                RM_integral *= 0.9;
            }
            if (RM_integral > 400) {
                RM_integral = 400;
            } else if (RM_integral < -400) {
                RM_integral = -400;
            }
            RM_effort += ((RM_error * RM_P)/16.0) +((RM_integral * RM_I)/64.0) + (((RM_lastSpeed - RM_Speed) * RM_D)/32.0);
            
            if(RM_effort > 3999){
                RM_effort = 3999;
            }
            else if(RM_effort < 1){
                RM_effort = 1;
            }
            
        } 

        if (Status1 & RB_ENC_STAT) {
            
            if(RB_effort > MIN_STOP_SPEED && RB_effort < MAX_STOP_SPEED){
                if(last_RB_effort != RB_effort){//check for divide by zero condition
                    RB_dir = (RB_Speed- RB_lastSpeed)/(RB_effort - last_RB_effort) ;//+((RB_Speed- RB_lastSpeed)/(RB_effort - last_RB_effort)- (RB_lastSpeed- RB_last_last_speed)/(last_RB_effort - last_last_RB_effort))/(RB_effort-last_last_RB_effort);// ds/de gives direction information    
                }
            }
            else if (RB_effort <= MIN_STOP_SPEED){//effort very small, speed is certainly negative
                RB_dir = -1;
            }
            else if (RB_effort >= MAX_STOP_SPEED){//effort very large, positive speed
                RB_dir = 1;
            }
            else{
                RB_dir = RB_reverse;
            }
            
            if(RB_dir < 0) {
                if(RB_Speed > 0){
                    RB_Speed *= -1;
                }
                RB_reverse = -1;//save direction
            }
            else if(RB_dir == 0 && (RB_reverse==-1)){
                if(RB_Speed>0){
                    RB_Speed *= -1;
                }//in special case when speed did not change, latch last direction
            }
            else{
                if(RB_Speed < 0){
                    RB_Speed *= -1; 
                }
                RB_reverse = 1; //save direction
            }            
            RB_speed_out = RB_Speed;
            
            
            RB_error = RB_PWM - RB_Speed; // get error from difference

            if (abs(RB_error) > IntThresh) {
                RB_integral += RB_error;
            } else {
                RB_integral *= 0.9;
            }
            if (RB_integral > 400) {
                RB_integral = 400;
            } else if (RB_integral < -400) {
                RB_integral = -400;
            }
            RB_effort += ((RB_error * RB_P)/16.0) +((RB_integral * RB_I)/64.0) + (((RB_lastSpeed - RB_Speed) * RB_D)/32.0);
            
            if(RB_effort > 3999){
                RB_effort = 3999;
            }
            else if(RB_effort < 1){
                RB_effort = 1;
            }
            
        } 
        
//        tft_fillRect(0,0,240,30, ILI9340_BLACK);
//        tft_setTextColor(ILI9340_WHITE);
//        tft_setTextSize(2);
//        tft_setCursor(0,0);
//        sprintf(buffer, "%d %d %d", RB_Speed, RB_effort, RB_PWM);
//        tft_writeString(buffer);
        
        RM_lastSpeed = RM_Speed;
        RB_lastSpeed = RB_Speed;
        RF_lastSpeed = RF_Speed;
        last_RB_effort = RB_effort;
        last_RM_effort = RM_effort;
        last_RF_effort = RF_effort;
        SetDCOC2PWM(RM_effort);
        SetDCOC1PWM(RB_effort);
        SetDCOC3PWM(RF_effort);
        PT_YIELD_TIME_msec(20);
    }
    PT_END(pt);
}

void __ISR(_EXTERNAL_1_VECTOR, ipl1) INT1Interrupt(void) {
    count_RB++;

    // clear interrupt flag
    mINT1ClearIntFlag();
}

void __ISR(_EXTERNAL_2_VECTOR, ipl2) INT2Interrupt(void) {
    count_RM++;
    
    // clear interrupt flag
    mINT2ClearIntFlag();
}

void __ISR(_EXTERNAL_3_VECTOR, ipl3) INT3Interrupt(void) {
    count_RF++;

    // clear interrupt flag
    mINT3ClearIntFlag();
}

void __ISR(_TIMER_2_VECTOR, ipl4) Timer2Handler(void) { //empty ISR

    RM_Speed = count_RM;//record number of encoder pulses
    RF_Speed = count_RF;
    RB_Speed = count_RB;



    count_RM = 0;//zero counter variables
    count_RF = 0;
    count_RB = 0;
    mT2ClearIntFlag(); //clear interrupt flag, if you forget to do this, the microcontroller will interrupt continuously
}

static int I2Cstate = 0;
static unsigned char I2C_request;

void __ISR(_TIMER_3_VECTOR, ipl5) Timer3Handler(void) { //empty ISR
    mT3ClearIntFlag(); //clear interrupt flag, if you forget to do this, the microcontroller will interrupt continuously
}

///////////////////////////////////////////////////////////////////
//
// Slave I2C interrupt handler
// This handler is called when a qualifying I2C events occurs
// this means that as well as Slave events
// Master and Bus Collision events will also trigger this handler.
//
///////////////////////////////////////////////////////////////////

void SendData3 (int data1, int data2, int data3,  unsigned int address){
	StartI2C1();	        //Send the Start Bit
	IdleI2C1();		//Wait to complete

	MasterWriteI2C1((address << 1));  //Sends the slave address over the I2C line.  This must happen first so the 
                                             //proper slave is selected to receive data.
	IdleI2C1();	        //Wait to complete

	MasterWriteI2C1(data1);  //Sends data byte over I2C line
	IdleI2C1();		//Wait to complete
    MasterWriteI2C1(data2);  //Sends data byte over I2C line
	IdleI2C1();		//Wait to complete
    MasterWriteI2C1(data3);
    IdleI2C1();
	StopI2C1();	        //Send the Stop condition
	IdleI2C1();	        //Wait to complete

} //end function

static unsigned char WDTCount = 0;
void __ISR(_I2C_1_VECTOR, ipl3) _SlaveI2CHandler(void) {
    unsigned char temp;
    static unsigned int dIndex;

    // check for MASTER and Bus events and respond accordingly
    if (IFS1bits.I2C1MIF) {
        mI2C1MClearIntFlag();
        return;
    }
    if (IFS1bits.I2C1BIF) {//bus collision
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
        WriteTimer1(0);//feed the watchdog
        
        // writing data to our module
        I2CDataIn = SlaveReadI2C1();
        byteReceved = 1;
        I2C1CONbits.SCLREL = 1; // release clock stretch bit

        if (I2Cstate == 0 && I2CDataIn != ADDR_CLR_I2C_STATE) {
            I2Cstate = 1;
            I2C_request = I2CDataIn;
        } else if (I2Cstate == 1) {
            switch (I2C_request) {
                case ADDR_ALL_PWM:
                    if (config1 & PID_EN){
                        RF_PWM = getPWM(I2CDataIn);
                        RM_PWM = RF_PWM;
                        RB_PWM = RF_PWM;
                    }
                    else{
                        RF_effort = getEffort(I2CDataIn);
                        RM_effort = RF_effort;
                        RB_effort = RF_effort;
                    }
                    break;
                case ADDR_ALL_P:
                    RF_P = *(unsigned char*)(&I2CDataIn);;
                    RM_P = *(unsigned char*)(&I2CDataIn);;
                    RB_P = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_ALL_I:
                    RF_I = *(unsigned char*)(&I2CDataIn);;
                    RM_I = *(unsigned char*)(&I2CDataIn);;
                    RB_I = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_ALL_D:
                    RF_D = *(unsigned char*)(&I2CDataIn);;
                    RM_D = *(unsigned char*)(&I2CDataIn);;
                    RB_D = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_F_PWM:
                    if (config1 & PID_EN){
                        RF_PWM = getPWM(I2CDataIn);
                    }
                    else{
                        RF_effort = getEffort(I2CDataIn);
                    }
                    break;
                case ADDR_F_P:
                    RF_P = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_F_I:
                    RF_I = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_F_D:
                    RF_D = *(unsigned char*)(&I2CDataIn);;
                    break;

                case ADDR_M_PWM:
                    if (config1 & PID_EN){
                        RM_PWM = getPWM(I2CDataIn);
                    }
                    else{
                        RM_effort = getEffort(I2CDataIn);
                    }
                    break;
                case ADDR_M_P:
                    RM_P = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_M_I:
                    RM_I = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_M_D:
                    RM_D = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_B_PWM:
                    if (config1 & PID_EN){
                        RB_PWM = getPWM(I2CDataIn);
                    }
                    else{
                        RB_effort = getEffort(I2CDataIn);
                    }
                    break;
                case ADDR_B_P:
                    RB_P = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_B_I:
                    RB_I = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_B_D:
                    RB_D = *(unsigned char*)(&I2CDataIn);;
                    break;
                case ADDR_Config1:
                    config1 = I2CDataIn;
                    break;
                case ADDR_LATCH:
                    if(I2CDataIn){
                        mPORTBSetBits(BIT_10);
                    }
                    else{
                        mPORTBClearBits(BIT_10);
                    }
                    break;
                default:
                    break;

            }
            I2Cstate = 0;
            I2C_request = 0;
        }


    } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0)) {
        // read of the slave device, read the address
        temp = SlaveReadI2C1();//dummy read
        
        if(I2C_request == ADDR_HUM_SENSE){
            SlaveWriteI2C1(Hum_sense >> 8);            
        }
        else if(I2C_request == ADDR_Status1){
            SlaveWriteI2C1(Status1);            
        }
        else if(I2C_request == ADDR_B_SPEED){
            SlaveWriteI2C1(RB_speed_out>>8);
        }
        else{
            SlaveWriteI2C1(0x00);
            I2C_request = 0;
        }
    } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 1)) {
        // output the data until the MASTER terminates the
        // transfer with a NACK, continuing reads return 0
//        temp = SlaveReadI2C1();//dummy read
        if(I2C_request == ADDR_HUM_SENSE){
            SlaveWriteI2C1(Hum_sense & 255);            
        }
        else if(I2C_request == ADDR_B_SPEED){
            SlaveWriteI2C1(RB_speed_out & 255);
        }
        else{
            SlaveWriteI2C1(0x00);
            I2C_request = 0;
        }

    }

    mI2C1SClearIntFlag();
}

void InitI2C(void) {

    CloseI2C1();

    // Enable the I2C module with clock stretching enabled
    // define a Master BRG of 400kHz although this is not used by the slave
    OpenI2C1( I2C_SLW_DIS |I2C_ON | I2C_7BIT_ADD | I2C_STR_EN | I2C_SM_DIS, BRG_VAL); //48); //
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
        if(config1 & PID_EN){
            RM_PWM = 0;
            RB_PWM = 0;
            RF_PWM = 0;
        }
        else{
            RM_effort = 2250;
            RB_effort = 2250;
            RF_effort = 2250;
        }
    }
    mT1ClearIntFlag();  
}


void __ISR( _TIMER_4_VECTOR, ipl3) T4Interrupt( void){
    if (servo_ON){
        //int x = MAX_SERVO_DUTY  - (NEUTRAL_SERVO_DUTY + per);
        WritePeriod4(MAX_SERVO_DUTY); //- (NEUTRAL_SERVO_DUTY + Servo1_PWM * SERVO_PER_UNIT));
        servo_ON = 0;
        mPORTBClearBits(BIT_10);
    }
    else { 
        WritePeriod4(MAX_SERVO_DUTY - (Servo_PWM<<3));
        servo_ON = 1;
        mPORTBSetBits(BIT_10);
    }
    mT4ClearIntFlag(); 
} // T4 Interrupt

// === Main  ======================================================

void main(void) {

    SYSTEMConfigPerformance(sys_clock); //clock CPU at 40MHz
    // === I2C Init ================
    InitI2C();

    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
    AD1CON2 = 0;
    mJTAGPortEnable(0);

    
    
    
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
#define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

    // define setup parameters for OpenADC10
    // set AN4 and  as analog inputs
#define PARAM4	ENABLE_AN9_ANA


    // define setup parameters for OpenADC10
    // do not assign channels to scan
#define PARAM5	SKIP_SCAN_AN0|SKIP_SCAN_AN1|SKIP_SCAN_AN2|SKIP_SCAN_AN3|SKIP_SCAN_AN4|SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8| SKIP_SCAN_AN10|SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // use ground as neg ref for A | use AN4 for input A     
    // configure to sample AN4 
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF); // configure to sample AN4 
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC 

    PT_setup();



    RPB3R = 0x05; // OC 1, RB
    RPA1R = 0x05; // OC 2, RM
    RPB14R = 0x05; // OC 3, RF

    mPORTBSetPinsDigitalOut(BIT_14|BIT_3|BIT_10);
    mPORTASetPinsDigitalOut(BIT_1);
    mPORTASetPinsDigitalIn(BIT_0|BIT_2);
    mPORTBSetPinsDigitalIn(BIT_0|BIT_1|BIT_2);
    //    CNPUA |= 0xFF;
    //    ODCA |= 0xC0;
    mPORTBClearBits(BIT_10);
    // external interrupt0:

    INT1R = 0x02; //pin 4, RB_ENC
    INT3R = 0x02; //pin 5, RF_ENC
    INT2R = 0x04; //pin 6, RM_ENC


    ConfigINT1(EXT_INT_ENABLE | RISING_EDGE_INT |EXT_INT_PRI_1);
    ConfigINT2(EXT_INT_ENABLE | RISING_EDGE_INT |EXT_INT_PRI_2);
    ConfigINT3(EXT_INT_ENABLE | RISING_EDGE_INT |EXT_INT_PRI_3);

    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 4000);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);

    OpenTimer3(T3_ON|T3_SOURCE_INT|T3_PS_1_8,4000);
    ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_2);
    
//    // set up OC for PWM
    CloseOC1();
    ConfigIntOC1(OC_INT_PRIOR_5 | EXT_INT_SUB_PRI_2);
    OpenOC1(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 2250, 2250); //RB
    ConfigIntOC2(OC_INT_PRIOR_5 | EXT_INT_SUB_PRI_2);
    OpenOC2(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 2250, 2250); //RM
    ConfigIntOC3(OC_INT_PRIOR_5 | EXT_INT_SUB_PRI_2);
    OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 2250, 2250); //RF


    RF_PWM = 0;
    RM_PWM = 0;
    RB_PWM = 0;

    RF_effort = 2250;
    RM_effort = 2250;
    RB_effort = 2250;
    SetDCOC2PWM(RM_effort);
    SetDCOC1PWM(RB_effort);
    SetDCOC3PWM(RF_effort);

    // init the threads
    PT_INIT(&pt_ENC);
    PT_INIT(&pt_pid);
    PT_INIT(&pt_ADC);
    //round robin thread schedule
//    tft_init_hw();
//    tft_begin();
//    tft_fillScreen(ILI9340_BLACK); //240x320 vertical display
    setWDT();
    INTEnableSystemMultiVectoredInt();
   
    //Timer, servo pwm
//    OpenTimer4( T4_ON | T4_PS_1_16 | T4_SOURCE_INT, MAX_SERVO_DUTY);
//    ConfigIntTimer4(T4_INT_OFF| T4_INT_PRIOR_3);
//    mT4IntEnable(1);      // enable timer4 interrupts
    
    
    while (1) {
        if(config1 & PID_EN){
//            PT_SCHEDULE(protothread_ENC(&pt_ENC));
            PT_SCHEDULE(protothread_pid(&pt_pid));
        }
        else{
            SetDCOC2PWM(RM_effort);
            SetDCOC1PWM(RB_effort);
            SetDCOC3PWM(RF_effort);
        }
        PT_SCHEDULE(protothread_ADC(&pt_ADC));
        I2C1CONbits.SCLREL = 1; // release the clock
    }
} // main
//RF_DIR

// === end  ======================================================
