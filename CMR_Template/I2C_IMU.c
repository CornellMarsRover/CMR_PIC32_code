/*
 * File:        main
 * Author:      Matthew Filipek
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
#include "Adafruit_BNO055.h"
#include "pt_cornell_1_2.h"

////////////////////////////////////
// graphics libraries
//#include "tft_master_spi2.c"
#include "tft_gfx.h"
#include "tft_master.h"
#define I2CBAUD 20000
#define BRG_VAL  ((PBCLK/2/I2CBAUD)-2)

char buffer[60]; // string buffer
//static int speedTarget; // target fan speed
//static float propGain; //   p
//static float intGain; //    i
//static float diffGain; //   d
//static int speed = 0;
//static int target; 
//
//// command array
static char cmd[30];
static int value;
static unsigned int rcv; 
//static unsigned int rcv; 

// === the fixed point macros ========================================
typedef signed int fix16 ;
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

//// receive function prototype (see below for code)
//// int GetDataBuffer(char *buffer, int max_size);
//
//// === thread structures ============================================
//// thread control structs
static struct pt pt_uart, pt_pid, pt_anim, pt_input, pt_output, pt_DMA_output, pt_I2C_Master;




volatile unsigned int temp; //temp received from i2c
float cels;
static unsigned char addr = 0x5A;
static unsigned int placeholder;



/*****************************************************
 * RcvData(unsigned int address)             *
 *                           *
 * Gets a byte of data from I2C slave device at      *
 *  ADDRESS.                         *
 *                           *
 * Returns: Received data                *
 ****************************************************/
int RcvData(unsigned int address) {
    StartI2C2();                //Send line start condition
    IdleI2C2();                 //Wait to complete
    MasterWriteI2C2((address << 1) | 1);    //Write out slave address OR 1 (read command)
    IdleI2C2();             //Wait to complete
    rcv = MasterReadI2C2();     //Read in a value  
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    StopI2C2();             //Send line stop condition
    IdleI2C2();             //Wait co complete
    return rcv;             //Return read value
}

//static PT_THREAD(protothread_I2C(struct pt *pt)){
//    PT_BEGIN(pt);
//    while(1){
//        
//        PT_YIELD_TIME_msec(500);
//    }   
//    PT_END(pt);
//}
static PT_THREAD(protothread_uart(struct pt *pt)) {
    // this thread interacts with the PC keyboard to take user input and set up PID parameters
    PT_BEGIN(pt);
    // send the prompt via DMA to serial
        sprintf(PT_send_buffer, "%s", "cmd>");
        // by spawning a print thread
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));//send date and time
    
    while (1) {
        buffer[0] = RcvIMUData(BNO055_EULER_H_LSB_ADDR);
        buffer[1] = RcvIMUData(BNO055_EULER_R_LSB_ADDR);
        buffer[2] = RcvIMUData(BNO055_EULER_P_LSB_ADDR);
//        temp = RcvIMUData(BNO055_ACCEL_DATA_X_LSB_ADDR);
        
        

        sprintf(PT_send_buffer, "%s     %s%d, %s%d, %s%d, %s%d%s", "Accelerometer value: " , "W: ", buffer[0], "X: "
                , buffer[1], "Y: ", buffer[2], "Z: ", buffer[3],"\n\r");
//        sprintf(PT_send_buffer, "%s     %s%d, %s%d, %s%d%s", "Accelerometer value: " , "W: ", buffer[0], "X: "
//                , buffer[1], "Y: ", buffer[2],"\n\r");
        // by spawning a print thread
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));//send date and time

        
//        //spawn a thread to handle terminal input
//        // the input thread waits for input
//        // -- BUT does NOT block other threads
//        // string is returned in "PT_term_buffer"
//        PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));//wait for  input
//        sscanf(PT_term_buffer, "%s %f", cmd, &value);
//                         
//        // echo
//        sprintf(PT_send_buffer,"%04x%s", rcv, "\n");//send original message
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\n");//next line
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\r");//carriage return
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                
        
       
        PT_YIELD_TIME_msec(500);
    } // while(1)
    PT_END(pt);
} // uart input thread


static PT_THREAD(protothread_anim(struct pt *pt)) {
    // runs the LCD and updates around 5/second
    PT_BEGIN(pt);
    //write to screen
    tft_fillRect(0,0, 50, 50, ILI9340_BLACK);//write black over previous message
    tft_setCursor(0, 0);
    tft_setTextColor(ILI9340_WHITE);  
    tft_setTextSize(1);//smallest size 
    sprintf(buffer,"%s%d", "time since startup:\n", PT_GET_TIME()/1000 );
    tft_writeString(buffer);
    
    PT_YIELD_TIME_msec(30);
    PT_END(pt);
} // animation thread

void __ISR(_TIMER_2_VECTOR, ipl2) Timer3Handler(void) { //empty ISR
    mT2ClearIntFlag();//clear interrupt flag, if you forget to do this, the microcontroller will interrupt continuously
}

void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl3) C1Handler(void) {//empty ISR
    mIC1ClearIntFlag();//clear interrupt flag
}


/**
 * Reads temperature from IR sensor of an object in front of sensor
 * @return A hex value from 0x27AD (-70 ?C) to 0x7fff (382.19?C)
 */
int RcvIRTemp(void){
    rcv = 0;
    StartI2C2();                //Send line start condition
    IdleI2C2();                 //Wait to complete
    MasterWriteI2C2(0xb4);      //Write slave addr WRITE (OR 0)
    IdleI2C2();
    MasterWriteI2C2(0x07);      //0x07 = read RAM for TObj1 temperature
    IdleI2C2();             
  
    RestartI2C2();
    IdleI2C2();
    
    MasterWriteI2C2(0xb5);      //Write the slave address, READ (OR 1)
    IdleI2C2(); 
    
    rcv |= MasterReadI2C2();    //Read in LSB
    IdleI2C2();         
    AckI2C2();                  //Acknowledge LSB
    IdleI2C2();
    
    rcv |= MasterReadI2C2()<<8; //Read in MSB       
    AckI2C2();                  //Acknowledge MSB
    IdleI2C2();
    
    MasterReadI2C2();           //Read in PEC            
    AckI2C2();                  //Acknowledge MSB
    IdleI2C2();
   
    StopI2C2();                 //Send line stop condition
    IdleI2C2();                 
    return rcv;                 //Return read value    
}

void SendIMUData (int reg_addr, int data){
	StartI2C2();	        //Send the Start Bit
	IdleI2C2();		//Wait to complete

	MasterWriteI2C2(0x50);  //Sends the slave address over the I2C line.                                               
	IdleI2C2();             //Wait to complete

	MasterWriteI2C2(reg_addr);  //Sends data byte over I2C line
	IdleI2C2();                 //Wait to complete
    
    MasterWriteI2C2(data);
    IdleI2C2();                 //wait for ack
    
	StopI2C2();	        //Send the Stop condition
	IdleI2C2();	        //Wait to complete
} 


int RcvIMUData(unsigned int reg_addr) {
//	//see page 92 of https://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
   
    rcv = 0;
    StartI2C2();                //Send line start condition
    IdleI2C2();                 //Wait to complete
    MasterWriteI2C2(0x50);      //Write slave addr WRITE (OR 0)
    IdleI2C2();
    MasterWriteI2C2(reg_addr);      //BNO055_QUATERNION_DATA_W_LSB_ADDR
    IdleI2C2();             
  
    RestartI2C2();
    IdleI2C2();
    
    MasterWriteI2C2(0x51);      //Write the slave address, READ (OR 1)
    IdleI2C2(); 
    
    rcv |= MasterReadI2C2();    //Read in LSB
    IdleI2C2();       
    AckI2C2();
    IdleI2C2();
    rcv |= MasterReadI2C2()<<8;    //Read in MSB
//    IdleI2C2(); 

    NotAckI2C2();
    IdleI2C2();
    StopI2C2();                 //Send line stop condition
    IdleI2C2();                 
    return rcv;  
    
}


// === Main  ======================================================

void main(void) {
        // === enable system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

//    // init the threads 
    PT_INIT(&pt_uart);

    PT_setup();
    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
   
    OpenI2C2( I2C_EN, BRG_VAL ); 
    I2C2CON |= I2C_7BIT_ADD + I2C_SM_EN;
    I2C2CON |= I2C_RESTART_EN; //restart needed to read from ir temp
    unsigned char cmd = 0; //command line 
    unsigned char data = 0; //output data, digital value to be converted
    //unsigned char addr = 0x1C; 
    //Thermometer sensor: 5A
    
    //===========Configuring the IMU========================//
    unsigned int OPR_CODE = 0x3D;
    SendIMUData(OPR_CODE, 0x0C);        //sets IMU mode
            
    //========================================================//
            
    while(1){
//        temp = RcvIRTemp();
//        cels = temp / 50 - 273; //convert read value to celsius
//        delay_ms(10);
        PT_SCHEDULE(protothread_uart(&pt_uart));
        temp = RcvIMUData(0x20);
        delay_ms(10);

    }


//    // PuTTY
    clrscr();  //clear PuTTY screen
    home();
//    // By default, MPLAB XC32's libraries use UART2 for STDOUT.
//    // This means that formatted output functions such as printf()
//    // will send their output to UART2

//-----------------------------------------------//    
   
   
   
   


    // === configure threads ==========
    // turns OFF UART support and debugger pin, unless defines are set

    // === enable system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

//    // init the threads 
    PT_INIT(&pt_uart);


//    // === set up input capture 
//  // based on timer3 (need to configure timer 3 seperately)
//  OpenCapture1(IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_CAP_32BIT | IC_TIMER2_SRC | IC_ON);
//  // turn on the interrupt so that every capture can be recorded
//  ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3);
//  INTClearFlag(INT_IC1);
//  // connect PIN 24 to IC1 capture unit
//  PPSInput(3, IC1, RPB13);
//  mPORTBSetPinsDigitalIn(BIT_13); //Set port as input
//
//    // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_uart(&pt_uart));
    }
} // main