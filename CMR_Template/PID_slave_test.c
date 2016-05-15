/*
 * File:        main
 * Author:      Matthew Filipek
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
#include "pt_cornell_1_2.h"

////////////////////////////////////
// graphics libraries
//#include "tft_master_spi2.c"
#include "tft_gfx.h"
#include "tft_master.h"
#define I2CBAUD 10000
#define BRG_VAL  ((PBCLK/2/I2CBAUD)-2)

static char buffer[100]; // string buffer
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
static unsigned char I2CAddress;
static unsigned char I2CDataIn;
static unsigned char isMaster = 0;
static unsigned char byteReceved; 

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
static struct pt pt_uart, pt_pid, pt_anim, pt_input, pt_output, pt_DMA_output;

static PT_THREAD(protothread_uart(struct pt *pt)) {
    // this thread interacts with the PC keyboard to take user input and set up PID parameters
    PT_BEGIN(pt);
    while (1) {
        // send the prompt via DMA to serial
        sprintf(PT_send_buffer, "%s", "cmd>");
        // by spawning a print thread
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));//send date and time

//        while(!byteReceved);
//        sprintf(PT_send_buffer,"%s", "byte received!");
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\n");//next line
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\r");//carriage return
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );                
//        
//        byteReceved = 0;
        //spawn a thread to handle terminal input
        // the input thread waits for input
        // -- BUT does NOT block other threads
        // string is returned in "PT_term_buffer"
        PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));//wait for  input
        sscanf(PT_term_buffer, "%s %f", cmd, &value);
                         
        // echo
        sprintf(PT_send_buffer,"%s%02x%s", "I2C Data:\n", I2CDataIn, "\n");//send original message
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\n");//next line
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\r");//carriage return
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        
//        sprintf(PT_send_buffer,"%d", I2CDataIn);//send original message
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
        
       
        PT_YIELD_TIME_msec(30);
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

void InitI2C(void)
{
 unsigned char temp;
// I2CEnable(I2C1, 0);	//Disable I2C before changing its settings 
// I2CConfigure(I2C1, I2C_ENABLE_SLAVE_CLOCK_STRETCHING);
// I2CSetSlaveAddress(I2C2, 0x40, 0xFF, I2C_USE_7BIT_ADDRESS | I2C_ENABLE_GENERAL_CALL_ADDRESS);	//Sets the slave address for the I2C communication 
// 
// I2C1BRG = BRG_VAL;
 
 // Enable the I2C module with clock stretching enabled
 // define a Master BRG of 400kHz although this is not used by the slave
 OpenI2C1(I2C_ON | I2C_7BIT_ADD| I2C_STR_EN , BRG_VAL );//48);
 
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


// === Main  ======================================================

void main(void) {
    PT_setup();
    
    
    
    // === I2C Init ================
    I2CAddress = 0x1C;
    InitI2C();
    
    mPORTBSetPinsDigitalOut(BIT_0);
    mPORTBClearBits(BIT_0);

    
    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
    
    
    
    
    mJTAGPortEnable(0);
 

     
    
    INTEnableSystemMultiVectoredInt();
    
     
    
    
//------- uncomment to init the uart2 -----------//
    //UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    //UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    //UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
    //UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    //ConfigIntUART2(UART_RX_INT_EN | UART_TX_INT_EN | UART_ERR_INT_EN | UART_INT_PR0 | UART_INT_SUB_PR0);

    // rxchar = 0 ; // a received character
    // count = 0 ; // count the number of characters

//    // PuTTY
//    clrscr();  //clear PuTTY screen
//    home();
//    // By default, MPLAB XC32's libraries use UART2 for STDOUT.
//    // This means that formatted output functions such as printf()
//    // will send their output to UART2

//-----------------------------------------------//    
   
   
   
   


    // === configure threads ==========
    // turns OFF UART support and debugger pin, unless defines are set

    // === enable system wide interrupts  ========

//    // init the threads 
    PT_INIT(&pt_uart);
//    PT_INIT(&pt_anim);
   
// ---------- uncomment to init the tft display -----------//
//    tft_init_hw();
//    tft_begin();
//    tft_fillScreen(ILI9340_BLACK); //240x320 vertical display
//    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
//---------------------------------------------------------//
    

//    // === set up input capture 
//	// based on timer3 (need to configure timer 3 seperately)
//	OpenCapture1(IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_CAP_32BIT | IC_TIMER2_SRC | IC_ON);
//	// turn on the interrupt so that every capture can be recorded
//	ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3);
//	INTClearFlag(INT_IC1);
//	// connect PIN 24 to IC1 capture unit
//	PPSInput(3, IC1, RPB13);
//	mPORTBSetPinsDigitalIn(BIT_13); //Set port as input
//
//    // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_uart(&pt_uart));
        //PT_SCHEDULE(protothread_anim(&pt_anim));
    }
} // main

/*
// build a string from the UART2 /////////////
// feed in buffer size (should be 60)
int GetDataBuffer(char *buffer, int max_size)
{
    int num_char;
    num_char = 0;
    while(num_char < max_size)
    {
        char character;
        while(!UARTReceivedDataIsAvailable(UART2)){};
        character = UARTGetDataByte(UART2);
        UARTSendDataByte(UART2, character);
        if(character == '\r'){
            *buffer = 0;
            crlf;
            break;
        }
        *buffer = character;
        buffer++;
        num_char++;
    }
    return num_char;
} */

///////////////////////////////////////////////////////////////////
//
// Slave I2C interrupt handler
// This handler is called when a qualifying I2C events occurs
// this means that as well as Slave events 
// Master and Bus Collision events will also trigger this handler.
//
///////////////////////////////////////////////////////////////////
void __ISR(_I2C_1_VECTOR, ipl3) _SlaveI2CHandler(void)
{
 unsigned char temp;
 static unsigned int dIndex;

 // check for MASTER and Bus events and respond accordingly
 if ( IFS1bits.I2C1MIF ){
  mI2C1MClearIntFlag();
  return;  
 }
 if ( IFS1bits.I2C1BIF ){
  mI2C1BClearIntFlag();
  return;
 }
 
 // handle the incoming message
 if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0)) {
  // reset any state variables needed by a message sequence 
  // perform a dummy read
  temp = SlaveReadI2C1();
  I2C1CONbits.SCLREL = 1; // release the clock
 } else if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1)) {//data received, input to slave
  // writing data to our module, just store it in adcSample
  mPORTBSetBits(BIT_0);
  I2CDataIn = SlaveReadI2C1();
  if(I2CDataIn == 0x55){
    mPORTAToggleBits(BIT_0);
  }
  byteReceved = 1;
  I2C1CONbits.SCLREL = 1; // release clock stretch bit
 } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0)) {//request for output
  // read of the slave device, read the address 
  temp = SlaveReadI2C1();
  dIndex = 0;
  SlaveWriteI2C1(0xF6);
 } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 1)) {//data received  
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


// === end  ======================================================
