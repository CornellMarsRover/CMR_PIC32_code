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
#include <plib.h>

////////////////////////////////////
// graphics libraries
//#include "tft_master_spi2.c"
#include "tft_gfx.h"
#include "tft_master.h"

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
static int count;
static int a;


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

#define FOSC 60E6
#define PB_DIV 8
#define PRESCALE 256
#define T1_TICK (FOSC/PB_DIV/PRESCALE/4)

#define CONFIG (CN_ON | CN_IDLE_CON)
#define PINS (CN15_ENABLE)
#define PULLUPS (CN_PULLUP_DISABLE_ALL)

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

        //spawn a thread to handle terminal input
        // the input thread waits for input
        // -- BUT does NOT block other threads
        // string is returned in "PT_term_buffer"
        PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));//wait for  input
        sscanf(PT_term_buffer, "%s %f", cmd, &value);
                         
        // echo
        sprintf(PT_send_buffer,PT_term_buffer);//send original message
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
        sprintf(PT_send_buffer,"\n");//next line
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
        sprintf(PT_send_buffer,"\r");//carriage return
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                
        
       
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

// external interrupt

void __ISR(_EXTERNAL_0_VECTOR, ipl1) INT0Interrupt(void){
	//mLED_3_On();//_EXTERNAL_0_Vector
     tft_writeString("CMR");
  
     count++;
     //printf("External: number of count: %d", count);
     // clear interrupt flag
     mINT0ClearIntFlag();
}

void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void) { //empty ISR
	// Timer interupt to read data
    //sprintf(PT_send_buffer, "%s", "cmd>");
    mPORTAToggleBits(BIT_0);
    printf("Timer: number of count: %d", count);
    count = 0;
    mT1ClearIntFlag();//clear interrupt flag, if you forget to do this, the microcontroller will interrupt continuously
}

void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl3) C1Handler(void) {//empty ISR
	mIC1ClearIntFlag();//clear interrupt flag
}

// === Main  ======================================================

void main(void) {
    PT_setup();
    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
    a = 0;
    
    // LED on
    mPORTAClearBits(BIT_0);
    mPORTASetPinsDigitalOut(BIT_0);
    mINT0IntEnable(TRUE);
    mINT0SetIntPriority(INT_PRIORITY_LEVEL_1);
    //mPORTBSetPinsDigitalIn(BIT_7);

    //RPA0R = 1;
    // RPA0R Interrupt
    //INT4R = 0x0000;
    
    // STEP 1. configure the wait states and peripheral bus clock
    //SYSTEMConfigWaitStatesAndPB(72000000L);
    // STEP 2. configure the port registers
    //ORTSetPinsDigitalOut(IOPORT_A, BIT_0);
    // STEP 3. initialize the port pin states = outputs low
    //mPORTAClearBits(BIT_0);
    // STEP 4. setup the change notice options
    //mCNOpen(CONFIG, PINS, PULLUPS);
    // STEP 5. read port(s) to clear mismatch on change notice pins
    //value = mPORTDRead();
    // STEP 6. clear change notice interrupt flag
    //ConfigIntCN(INTERRUPT);
      
    // STEP 1. configure the Timer1
    //ConfigInt0(EXT_INT_PRI_3|(FALLING_EDGE_INT || RISING_EDGE_INT)|EXT_INT_ENABLE);
    //ConfigINT0(EXT_INT_PRI_3|(FALLING_EDGE_INT || RISING_EDGE_INT)|EXT_INT_ENABLE);
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // STEP 2. set the timer interrupt to prioirty level 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // STEP 3. enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();
    
    // external interrupt0
    /*
    IEC0CLR = 0x00000008;// disable INT0
    INTCONCLR = 0x00000008;// clear the bit for falling edge trigger
    IFS0CLR = 0x00000008;// clear the interrupt flag
    IEC0SET = 0x00000008;// enable INT0
    */
    // set up LED
    //PORTSetPinsDigitalOut(IOPORT_A, BIT_0);
    
    //ConfigInt0(EXT_INT_PRI_3|(FALLING_EDGE_INT || RISING_EDGE_INT)|EXT_INT_ENABLE);
   
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

//    // init the threads 
    PT_INIT(&pt_uart);
//    PT_INIT(&pt_anim);
   
// ---------- uncomment to init the tft display -----------//
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK); //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    tft_setCursor(0, 0);
	tft_setTextColor(ILI9340_WHITE);  
    tft_setTextSize(4);// 
    //tft_writeString("CMR");
    while(1);
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
        //RPA0R = 1;
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

// === end  ======================================================
