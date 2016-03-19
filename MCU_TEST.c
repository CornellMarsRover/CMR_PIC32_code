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
static int value2;
static unsigned int rcv; 

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


/***************************************************
 * SendData(int data, unsigned int address)        *
 *						    *
 * Sends a byte of data (DATA) over the I2C line   *
 *	to I2C address ADDRESS			    *
 *						    *
 * Returns: nothing				    *
 ***************************************************/
void SendData (int data, unsigned int address){
	StartI2C1();	        //Send the Start Bit
	IdleI2C1();		//Wait to complete

	MasterWriteI2C1((address << 1));  //Sends the slave address over the I2C line.  This must happen first so the 
                                             //proper slave is selected to receive data.
	IdleI2C1();	        //Wait to complete

	MasterWriteI2C1(data);  //Sends data byte over I2C line
	IdleI2C1();		//Wait to complete
	StopI2C1();	        //Send the Stop condition
	IdleI2C1();	        //Wait to complete

} //end function

/***************************************************
 * SendData(int data, unsigned int address)        *
 *						    *
 * Sends a byte of data (DATA) over the I2C line   *
 *	to I2C address ADDRESS			    *
 *						    *
 * Returns: nothing				    *
 ***************************************************/
void SendData2 (int data1, int data2, unsigned int address){
	StartI2C1();	        //Send the Start Bit
	IdleI2C1();		//Wait to complete

	MasterWriteI2C1((address << 1));  //Sends the slave address over the I2C line.  This must happen first so the 
                                             //proper slave is selected to receive data.
	IdleI2C1();	        //Wait to complete

	MasterWriteI2C1(data1);  //Sends data byte over I2C line
	IdleI2C1();		//Wait to complete
    MasterWriteI2C1(data2);  //Sends data byte over I2C line
	IdleI2C1();		//Wait to complete
	StopI2C1();	        //Send the Stop condition
	IdleI2C1();	        //Wait to complete

} //end function
static int slaveAdd = 0x1D;

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
        sscanf(PT_term_buffer, "%x %x", &value2, &value);
//                         
        
//        // echo
        sprintf(PT_send_buffer,"%x%s", value, "\n\r");//send original message
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\n");//next line
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
//        sprintf(PT_send_buffer,"\r");//carriage return
//        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
        int temp;
        if(value){
            SendData2(value2, value, slaveAdd);     
            sprintf(PT_send_buffer,"sent data\n\r");//send original message
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
        }
        else if(value2){
            SendData(value2, slaveAdd);
            sprintf(PT_send_buffer,"sent command\n\r");//send original message
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
        }
        else{
            temp = RcvData2(slaveAdd);
            sprintf(PT_send_buffer, "read data %d\n\r", temp );
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );            
        }
        
        
        
       
        PT_YIELD_TIME_msec(50);
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





/*****************************************************
 * RcvData(unsigned int address)		     *
 *					  	     *
 * Gets a byte of data from I2C slave device at      *
 *  ADDRESS.					     *
 *						     *
 * Returns: Received data			     *
 ****************************************************/
int RcvData(unsigned int address) {
	StartI2C1();				//Send line start condition
	IdleI2C1();			        //Wait to complete
	MasterWriteI2C1((address << 1) | 1);	//Write out slave address OR 1 (read command)
	IdleI2C1();				//Wait to complete
	rcv = MasterReadI2C1();		//Read in a value   
    StopI2C1();				//Send line stop condition
	IdleI2C1();				//Wait co complete
	return rcv;				//Return read value
}
int RcvData2(unsigned int address){
    rcv = 0;
    StartI2C1();				//Send line start condition
	IdleI2C1();			        //Wait to complete
	MasterWriteI2C1((address << 1) | 1);	//Write out slave address OR 1 (read command)
	IdleI2C1();				//Wait to complete
	rcv = MasterReadI2C1()<<8;		//Read in a value   
    AckI2C1();
    IdleI2C1();
    rcv |= MasterReadI2C1();    
	StopI2C1();				//Send line stop condition
	IdleI2C1();				//Wait co complete
	return rcv;				//Return read value    
}


// === Main  ======================================================

void main(void) {
    
    PT_setup();
    ANSELA = 0; //make sure analog is cleared
    ANSELB = 0;
   
    OpenI2C1( I2C_EN, BRG_VAL ); 
    unsigned char cmd = 0; //command line 
    unsigned char data = 0; //output data, digital value to be converted
    unsigned char addr = 0x1C; 
    
    
//    SendData(0x75, addr);
//    RcvData2(addr);
    
//    while(1);
//    delay_ms(5000);
//        while(1) // run the code over and over again
//    { 
//
//        // start the I2C communication
//        StartI2C1(); // Send the Start Bit (begin of data send) 
//        IdleI2C1(); // Wait to complete 
//
//        // write the address of the chip, defined by pins AD0 and AD1 on the MAX518
//        MasterWriteI2C1 (addr); // address 
//        IdleI2C1(); 
//        
//        
//        while( !I2C1STATbits.ACKSTAT==0 ){}
//        
//        // write the command to tell the MAX518 to change its output on output 0
//        MasterWriteI2C1 (cmd); // command line 
//        IdleI2C1(); 
//        while( !I2C1STATbits.ACKSTAT==0 ){}
//
//        // wite the value to put on the output
//        MasterWriteI2C1(data); // output 
//        IdleI2C1(); 
//        while( !I2C1STATbits.ACKSTAT==0 ){}
//
//        // end the I2C communication
//        StopI2C1(); // end of data send 
//        IdleI2C1(); // Wait to complete 
//
//        // the total write time is ~285us with an I2C clock of 100 kHz
//
//        data++; // increase the data, make a sawtooth wave as an example
//
//        //if(data>255) { // don't have to worry about this, data is 8 bit so it will roll over automatically
//        //    data=0;
//        //}
//
//    } // end while(1)
//    
    
    
    
//------- uncomment to init the uart2 -----------//
    //UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    //UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    //UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
    //UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    //ConfigIntUART2(UART_RX_INT_EN | UART_TX_INT_EN | UART_ERR_INT_EN | UART_INT_PR0 | UART_INT_SUB_PR0);

    // rxchar = 0 ; // a received character
    // count = 0 ; // count the number of characters

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

// === end  ======================================================
