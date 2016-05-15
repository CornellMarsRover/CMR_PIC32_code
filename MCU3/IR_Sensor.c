#include "IR_Sensor.h"
#include <plib.h>
unsigned short RcvIRTemp(){
    unsigned short rcv = 0;
    StartI2C2();				//Send line start condition
	IdleI2C2();			        //Wait to complete
	MasterWriteI2C2(IRADD<<1);	//Write slave addr WRITE (OR 0)
    IdleI2C2();
    MasterWriteI2C2(IR_READ_COMMAND); //command
    //Command to read Tobj1 on ir sensor's ram: 000 = read ram; 0x07 = tobj1
	IdleI2C2();				//Wait to complete
    
    RestartI2C2();
    IdleI2C2();
    MasterWriteI2C2(IRADD<<1 | 0x01);//read command
    
    rcv = MasterReadI2C2();		//Read in LSB
    IdleI2C2();	        
    AckI2C2();              //Acknowledge LSB
    IdleI2C2();
    
    rcv |= MasterReadI2C2()<<8;		//Read in MSB
    IdleI2C2();	        
    AckI2C2();              //Acknowledge MSB
    IdleI2C2();
    
    MasterReadI2C2();		//Read in PEC
    IdleI2C2();             
    AckI2C2();              //Acknowledge MSB
    IdleI2C2();
   
	StopI2C2();				//Send line stop condition
	IdleI2C2();				//Wait co complete
	return rcv;				//Return read value    
}