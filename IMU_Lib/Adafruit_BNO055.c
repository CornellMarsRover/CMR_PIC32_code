/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

 * 
  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/
//
//#if ARDUINO >= 100
 //#include "Arduino.h"
//#else
// #include "WProgram.h"
//#endif

//#include "math.h"
//#include "limits.h"
#include "config.h"
#include "pt_cornell_1_2.h"
////
#include "tft_gfx.h"
#include "tft_master.h"

#include "Adafruit_BNO055.h"
#include "stdint.h"

char buffer[60]; // string buffer

int32_t id;
uint8_t _address;
int32_t _sensorID;
uint8_t i = 0;
static unsigned int rcv; 

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BNO055 class
*/
/**************************************************************************/
void Adafruit_BNO055(int32_t sensorID, uint8_t address)
{
  _sensorID = sensorID;
  _address = address;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
uint8_t begin(adafruit_bno055_opmode_t mode)
{
  /* Enable I2C */
  //Wire.begin();
  StartI2C2();
  IdleI2C2();

  /* Make sure we have the right device */
  id = read8(BNO055_CHIP_ID_ADDR);
  if(id != BNO055_ID)
  {
    //delay(1000); // hold on for boot
    id = read8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID) {
      return 0;  // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    //delay(10);
  }
  //delay(50);

  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  //delay(10);

  write8(BNO055_PAGE_ID_ADDR, 0); 
  
  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  //delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(mode);
  //delay(20);

  return 1;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
void setMode(adafruit_bno055_opmode_t mode)
{
  _mode = mode;
  write8(BNO055_OPR_MODE_ADDR, _mode);
  //delay(30);
}

/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void setExtCrystalUse(int usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  //delay(25);
  write8(BNO055_PAGE_ID_ADDR, 0);
  if (usextal == 1) {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  //delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  //delay(20);
}


/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error     = read8(BNO055_SYS_ERR_ADDR);

  //delay(200);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
  if (sys != 0) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != 0) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != 0) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != 0) {
    *mag = calData & 0x03;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
int8_t getTemp(void)
{
  int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
  return temp;
}

/**************************************************************************/
/*!
@brief  Writes an array of calibration values to the sensor's offset registers
*/
/**************************************************************************/
void setSensorOffsets(const uint8_t* calibData)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    //delay(25);

    /* A writeLen() would make this much cleaner */
    write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
    write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
    write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
    write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
    write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
    write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

    write8(MAG_OFFSET_X_LSB_ADDR, calibData[12]);
    write8(MAG_OFFSET_X_MSB_ADDR, calibData[13]);
    write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
    write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
    write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
    write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

    write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
    write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

    setMode(lastMode);
}

int isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return 0;
    return 1;
}

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

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
int readLen(adafruit_bno055_reg_t reg, uint8_t len)
{

  while( i < len)
  {
      buffer[i] = RcvData(BNO055_QUATERNION_DATA_W_LSB_ADDR + i);
      i = i + 1;
  }
  i = 0;
  /* ToDo: Check for errors! */
  return 1;
}

void main(void) {}
