#include "config.h"

#define I2CBAUD 100000 //PIC32 master clock operating at 10kHz (irrelevant if PIC32 is a slave)
#define BRG_VAL  ((PBCLK/2/I2CBAUD)-2)

//---I2C Register Addresses & associated bit masks---

//Config
#define ADDR_CLR_I2C_STATE 0xFF

#ifdef DRIVE

#define ADDR_Config1  0x00 
/* BIT7(MSB) = PID_EN
 * 
 * PID_EN - If enabled, PWM addresses represent target speeds, if
 * disabled, they will be interpreted as duty cycles. Note: in all cases, motor
 * speed/effort is set with a signed 8-bit number
 * 
 * BIT6 = LowPowerShutoff_EN
 * 
 * LowPowerShutoff_EN - MCU will halt all motors if low battery signal recieved
 * 
 * BIT5 = MOTOR_HALT
 * 
 * MOTOR_HALT - MCU will halt all motors
 * 
 * BIT4 = RAMP_DOWN_EN
 * 
 * RAMP_DOWN_EN - MCU will set a maximum speed delta; any speed changes
 * that exceed this delta will be done gradually rather than
 * instantaneously
 * 
 * BIT3 = MCU_TIMEOUT_EN
 * 
 * MCU_TIMEOUT - MCU will set all motor efforts to 0 if no command has been received 
 * for 2 seconds
 * 
 * BIT2 = ENCODER_DISCONECT_BEHAVIOR
 * 
 * ENCODER_DISCONECT_BEHAVIOR - When bit is set: If the encoder for a given 
 * motor is disconnected, * MCU will set  effort level of motor to average 
 * effort level of motors on the same bank. When bit is cleared: all motors on 
 * bank will halt (note: only half of the rover will stop)
 * 
 * BIT1 = 6_WHEEL_EN
 * 
 * 6_WHEEL_EN - when bit is set: 6 wheel drive is activated, otherwise 4 wheel 
 * drive is used
 * 
 * 
 */
#define PID_EN                          0x80
#define LowPowerShutoff_EN              0x40
#define MOTOR_HALT                      0x20
#define RAMP_DOWN_EN                    0x10
#define MCU_TIMEOUT                     0x08
#define ENCODER_DISCONECT_BEHAVIOR      0x04
#define SIX_WHEEL_EN                    0x02



//Read-only
#define ADDR_Status1  0x01
/*
 * BIT7(MSB) = F_ENC_STAT
 * BIT6 = M_ENC_STAT
 * BIT5 = B_ENC_STAT
 * 
 * XX_ENC_STAT - Bit indicates if the encoder for the given motor is
 * connected, 1 for connected, 0 for disconnected
 * 
 * BIT4 = F_STALL
 * BIT3 = M_STALL
 * BIT2 = B_STALL
 * 
 * XX_STALL - Bit indicates if the given motor has stalled, i.e. it is
 * being given a sufficiently high PWM duty cycle, but is still not
 * moving. This may also indicate an improper connection to the motor. 0
 * if stalled, 1 if nominal
 * 
 * BIT1 = BAT_CUTOFF
 * 
 * BAT_CUTOFF - Bit indicates the rover's battery is running low, 1 if
 * nominal, 0 if low

Default state of register = 0xFF
 */
#define F_ENC_STAT 0x80
#define M_ENC_STAT 0x40
#define B_ENC_STAT 0x20
#define F_STALL    0x80
#define M_STALL    0x40
#define B_STALL    0x20


//Motor addresses
#define ADDR_F_PWM 0x10 //NOTE, motor speeds are signed, 2s compliment numbers
#define ADDR_F_P 0x11
#define ADDR_F_I 0x12
#define ADDR_F_D 0x13
#define ADDR_M_PWM 0x14
#define ADDR_M_P 0x15
#define ADDR_M_I 0x16
#define ADDR_M_D 0x17
#define ADDR_B_PWM 0x18
#define ADDR_B_P 0x19
#define ADDR_B_I 0x1A
#define ADDR_B_D 0x1B

//Motor Bank Addresses
#define ADDR_ALL_PWM 0x20 //NOTE, motor speeds are signed, 2s compliment numbers
#define ADDR_ALL_P  0x21
#define ADDR_ALL_I  0x22
#define ADDR_ALL_D  0x23
#define ADDR_HALT  0x2F


//----Read-only addresses----

#define ADDR_F_SPEED 0x30 //NOTE, motor speeds are signed, 2s compliment numbers
#define ADDR_M_SPEED 0x31
#define ADDR_B_SPEED 0x32
#endif

#ifdef MCU1
#define I2CAddress 0x1C
#define LF_ENC_MASK 0x08
#define LM_ENC_MASK 0x10
#define LB_ENC_MASK 0x10
#define STOP_SPEED 2250
#endif

#ifdef MCU2
#define I2CAddress 0x1D
#define RF_ENC_MASK 0x02
#define RM_ENC_MASK 0x04
#define RB_ENC_MASK 0x01
#define ADDR_BAT_LVL 0x33
#define ADDR_HUM_SENSE 0x34
#endif

#ifdef TASK
#define ADDR_Config1  0x00
/* BIT7(MSB) = CUR_Sense_EN
 * 
 * CUR_Sense_EN - If enabled, PWM duty cycles will be set to 0% when current
 * sense threshold is reached. Status1 reg holds information on which thresholds
 * have been tripped. To resume normal motor operation, coresponding Status2 
 * bits must be cleared
 * 
 * BIT6 = LP_SHUTOFF_EN
 * 
 * LowPowerShutoff_EN - If set MCU will halt all motors if low battery signal received
 * 
 * BIT5 = MOTOR_HALT
 * 
 * MOTOR_HALT - If set, MCU will halt all motors 
 * 
 * BIT4 = RAMP_DOWN_EN
 * 
 * RAMP_DOWN_EN - MCU will set a maximum speed delta; any speed changes
 * that exceed this delta will be done gradually rather than
 * instantaneously
 *
 * BIT3 = MCU_TIMEOUT_EN
 * 
 * MCU_TIMEOUT - MCU will set all motor efforts to 0 if no command has been received 
 * for 2 seconds
 * 
 *
 * Default state of config1 = 0x88
 */
#define CUR_SENSE_EN 0x80
#define LP_SHUTOFF_EN 0x40
#define MOTOR_HALT 0x20
#define RAMP_DOWN_EN 0x10
#define MCU_TIMEOUT_EN 0x04


//Read-only
#define ADDR_Status1  0x01
/*
 * BIT7(MSB) = ~BAT_CUTOFF
 * 
 * BAT_CUTOFF - Bit indicates the rover's battery is running low, 1 if
 * nominal, 0 if low

Default state of register = 0xFF
 */
#define BAT_CUTOFF_INV 0x80
#endif

#ifdef MCU3
#define I2CAddress 0x1E
#define ADDR_Status2  0x02
/*
 * BIT7(MSB) = M2_CUR
 * BIT6 = M1_CUR
 * 
 * 
 * XX_CUR - Current sense threshold has been reached by the specified motor
 * motor will halt until bit is reset. 1 if threshold is reached

Default state of register = 0x00
 */
#define M2_CUR 0x80
#define M1_CUR 0x40

//Motor addresses
#define ADDR_M1_PWM 0x10
#define ADDR_M2_PWM 0x11
#define ADDR_SERVO1_PWM 0x12 //pan
#define ADDR_SERVO2_PWM 0x13 //tilt




//Motor Bank Addresses
#define ADDR_ALL_PWM 0x20

//ADC ADDRESSES
#define ADDR_POT1 0x40
#define ADDR_POT2 0x41
#define ADDR_MAG 0x42

//IMU Data Addresses
#define QUATERNION_DATA_W   0x50
#define QUATERNION_DATA_X   0x51
#define QUATERNION_DATA_Y   0x52
#define QUATERNION_DATA_Z   0x53
#endif

#ifdef MCU4
#define I2CAddress 0x1F

#define ADDR_Status2  0x02
/*
 * BIT7(MSB) = M7_CUR
 * BIT6 = M6_CUR
 * BIT5 = M5_CUR
 * BIT4 = M4_CUR
 * BIT3 = M3_CUR
 * 
 * 
 * XX_CUR - Current sense threshold has been reached by the specified motor
 * motor will halt until bit is reset. 1 if threshold is reached

Default state of register = 0x00
 */
#define M7_CUR 0x80
#define M6_CUR 0x40
#define M5_CUR 0x20
#define M4_CUR 0x10
#define M3_CUR 0x08

//Motor addresses
#define ADDR_M3_PWM 0x10
#define ADDR_M4_PWM 0x11
#define ADDR_M5_PWM 0x12
#define ADDR_M6_PWM 0x13
#define ADDR_M7_PWM 0x14



//Motor Bank Addresses
#define ADDR_ALL_PWM 0x20
#endif

// ============= the fixed point macros ========================================
typedef signed int fix16;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b))))
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a))))
#define absfix16(a) abs(a)



