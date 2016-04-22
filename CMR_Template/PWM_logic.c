#include "PWM_logic.h"

inline int getPWM(char I2CDataIn) {

    if ((I2CDataIn & 0x80)) {
        return (((((~I2CDataIn) + 1) << 1) > 150) ? 150 : ((~I2CDataIn) + 1) << 1); //bitwise complement
    }
    else {
        return (-1 * (I2CDataIn << 1)) < -150 ? (-150) : (-1 * (I2CDataIn << 1));

    }
}

inline int getEffort(char I2CDataIn) {
    if ((I2CDataIn & 0x80)) {
        return ((2000 + (((int) ((~I2CDataIn) + 1)) << 4)) > 3999 ? (3999) : (2000 + (((int) ((~I2CDataIn) + 1)) << 4)));
    } else {
        return (2000 - (((int) I2CDataIn) << 4)) < 1 ? (1) : (2000 - (((int) I2CDataIn) << 4));
    }



}
