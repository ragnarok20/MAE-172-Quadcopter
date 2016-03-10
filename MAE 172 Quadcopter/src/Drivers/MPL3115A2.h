///
/// @file		MPL3115A2.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		3/9/16 12:47 PM
/// @version	<#version#>
/// 
/// @copyright	(c) Sage Thayer, 2016
/// @copyright	<#license#>
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#   include "libpandora_types.h"
#   include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#   include "Arduino.h"
#elif defined(SPARK) // Spark specific
#   include "application.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE

#ifndef MPL3115A2_cpp
#define MPL3115A2_cpp

/* ------------- MPL3115A2 Barometer begin --------------------- */

#define BAR_STATUS     0x00
#define BAR_OUT_P_MSB  0x01
#define BAR_OUT_P_CSB  0x02
#define BAR_OUT_P_LSB  0x03
#define BAR_OUT_T_MSB  0x04
#define BAR_OUT_T_LSB  0x05
#define BAR_DR_STATUS  0x06
#define BAR_OUT_P_DELTA_MSB  0x07
#define BAR_OUT_P_DELTA_CSB  0x08
#define BAR_OUT_P_DELTA_LSB  0x09
#define BAR_OUT_T_DELTA_MSB  0x0A
#define BAR_OUT_T_DELTA_LSB  0x0B
#define BAR_WHO_AM_I   0x0C
#define BAR_F_STATUS   0x0D
#define BAR_F_DATA     0x0E
#define BAR_F_SETUP    0x0F
#define BAR_TIME_DLY   0x10
#define BAR_SYSMOD     0x11
#define BAR_INT_SOURCE 0x12
#define BAR_PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define BAR_P_TGT_MSB  0x16
#define BAR_P_TGT_LSB  0x17
#define BAR_T_TGT      0x18
#define BAR_P_WND_MSB  0x19
#define BAR_P_WND_LSB  0x1A
#define BAR_T_WND      0x1B
#define BAR_P_MIN_MSB  0x1C
#define BAR_P_MIN_CSB  0x1D
#define BAR_P_MIN_LSB  0x1E
#define BAR_T_MIN_MSB  0x1F
#define BAR_T_MIN_LSB  0x20
#define BAR_P_MAX_MSB  0x21
#define BAR_P_MAX_CSB  0x22
#define BAR_P_MAX_LSB  0x23
#define BAR_T_MAX_MSB  0x24
#define BAR_T_MAX_LSB  0x25
#define BAR_CTRL_REG1  0x26
#define BAR_CTRL_REG2  0x27
#define BAR_CTRL_REG3  0x28
#define BAR_CTRL_REG4  0x29
#define BAR_CTRL_REG5  0x2A
#define BAR_OFF_P      0x2B
#define BAR_OFF_T      0x2C
#define BAR_OFF_H      0x2D

#define MPL3115A2_ADDRESS 0x60 // 7-bit I2C address

#include <Wire.h>

uint8_t I2C_Read(uint8_t slaveAddr, uint8_t regAddr);
void I2C_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t value);

class Barometer {
public:
    //constructor
    Barometer();    // default constructor will put device into altimeter mode and polling mode
    Barometer(bool alt_mode);
    ~Barometer(){};
    
    //methods
    void calibrate(float known_param);
    float read();
    
private:
    bool its_ALT_MODE;  // if true, then in altimeter mode; false then in barometer mode reading pressure
    bool data_read;
    float itsAltitude;
    
};

/* ------------- MPL3115A2 Barometer end --------------------- */



#endif
