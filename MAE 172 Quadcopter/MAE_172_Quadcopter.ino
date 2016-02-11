//
// MAE 172 Quadcopter
//
// Autonomous Quadcopter
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Sage Thayer
// 				Sage Thayer
//
// Date			1/20/16 11:41 AM
// Version		<#version#>
//
// Copyright	© Sage Thayer, 2016
// Licence		<#licence#>
//
// See         ReadMe.txt for references
//


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
#elif defined(CurieIMU) // Arduino 101 Gyro
include "CurieIMU.h"
#else // error
#   error Platform not defined
#endif // end IDE


//---- Defines for which board being used -----------//

#define __TEENSY__
//#define __ARDUINO101__

// Include application, user and local libraries
#ifdef __TEENSY__
    #include "src/HardwareSpecific/Arduino/Teensy/gpio.h"
    #include "src/HardwareSpecific/Arduino/Teensy/gpio.cpp"
#elif __ARDUINO101__
    #include "src/HardwareSpecific/Arduino/Arduino101/gpio.h"
#endif

#include "src/Flight.h"
#include "src/Drivers/MadgwickAHRS.h"

// Prototypes
void initializeSystem();
void processIO();

QuadCopter test;



// Define variables and constants


// Add setup code
void setup()
{
    initializeSystem();
    
}

// Add loop code
void loop()
{
    processIO();
    test.steadyLevelFlight();
    
}
