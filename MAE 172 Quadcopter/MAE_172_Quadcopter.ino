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
//#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
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
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
#   include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE


//---- Defines for which board being used -----------//

#define __TEENSY__
//#define __ARDUINO101__

// Include application, user and local libraries
#ifdef __TEENSY__
    #include "/src/HardwareSpecific/Arduino/Teensy/gpio.h"
#elif __ARDUINO101__
    #include "/src/HardwareSpecific/Arduino/Arduino101/gpio.h"
#endif



// Prototypes


// Define variables and constants


// Add setup code
void setup()
{
    ;
}

// Add loop code
void loop()
{
    ;
}
