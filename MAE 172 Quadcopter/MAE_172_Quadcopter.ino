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
#include "ArduinoAll.h"


//---- Defines for which board being used -----------//

#define __TEENSY__
//#define __ARDUINO101__

// Include application, user and local libraries
#ifdef __TEENSY__
    #include "src/HardwareSpecific/Arduino/Teensy/gpio.h"
#elif __ARDUINO101__
    #include "src/HardwareSpecific/Arduino/Arduino101/gpio.h"
#endif



// Prototypes


// Define variables and constants


// Add setup code
void setup()
{
    initialzSystem();
    ;
}

// Add loop code
void loop()
{
    processIO()
    ;
}
