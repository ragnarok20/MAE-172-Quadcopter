///
/// @file		LinearControllers.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/20/16 11:42 AM
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

#ifndef LinearControllers_cpp
#define LinearControllers_cpp

//------------------------------------------------------------------------------//
//-----------------------General PID controller object--------------------------//
//-----input desired parameters and actual measurement to the parameter---------//
//------returns a PWM 8 bit value to be fed to most likely a motor--------------//
//------------------------------------------------------------------------------//

class PIDController {
public:
    //constructors and destructor
    PIDController(float desPos, float desVel, float actualPos, float actualVel);
    ~PIDController(){};
    
    //methods
    void getControlSignal(int *controlSignal);
private:
    int itsControlSignal;
    float itsKp, itsKd, itsKi;   // controller gains:: Kp: position, Kd: derivative, Ki: integral
    int itsDesPos, itsActualPos, itsDesVel, itsActualVel;
    int itsPosError, itsVelError, itsIntegralError;
    double currentTime, previousTime, differentialTime;
    
};






#endif
