///
/// @file		HC-SR04.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		2/20/16 10:48 PM
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

#ifndef HC_SR04_cpp
#define HC_SR04_cpp


unsigned long t1;
unsigned long t2;

void unsigned volatile timestamp();

    
class DistanceSensor {
public:
    // the TRIGGER pin sends out the pulse while the ECHO pin reports HIGH if pulse is heard
    DistanceSensor(const unsigned short TRIGGER, const unsigned short ECHO, unsigned long max_distance);
    ~DistanceSensor(){}
    
    //methods
    void calibrate(float cal);
    float read();
    
private:
    float itsMaxDistanceTime = 11764;   //in microseconds = 200cm
    float itsResponseTime;        //microseconds
    float itsDistance;
    
    float speed_of_sound = 344;     //m/s
    float read_length;   
    
    unsigned short ITS_TRIGGER;
    unsigned short ITS_ECHO;
    
};

#endif
