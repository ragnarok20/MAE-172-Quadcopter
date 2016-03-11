///
/// @file		Filters.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		3/10/16 12:17 PM
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

#ifndef Filters_cpp
#define Filters_cpp

#include "Vector.h"

// first order low pass filter
class Filter{
public:
    //constructors
    Filter(float samplePeriod, float cutoffFreq){ T = samplePeriod; w = cutoffFreq; timeConst = 1/w; a = T/(timeConst + T); b = timeConst/(T + timeConst);};
    ~Filter(){};
    
    //methods
    float update(float sample);
    Vector3<float> update(Vector3<float> sample);
private:
    float w;    //cutoff
    float T;    //sample period
    float timeConst;
    
    float a,b;  // constants in recursive series
    
    float prevSample = 0;
    float currSample = 0;
    float filteredSignal = 0;
    
    Vector3<float> prevSampleVec;
    Vector3<float> currSampleVec;
    Vector3<float> filteredSignalVec;
    
};


#endif
