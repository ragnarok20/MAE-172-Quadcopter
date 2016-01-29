///
/// @file		Flight.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/22/16 5:17 PM
/// @version	<#version#>
/// 
/// @copyright	(c) Sage Thayer, 2016
/// @copyright	<#license#>
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
#include "ArduinoAll.h"
#include "LinearControllers.h"
#include "src/HardwareSpecific/Arduino/Teensy/gpio.h"

#ifndef Flight_cpp
#define Flight_cpp

float yawGains[3];
float pitchGains[3];
float rollGains[3];
float altitudeGains[3];

int desiredYaw = 0;
int desiredPitch = 0;
int desiredRoll = 0;
int desiredAltitude = 0;

PIDController Yaw(dt, yawGains[0], yawGains[1], yawGains[2], desiredYaw);
PIDController Pitch(dt, pitchGains[0], pitchGains[1], pitchGains[2], desiredPitch);
PIDController Roll(dt, rollGains[0], rollGains[1], rollGains[2], desiredRoll);
PIDController Altitude(dt, altitudeGains[0], altitudeGains[1], altitudeGains[2], desiredAltitude);

//prototypes
void steadyLevelFlight();
void ascend(int altitude);
void descend(int altitude);
void land();


#endif
