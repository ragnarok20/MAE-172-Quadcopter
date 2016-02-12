///
/// @file		gpio.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/22/16 5:08 PM
/// @version	<#version#>
/// 
/// @copyright	(c) Sage Thayer, 2016
/// @copyright	<#license#>
///
/// @see		ReadMe.txt for references
///

#ifndef __gpio_h__
#define __gpio_h__

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

#include "../../../LinearControllers.h"
#include "../../../Vector.h"
#include "../../../Flight.h"
#include "../../../Drivers/MPU6050.h"
#include <Wire.h>


//-----defines-----//
#define ECHO
#define sampleFreq 100.0f		// sample frequency in Hz

//---- prototypes -------//
void initializeSystem();
void processIO();

//-----IMU------//
MPU6050 mpu;

Vector3<int16_t> acc_raw;
Vector3<int16_t> gyro_raw;
Vector3<float> attitude;

bool IMU_online = false;

//----Timer-----//
// variables to figure out cycle rate
double begin_of_loop = 0;
double loop_time = 0;
double measured_cycle_rate = sampleFreq;
int delay_time = 0;

//------Motor pins--------//
const int motor_LED_test[4] = {5,4,3,6};
int mapped_signal[4];

// ----- Quad -------//
float* signals[4];
QuadCopter alpha(signals);


#endif










