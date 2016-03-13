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

//#ifdef __TEENSY__
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
#   include "CurieIMU.h"
#else // error
#   error Platform not defined
#endif // end IDE

#include "../../../LinearControllers.h"
#include "../../../Filters.h"
#include "../../../Vector.h"
#include "../../../Flight.h"
#include "../../../Drivers/MPU6050.h"

#include "../../../Drivers/HC-SR04.h"
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>


//-----defines-----//
#define ECHO
//#define oneshot125

#define sampleFreq 470.0f		// sample frequency in Hz
#define SAMPLE_RATE_DIV (1000/(sampleFreq)) - 1   // to set our gyro sample rate
#define sampleFreqSonar 5.0f   // sample frequency of sonar in hz
#define delayTime ((1/sampleFreq)*1000000.0f)		// sample frequency in Hz

#define signed_32bits 2147483648
#define signed_16bits 32767

//---- prototypes -------//
void initializeSystem();
void processIO();

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);

//----------- FSM model -------------------//

//-----IMU------//
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

MPU6050 mpu;

Vector3<int16_t> acc_raw;
Vector3<int16_t> gyro_raw;

Vector3<float> acc_raw_float;
Vector3<float> gyro_raw_float;
Vector3<float> gyro_filtered;
Vector3<float> acc_filtered;
Vector3<float> Attitude;
Vector3<float> AttitudeFiltered;

Filter GyroLPF(1/sampleFreq, 256.0);
Filter AccLPF(1/sampleFreq, 256.0);

bool IMU_online = false;


//----Timer-----//
// variables to figure out cycle rate
unsigned long begin_of_loop = 0;
unsigned long loop_time = 0;    //microseconds
float dt = (float)loop_time/1000000;   //seconds
float measured_cycle_rate = sampleFreq;
int delay_time = 0;

//------Motor pins--------//
const int motor_LED_test[4] = {3,4,5,6};
Servo motor[4];
short mapped_signal[4];
int cal_pot = 0;

// ----- Quad -------//
float *signals[4];
QuadCopter alpha(&dt, signals);

//----- Sonar -------//
DistanceSensor AltitudeSonar(10,9,200);
//NewPing sonar(10,9,200);
Vector3<float> Position;
unsigned long sonarTimer = 0;

//------ altimeter -------//
//Barometer altimeter;
//MPL3115A2 altimeter;
//NewPing sonar(10,9);


#endif
//#endif








