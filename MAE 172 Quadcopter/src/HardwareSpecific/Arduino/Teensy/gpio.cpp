//
// gpio.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 1/22/16 5:08 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			gpio.h and ReadMe.txt for references
//

#ifdef __TEENSY__

// Library header
#include "gpio.h"

// Code

void initializeSystem() {
    Serial.begin(115200);
    Wire.begin();
    
    pinMode(motor_LED_test[0],OUTPUT);
    pinMode(motor_LED_test[1],OUTPUT);
    pinMode(motor_LED_test[2],OUTPUT);
    pinMode(motor_LED_test[3],OUTPUT);
    mpu.initialize();
}

void processIO() {
    Serial.println(mpu.testConnection());
    Serial.println("test");
    
    analogWrite(motor_LED_test[0], 255);
    analogWrite(motor_LED_test[1], 255);
    analogWrite(motor_LED_test[2], 255);
    analogWrite(motor_LED_test[3], 255);
}

#endif