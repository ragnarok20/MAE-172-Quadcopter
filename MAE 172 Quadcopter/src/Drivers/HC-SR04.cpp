//
// HC-SR04.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 2/20/16 10:48 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			HC-SR04.h and ReadMe.txt for references
//


// Library header
#include "HC-SR04.h"

// Code


// ------ proximity sensor ------------//

DistanceSensor::DistanceSensor(const unsigned short TRIGGER, const unsigned short ECHO, unsigned long max_distance) {
    //set up correct pin modes
    pinMode(TRIGGER,OUTPUT);
    pinMode(ECHO,INPUT);
    
    ITS_TRIGGER = TRIGGER;
    ITS_ECHO = ECHO;
    
    //set object values
    itsMaxDistanceTime = (float)(max_distance/speed_of_sound) * 10000 ;      //microseconds
    
}

float DistanceSensor::read() {
    // ------ send out a pulse from trigger pin, time how long it takes for echo pin to report back -----//
    //make sure to delay about 50 microseconds before call to this method
    
    itsResponseTime = 0;
    
    // turn pulse on
    // check if the echo pin is sensing a ping first
    if (digitalRead(ITS_ECHO) != HIGH) {
        //make sure trigger is off
        digitalWrite(ITS_TRIGGER,LOW);
        delayMicroseconds(4);
        
        // sensor specs says to have a trigger duration of at least 10 microseconds
        digitalWrite(ITS_TRIGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(ITS_TRIGGER, LOW);
        
        //this will measure the time
        itsResponseTime = pulseIn(ITS_ECHO, HIGH, itsMaxDistanceTime);
    }
    
    if (itsResponseTime) {
        itsDistance = (((itsResponseTime/10000.0) * speed_of_sound)/2.00000);    //in cm, gain comes from the calibration procedure
    }
    //zero if error out
    else {
        itsDistance = 0;
    }
    
    return itsDistance;
    
}

void DistanceSensor::calibrate(float cal_length) {
    // cal_length is the length to an object you put in front of the sensor at a known distance
    //in cm
    int prev_speed_of_sound = 0;
    
    for(int i = 1; i <= 10; i++) {
        read_length = DistanceSensor::read();   // ask a read to get a response time
        speed_of_sound = cal_length/itsResponseTime;
        
        //get an average
        speed_of_sound = (speed_of_sound + prev_speed_of_sound)/i;
        prev_speed_of_sound = speed_of_sound;
        
        delay(100);
    }
}