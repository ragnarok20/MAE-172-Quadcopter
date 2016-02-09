//
// Flight.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 1/22/16 5:17 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			Flight.h and ReadMe.txt for references
//


// Library header
#include "Flight.h"

// Code

//Constructor
QuadCopter::QuadCopter() {
    Yaw.setGains(yawGains);
    Pitch.setGains(pitchGains);
    Roll.setGains(rollGains);
    
}

QuadCopter::QuadCopter(float* ESCSignal[4]) {
    ESCSignal[0] = &escSignal[0];
    ESCSignal[1] = &escSignal[1];
    ESCSignal[2] = &escSignal[2];
    ESCSignal[3] = &escSignal[3];

    Yaw.setGains(yawGains);
    Pitch.setGains(pitchGains);
    Yaw.setGains(rollGains);
    
}

void QuadCopter::mixMotors() {
    //positve roll is a dip right in the quads frame
    //positve pitch is a moment upwards in the quads frame
    //positve yaw is a spin to the right
    
    //stabilization or control of each axis
    escSignal[0] = -Yaw.getControlSignal()/2 - Roll.getControlSignal()/2 - Pitch.getControlSignal()/2;            //motor 1
    escSignal[1] = Yaw.getControlSignal()/2 - Roll.getControlSignal()/2 + Pitch.getControlSignal()/2;           //motor 2
    escSignal[2] = Yaw.getControlSignal()/2 + Roll.getControlSignal()/2 - Pitch.getControlSignal()/2;           //motor 3
    escSignal[3] = -Yaw.getControlSignal()/2 + Roll.getControlSignal()/2 + Pitch.getControlSignal()/2;          //motor 4
    
    //append altitude control
    escSignal[0] += Altitude.getControlSignal();            //motor 1
    escSignal[1] += Altitude.getControlSignal();           //motor 2
    escSignal[2] += Altitude.getControlSignal();           //motor 3
    escSignal[3] += Altitude.getControlSignal();          //motor 4
    
    
}
// method to hold the quad level and at a set hieght 
void QuadCopter::steadyLevelFlight() {
    desiredAltitude = 100;  // in cm. will start with a hover 1m above ground
    
    // 0 degrees for level
    desiredAttitude[0] = 0;
    desiredAttitude[1] = 0;
    desiredAttitude[2] = 0;
    
}








