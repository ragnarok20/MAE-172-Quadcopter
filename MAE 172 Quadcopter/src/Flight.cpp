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
QuadCopter::QuadCopter(float* Dt) {

    Yaw.setDifferentialTime(Dt);
    Pitch.setDifferentialTime(Dt);
    Roll.setDifferentialTime(Dt);
    
    //Set the controller feedbacks (actual position)
    Yaw.setFeedback(&attitude[2]);
    Pitch.setFeedback(&attitude[0]);
    Roll.setFeedback(&attitude[1]);
    
    //Standard gains
    yawGains[0] = 1.0; yawGains[1] = 5; yawGains[2] = 10;   //P, D, DD
    pitchGains[0] = 1.0; pitchGains[1] = 5; pitchGains[2] = 10;   //P, D, DD
    rollGains[0] = 1.0; rollGains[1] = 5; rollGains[2] = 10;   //P, D, DD
    
    //Standard Mixer Percent
    yawPercent = .2;
    pitchPercent = .4;
    rollPercent = .4;
    
    Yaw.setGains(yawGains);
    Pitch.setGains(pitchGains);
    Roll.setGains(rollGains);
    
}

QuadCopter::QuadCopter(float* Dt, T* ESCSignal[4]) {
    Yaw.setDifferentialTime(Dt);
    Pitch.setDifferentialTime(Dt);
    Roll.setDifferentialTime(Dt);
    
    ESCSignal[0] = &escSignal[0];
    ESCSignal[1] = &escSignal[1];
    ESCSignal[2] = &escSignal[2];
    ESCSignal[3] = &escSignal[3];
    
    //Set the controller feedbacks (actual position)
    Yaw.setFeedback(&attitude[2]);
    Pitch.setFeedback(&attitude[0]);
    Roll.setFeedback(&attitude[1]);
    
    //Standard gains
    yawGains[0] = 1.0; yawGains[1] = 5; yawGains[2] = 10;   //P, D, DD
    pitchGains[0] = 1.0; pitchGains[1] = 5; pitchGains[2] = 10;   //P, D, DD
    rollGains[0] = 1.0; rollGains[1] = 5; rollGains[2] = 10;   //P, D, DD
    
    yawGains = yawGains/10;
    pitchGains = yawGains/10;
    rollGains = yawGains/10;
    
    yawPercent = .2;
    pitchPercent = .4;
    rollPercent = .4;

    Yaw.setGains(yawGains);
    Pitch.setGains(pitchGains);
    Roll.setGains(rollGains);
    
}

void QuadCopter::mixMotors() {
    //positve roll is a dip right in the quads frame
    //positve pitch is a moment upwards in the quads frame
    //positve yaw is a spin to the right
    
    //stabilization or control of each axis
    escSignal[0] = -(yawPercent)*Yaw.getControlSignal()/2 + (rollPercent)*Roll.getControlSignal()/2 + (pitchPercent)*Pitch.getControlSignal()/2;            //motor 1
    escSignal[1] = (yawPercent)*Yaw.getControlSignal()/2 + (rollPercent)*Roll.getControlSignal()/2 - (pitchPercent)*Pitch.getControlSignal()/2;           //motor 2
    escSignal[2] = (yawPercent)*Yaw.getControlSignal()/2 - (rollPercent)*Roll.getControlSignal()/2 + (pitchPercent)*Pitch.getControlSignal()/2;           //motor 3
    escSignal[3] = -(yawPercent)*Yaw.getControlSignal()/2 - (rollPercent)*Roll.getControlSignal()/2 - (pitchPercent)*Pitch.getControlSignal()/2;          //motor 4
    
   /*
    //append altitude control
    escSignal[0] += Altitude.getControlSignal();            //motor 1
    escSignal[1] += Altitude.getControlSignal();           //motor 2
    escSignal[2] += Altitude.getControlSignal();           //motor 3
    escSignal[3] += Altitude.getControlSignal();          //motor 4
    */
    
}
// method to hold the quad level and at a set hieght 
void QuadCopter::steadyLevelFlight() {
    
    //Set the controller desired position
    Yaw.setDesiredOuptut(0);
    Pitch.setDesiredOuptut(0);
    Roll.setDesiredOuptut(0);
    
    Yaw.update();
    Pitch.update();
    Roll.update();
    
    this->mixMotors();
    
}








