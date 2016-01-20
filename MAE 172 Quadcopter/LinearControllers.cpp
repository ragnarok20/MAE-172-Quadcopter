//
// LinearControllers.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 1/20/16 11:42 AM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			LinearControllers.h and ReadMe.txt for references
//


// Library header
#include "LinearControllers.h"

PIDController::PIDController(float desPos, float desVel, float actualPos, float actualVel) {
    itsDesPos = desPos;
    itsDesVel = desVel;
    
    itsActualPos = actualPos;
    itsActualVel = actualVel;
}

void PIDController::getControlSignal(int *controlSignal) {
    // calculate a differential time unit to compute the integral
    differentialTime = currentTime - previousTime;
    
    // calculate errors
    itsPosError = itsActualPos - itsDesPos;
    itsVelError = itsActualVel - itsDesVel;
    
    itsIntegralError = itsIntegralError + (itsPosError * differentialTime); // compute the integral
    
    //put all together
    *controlSignal = -itsKp*itsPosError - itsKd*itsVelError - itsKi*itsIntegralError;
    
}

// Code
