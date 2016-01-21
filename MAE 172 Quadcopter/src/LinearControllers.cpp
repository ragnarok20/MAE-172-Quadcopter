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

// Constructors and overloads
PIDController::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
}

PIDController::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, int desiredOutput) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    itsDesiredOutput = desiredOutput;
}

PIDController::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, int desiredOutput, int feedback) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    itsDesiredOutput = desiredOutput;
    itsFeedback = feedback;
}

// Methods

void PIDController::setGains(float Kp, float Kd, float Ki) {
    itsKp = Kp; itsKd = Kd; itsKi = Ki; // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
}

void PIDController::setDesiredOuptut(int desiredOutput) {
    itsDesiredOutput = desiredOutput;
}
void PIDController::setFeedback(int feedback) {
    itsFeedback = feedback;
}

void PIDController::getControlSignal(int *controlSignal) {
    
    // calculate errors
    itsPorportionalError = itsDesiredOutput - itsFeedback;
    itsDerivativeError = itsPorportionalError/dt;
    
    itsIntegralError = itsIntegralError + (itsPorportionalError * dt); // compute the integral
    
    //put all together
    *controlSignal = -itsKp*itsPorportionalError - itsKd*itsDerivativeError - itsKi*itsIntegralError;
    
}

// Code
