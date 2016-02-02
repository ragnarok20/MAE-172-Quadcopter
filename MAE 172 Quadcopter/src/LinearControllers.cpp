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
template <class TT> PIDController<TT>::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
}

template <class TT> PIDController<TT>::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, TT desiredOutput) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    itsDesiredOutput = desiredOutput;
}

template <class TT> PIDController<TT>::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, TT desiredOutput, TT feedback) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    itsDesiredOutput = desiredOutput;
    itsFeedback = feedback;
}

// Methods

template <class TT> void PIDController<TT>::setGains(float Kp, float Kd, float Ki) {
    itsKp = Kp; itsKd = Kd; itsKi = Ki; // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
}

template <class TT> void PIDController<TT>::setDesiredOuptut(TT desiredOutput) {
    itsDesiredOutput = desiredOutput;
}
template <class TT> void PIDController<TT>::setFeedback(TT feedback) {
    itsFeedback = feedback;
}

template <class TT> void PIDController<TT>::update() {
    // calculate errors
    itsPorportionalError = itsDesiredOutput - itsFeedback;
    itsDerivativeError = itsPorportionalError/dt;
    
    itsIntegralError = itsIntegralError + (itsPorportionalError * dt); // compute the integral
    
    //put all together
    itsControlSignal = -itsKp*itsPorportionalError - itsKd*itsDerivativeError - itsKi*itsIntegralError;
}

