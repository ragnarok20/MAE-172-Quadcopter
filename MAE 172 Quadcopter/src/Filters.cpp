//
// Filters.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 1/22/16 5:03 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			Filters.h and ReadMe.txt for references
//


// Library header
#include "Filters.h"

// Code

// Bandpass standard
float bandpass(int signal) {
    //constatnts from http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html with bandpass 4 poles
    const float b[4] = {-0.3851904848, 1.7942980358, -3.3989739250, 2.9877331071};	//2 hz to 20 hz
    const float GAIN_BANDPASS = 9.413148578;
    
    xv[0] = xv[1];
    xv[1] = xv[2];
    xv[2] = xv[3];
    xv[3] = xv[4];
    
    xv[4] = signal /GAIN_BANDPASS;
    
    yv[0] = yv[1];
    yv[1] = yv[2];
    yv[2] = yv[3];
    yv[3] = yv[4];
    
    yv[4] = (xv[0] + xv[4]) - 2 * xv[2] + ( b[0] * yv[0]) + (  b[1] * yv[1]) + ( b[2] * yv[2]) + ( b[3] * yv[3]);
    return yv[4];
    
}

// Bandpass overload to set gains
float bandpass(int signal, const float * recurssive_constants, const float GAIN_BANDPASS) {
    
    xv[0] = xv[1];
    xv[1] = xv[2];
    xv[2] = xv[3];
    xv[3] = xv[4];
    
    xv[4] = signal /GAIN_BANDPASS;
    
    yv[0] = yv[1];
    yv[1] = yv[2];
    yv[2] = yv[3];
    yv[3] = yv[4];
    
    yv[4] = (xv[0] + xv[4]) - 2 * xv[2] + ( recurssive_constants[0] * yv[0]) + (  recurssive_constants[1] * yv[1]) + ( recurssive_constants[2] * yv[2]) + ( recurssive_constants[3] * yv[3]);
    return yv[4];
    
}

// high pass filter standard
float HPF(int signal) {
    
    //constatnts from http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html with butterworth 2 poles
    // 1 hz cutoff
    const float H = 0.9629940582;
    const float GAIN_HPF = 1.018851785;
    
    //first order high pass filter
    hxv[0] = hxv[1];
    hxv[1] = signal / GAIN_HPF;
    
    hyv[0] = hyv[1];
    hyv[1] =   (hxv[1] - hxv[0]) + (  H * hyv[0]);
    
    return hyv[1];
    
}

// high pass filter overload to set gains
float HPF(int signal, const float H, const float GAIN_HPF) {
    
    //first order high pass filter
    hxv[0] = hxv[1];
    hxv[1] = signal / GAIN_HPF;
    
    hyv[0] = hyv[1];
    hyv[1] =   (hxv[1] - hxv[0]) + (  H * hyv[0]);
    
    return hyv[1];
    
}

// low pass filter standard
float LPF(int signal) {
    //constatnts from http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html with butterworth 2 poles
    //10 Hz cuttof
    const float L[2] = {-0.5869195143,1.4754804538};
    const float GAIN_LPF = 35.89405709;
    
    
    //2nd oreder low pas filter
    lxv[0] = lxv[1];
    lxv[1] = lxv[2];
    lxv[2] = signal / GAIN_LPF;
    
    lyv[0] = lyv[1];
    lyv[1] = lyv[2];
    lyv[2] =   (lxv[0] + lxv[2]) + 2 * lxv[1] + ( L[0] * lyv[0]) + (  L[1] * lyv[1]);
    
    return lyv[2];
    
}

// low pass filter overload to set gains
float LPF(int signal, const float * recurssive_constants, const float GAIN_LPF) {
    
    //2nd oreder low pas filter
    lxv[0] = lxv[1];
    lxv[1] = lxv[2];
    lxv[2] = signal / GAIN_LPF;
    
    lyv[0] = lyv[1];
    lyv[1] = lyv[2];
    lyv[2] =   (lxv[0] + lxv[2]) + 2 * lxv[1] + ( recurssive_constants[0] * lyv[0]) + (  recurssive_constants[1] * lyv[1]);
    
    return lyv[2];
    
}




