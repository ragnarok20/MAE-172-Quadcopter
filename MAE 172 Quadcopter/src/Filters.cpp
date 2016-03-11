//
// Filters.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 3/10/16 12:17 PM
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

float Filter::update(float sample) {
    currSample = sample;
    filteredSignal = a*currSample + b*prevSample;
    prevSample = currSample;
    
    return filteredSignal;
    
}

Vector3<float> Filter::update(Vector3<float> sampleVec) {
    
    currSampleVec = sampleVec;
    filteredSignalVec = currSampleVec*a + prevSampleVec*b;
    prevSampleVec = filteredSignalVec;
    
    return filteredSignalVec;
    
}