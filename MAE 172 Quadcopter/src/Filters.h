///
/// @file		Filters.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/22/16 5:03 PM
/// @version	<#version#>
/// 
/// @copyright	(c) Sage Thayer, 2016
/// @copyright	<#license#>
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
#include "ArduinoAll.h"

#ifndef Filters_cpp
#define Filters_cpp

//---------------------------IIR bandpass filter------------------------------------//
//---Implemented from http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html ---//

float bandpass(int signal); //2 hz to 20 hz
float bandpass(int signal, const float * recurssive_constants[], const float GAIN_BANDPASS);

// Keep these global since we need to do recursive series math
float xv[5];
float yv[5];

//----------IIR HPF & LPF -------------------//
//Filter implemented: http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
//Butterworth method used
//1st order HPF
//2nd order LPF

float HPF(int signal);  //1hz cutoff
float LPF(int signal);  //10hz cutoff

//overloads
float HPF(int signal, const float * recurssive_constants[], const float GAIN_HPF);
float LPF(int signal, const float * recurssive_constants[], const float GAIN_LPF);

// Keep these global since we need to do recursive series math
float hxv[3];
float hyv[3];
float lxv[3];
float lyv[3];



#endif
