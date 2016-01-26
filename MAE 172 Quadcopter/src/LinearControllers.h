///
/// @file		LinearControllers.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/20/16 11:42 AM
/// @version	<#version#>
/// 
/// @copyright	(c) Sage Thayer, 2016
/// @copyright	<#license#>
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
#include "ArduinoAll.h"

#ifndef LinearControllers_cpp
#define LinearControllers_cpp

//------------------------------------------------------------------------------//
//-----------------------General PID controller object--------------------------//
//------------------------------------------------------------------------------//

class PIDController {
public:
    //constructors and destructor
    PIDController(unsigned long differentialTime){itsKp = 0; itsKd = 0; itsKi = 0; dt = differentialTime;};
    PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki);
    PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, int desiredOutput);
    PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, int desiredOutput, int feedback);
    ~PIDController(){};
    
    //methods
    void setGains(float Kp, float Kd, float Ki);
    void setDesiredOuptut(int desiredOutput);
    void setFeedback(int input);
    
    void getControlSignal(int *controlSignal);
    
private:
    int itsControlSignal;
    float itsKp, itsKd, itsKi;   // controller gains:: Kp: position, Kd: derivative, Ki: integral
    int itsDesiredOutput, itsFeedback;
    int itsPorportionalError;
    float itsDerivativeError, itsIntegralError;
    unsigned long dt;
    
};






#endif
