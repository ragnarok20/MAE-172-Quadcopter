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
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#   include "libpandora_types.h"
#   include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#   include "Arduino.h"
#elif defined(SPARK) // Spark specific
#   include "application.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#elif defined(CurieIMU) // Arduino 101 Gyro
#include "CurieIMU.h"
#else // error
#   error Platform not defined
#endif // end IDE

#ifndef LinearControllers_cpp
#define LinearControllers_cpp

#include "Vector.h"
//------------------------------------------------------------------------------//
//-----------------------General PID controller object--------------------------//
//------------------------------------------------------------------------------//


template <class TT>
class PIDController {
public:
    //--------constructors and destructor-------------//
    PIDController(){itsKp = 0; itsKd = 0; itsKi = 0;}
    PIDController(unsigned long* differentialTime){itsKp = 0; itsKd = 0; itsKi = 0; dt = differentialTime;}
    PIDController(unsigned long* differentialTime, Vector3<float> gains) {
        //constructor overload. set private vars
        dt = differentialTime;
        itsKp = gains[0]; itsKd = gains[1]; itsKi = gains[2];   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    }
    
    PIDController(unsigned long* differentialTime, Vector3<float> gains, TT desiredOutput) {
        //constructor overload. set private vars
        dt = differentialTime;
        itsKp = gains[0]; itsKd = gains[1]; itsKi = gains[2];   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
        itsDesiredOutput = desiredOutput;
    }
    PIDController(unsigned long* differentialTime, Vector3<float> gains, TT desiredOutput, TT *feedback) {
        //constructor overload. set private vars
        dt = differentialTime;
        itsKp = gains[0]; itsKd = gains[1]; itsKi = gains[2];   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
        itsDesiredOutput = desiredOutput;
        itsFeedback = feedback;
    }
    ~PIDController(){};
    
    //------------methods-------------------//
    void setGains(Vector3<float> gains) {itsKp = gains[0]; itsKd = gains[1]; itsKi = gains[2];}
    void setDesiredOuptut(TT desiredOutput) { itsDesiredOutput = desiredOutput;}
    void setFeedback(TT* feedback) {itsFeedback = feedback;}
    void setDifferentialTime(float* Dt) {dt = Dt;}
    TT getControlSignal() { return (TT)itsControlSignal;}
    
    void update() {
        //reset
        itsPorportionalError = 0;
        itsDerivativeError = 0;
        itsControlSignal = 0;
        
        // calculate errors
        itsPorportionalError = itsDesiredOutput - (*itsFeedback);
        itsDerivativeError = itsPorportionalError/(*dt);
        itsIntegralError = itsIntegralError + (itsPorportionalError * (*dt)); // compute the integral
        
        //put all together
        itsControlSignal = -itsKp*itsPorportionalError - itsKd*itsDerivativeError - itsKi*itsIntegralError;
    }
    
private:
    float itsControlSignal;
    float itsKp, itsKd, itsKi;   // controller gains:: Kp: position, Kd: derivative, Ki: integral
    TT itsDesiredOutput;
    TT*  itsFeedback;       //pointer since its usually a sensor signal
    float itsPorportionalError;
    float itsDerivativeError, itsIntegralError;
    float *dt;
    float dt_sec;
};






#endif
