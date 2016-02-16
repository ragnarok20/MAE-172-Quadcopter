///
/// @file		Flight.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project MAE 172 Quadcopter
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/22/16 5:17 PM
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
include "CurieIMU.h"
#else // error
#   error Platform not defined
#endif // end IDE

#include "LinearControllers.h"
#include "Vector.h"

#ifndef Flight_cpp
#define Flight_cpp

//Easy way to switch between math types of the controller
typedef int T;
//typedef float T;

// Rigid body Abstract Data Type
class RigidBody {
public:
    //constructors
    RigidBody(){};
    virtual ~RigidBody(){};
    
    //virtual methods to overload
    virtual void setPosition(Vector3<T> a) = 0;
    virtual void setVelocity(Vector3<T> a) = 0;
    virtual void setAcceleration(Vector3<T> a) = 0;
    
    virtual void setAttitude(Vector3<T> a) = 0;
    virtual void setAngularRates(Vector3<T> a) = 0;
    virtual void setAngularAccelerations(Vector3<T> a) = 0;
    
    virtual void setInertias(int& mass, int& momentOfInertia) = 0;
    
    //accessors
protected:
    //inertial Dynamics
    int Mass;   //in grams
    int I;      //in grams*cm^2
    
    // linear dynamics
    Vector3<T> position;    //x,y,z
    Vector3<T> velocity;      //u,v,w
    Vector3<T> acceleration;
    
    //rotation dynamics
    Vector3<T> attitude;
    Vector3<T> angular_rates;
    Vector3<T> angular_acc;
    
};

class QuadCopter : public RigidBody{
public:
    //constructors
    QuadCopter(float *Dt);
    QuadCopter(float *Dt, T* ESCSignal[4]);  //output signal to esc's to be handled by gpio
    ~QuadCopter(){};
    
    //parent methods
    void setPosition(Vector3<T> a) {position = a;}
    void setVelocity(Vector3<T> a) {velocity = a;}
    void setAcceleration(Vector3<T> a) {acceleration = a;}
    
    void setAttitude(Vector3<T> a) {attitude = a;}
    void setAngularRates(Vector3<T> a){angular_rates = a;}
    void setAngularAccelerations(Vector3<T> a) {angular_acc = a;}
    
    void setInertias(int& mass, int& momentOfInertia) {Mass = mass; I = momentOfInertia;}
    
    //methods
    void mixMotors();   //mix the control signals and map to the esc signal outputs
    void steadyLevelFlight();
    void ascend(T altitude);
    void descend(T altitude);
    void land();
    
    //public types
    
    PIDController<T> Yaw;
    PIDController<T> Pitch;
    PIDController<T> Roll;
    PIDController<T> Altitude;
    
    
private:
    //PID gains vector: P, D, I respectively
    Vector3<float> yawGains;
    Vector3<float> pitchGains;
    Vector3<float> rollGains;
    Vector3<float> altitudeGains;
    float yawPercent, pitchPercent, rollPercent;   //will represent how much influence on the mixer each controller has
    
    Vector3<T> desiredAttitude;
    
    T desiredYaw = 0;
    T desiredPitch = 0;
    T desiredRoll = 0;
    T desiredAltitude = 0;
    
    
    T escSignal[4];
    
};



#endif
