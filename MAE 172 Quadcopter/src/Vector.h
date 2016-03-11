//
// File			Vector.h
// Header
//
// Details		<#details#>
//
// Project		 MAE 172 Quadcopter
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author		Sage Thayer
// 				Sage Thayer
//
// Date			1/31/16 11:01 AM
// Version		<#version#>
//
// Copyright	© Sage Thayer, 2016
// Licence    <#license#>
//
// See			ReadMe.txt for references
//


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
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
#elif defined(SPARK) // Spark specific
#   include "application.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE


#ifndef Vector_
#define Vector_

template <class T>

class Vector3 {
public:
    //constructors and destructor
    Vector3() {elem[0] = 0; elem[1] = 0; elem[2] = 0;}; //default constructor creates zero vector
    Vector3(T &x, T &y, T &z) {elem[0] = x; elem[1] = y; elem[2] = z;};
    ~Vector3(){};
    
    //--------------methods-----------------//
    T magnitude() {
        T mag;
        mag = elem[1]^2 + elem[2]^2 + elem[3]^2;
        mag = sqrt(mag);
        return mag;
    }
    
    Vector3<T>& normalize() {
        Vector3<T> v; //init's a 0 Vector
        T mag = Vector3<T>::magnitude();
        
        for (int i = 0; i < 3; i++)
            v.elem[i] = elem[i]/mag;
        return v;
    }
    T dot_with(Vector3<T>& a) {
        T result = 0;
        
        for (int i = 0; i < 3; i++)
            result += elem[i] * a.elem[i];
        return result;
    }
    
    //--------operator overloads-------------//
    T& operator[](int i){return elem[i];}; //returns the directed element
    
    bool operator==(Vector3<T>& right) {
        for (int i = 0; i < 3; i++){
            if (elem[i] != right[i])
                return false;
        }
        
        return true;
    }
    Vector3<T> operator=(const Vector3<T>& a) {
        //copy vectors
        elem[0] = a.elem[0];
        elem[1] = a.elem[1];
        elem[2] = a.elem[2];
        return *this;
    }
    operator Vector3<int>() {
        //change to integers
        Vector3<int> v;
        v.elem[0] = (int)elem[0];
        v.elem[1] = (int)elem[1];
        v.elem[2] = (int)elem[2];
        return v;
    }
    operator Vector3<short>() {
        //change to integers
        Vector3<short> v;
        v.elem[0] = (short)elem[0];
        v.elem[1] = (short)elem[1];
        v.elem[2] = (short)elem[2];
        return v;
    }
    operator Vector3<float>() {
        //change to integers
        Vector3<float> v;
        v.elem[0] = (float)elem[0];
        v.elem[1] = (float)elem[1];
        v.elem[2] = (float)elem[2];
        return v;
    }
    
    Vector3<T> operator+(const Vector3<T>& a) {
        Vector3<T> v; //init's a 0 Vector
        //add individual elements
        for (int i = 0; i < 3; i++)
            v.elem[i] = elem[i] + a.elem[i];
        return v;
    }
    
    Vector3<T> operator-(const Vector3<T>& a) {
        Vector3<T> v; //init's a 0 Vector
        //subtract individual elements
        for (int i = 0; i < 3; i++)
            v.elem[i] = elem[i] - a.elem[i];
        return v;
    }
    
    //Cross Product
    Vector3<T> operator*(const Vector3<T>& a) {
        Vector3<T> v; //init's a 0 Vector
    
        v.elem[1] = elem[2]*a.elem[3] - elem[3]*a.elem[2];
        v.elem[2] = elem[3]*a.elem[1] - elem[1]*a.elem[3];
        v.elem[3] = elem[1]*a.elem[2] - elem[2]*a.elem[1];
        return v;
    }
    //Scalar times the vector
    Vector3<T> operator*(const T& a) {
        Vector3<T> v; //init's a 0 Vector
        
        v.elem[0] = elem[0] * a;
        v.elem[1] = elem[1] * a;
        v.elem[2] = elem[2] * a;
        return v;
    }
    
    //Scalar divide the vector
    Vector3<T> operator/(const T& a) {
        Vector3<T> v; //init's a 0 Vector
        
        v.elem[0] = elem[0] / a;
        v.elem[1] = elem[1] / a;
        v.elem[2] = elem[2] / a;
        return v;
    }


    //elements
    T elem[3];
    
};



#endif
