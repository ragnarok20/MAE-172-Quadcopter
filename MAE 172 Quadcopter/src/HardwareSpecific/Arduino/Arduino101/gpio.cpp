//
// gpio.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 1/22/16 5:08 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			gpio.h and ReadMe.txt for references
//

#ifdef __ARDUINO101__
#include "gpio.h"

void initializeSystem(){
#ifdef ECHO 
    Seirial.begin(9600)
#endif

    Wire.begin();
   
    CurieImu.initialize();
    if (CurieImu.testConnection()){
        IMU_online = true;
        CurieImu.autoCalibrateGyroOffset();
        CurieImu.autoCalibrateXAccelOffset(0);
        CurieImu.autoCalibrateYAccelOffset(0);
        CurieImu.autoCalibrateZAccelOffset(1);
        CurieImu.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
        CurieImu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);  
    }
    
    //Set up motor signal wires with the servo library
    motor[0].attach(motor_LED_test[0]);
    motor[1].attach(motor_LED_test[1]);
    motor[2].attach(motor_LED_test[2]);
    motor[3].attach(motor_LED_test[3]);
    
    motor[0].writeMicrosecond(1000);
    motor[1].writeMicrosecond(1000);    
    motor[2].writeMicrosecond(1000);
    motor[3].writeMicrosecond(1000);
    
    #ifdef ECHO
        if (IMU_online){
            Serial.println("Imu online & Calibrated");
        }
        else {
            Serial.println("It's dead Jim.");
        } 
    #endif
}

void processIO(){
    begin_of_loop = micros();
    
    if ((micros() - sonarTimer) > (1/sampleFreqSonar)*1000){
        Position[2] = AltitudeSonar.read();
        sonarTimer = micros();
    }
    
    if(IMU_online){
        CurieImu.getMotion6(&acc_raw[0], &acc_raw[1], &acc_raw[2], &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
        
        acc_raw[0] = acc_raw[0]*2.0f/32768.0f; // 2 g full range for accelerometer
        acc_raw[1] = acc_raw[1]*2.0f/32768.0f;
        acc_raw[2] = acc_raw[2]*2.0f/32768.0f;
        
        gyro_raw[0] = gyro_raw[0]*2000.0f/32768.0f; // 250 deg/s full range for gyroscope
        gyro_raw[1] = gyro_raw[1]*2000.0f/32768.0f;
        gyro_raw[2] = gyro_raw[2]*2000.0f/32768.0f;
        
        Attitude[0] += gyro_raw[0]/measured_cycle_rate;   //pitch
        Attitude[1] += gyro_raw[1]/measured_cycle_rate;   //roll
        Attitude[2] += gyro_raw[2]/measured_cycle_rate;   //yaw
        
    }
    
    // Ask Sage about alpha class
    alpha.setAttitude(Attitude);
    alpha.steadyLevelFlight();
    
    mapped_signal[0] = map(*signals[0],0,signed_16bits,1000,2000);
    mapped_signal[1] = map(*signals[1],0,signed_16bits,1000,2000);
    mapped_signal[2] = map(*signals[2],0,signed_16bits,1000,2000);
    mapped_signal[3] = map(*signals[3],0,signed_16bits,1000,2000);
    
    mapped_signal[0] = constrain(mapped_signal[0],1080,2000);
    mapped_signal[1] = constrain(mapped_signal[1],1080,2000);
    mapped_signal[2] = constrain(mapped_signal[2],1080,2000);
    mapped_signal[3] = constrain(mapped_signal[3],1080,2000);
    
    #ifdef ECHO
    
    if (IMU_online) {
        Serial.print("yaw: ");
        Serial.print(Attitude[2]);
        Serial.print("\t pitch: ");
        Serial.print(Attitude[0]);
        Serial.print("\t roll 3: ");
        Serial.print(Attitude[1]);
        
        
        Serial.print("\t motor 1: ");
        Serial.print(mapped_signal[0]);
        Serial.print("\t motor 2: ");
        Serial.print(mapped_signal[1]);
        Serial.print("\t motor 3: ");
        Serial.print(mapped_signal[2]);
        Serial.print("\t motor 4: ");
        Serial.print(mapped_signal[3]);
        
        Serial.print("\t Altitude: ");
        Serial.print(Position[2]);
        
        
        Serial.print("\t Sample Rate: ");
        Serial.println(measured_cycle_rate);
    }

#endif
    
    // Timer update
    while (micros() - begin_of_loop < delayTime);
    
    // remeasure
    loop_time = micros() - begin_of_loop;     //microseconds
    dt = (float)loop_time/1000000;
    measured_cycle_rate = (1000000*(1/(float)loop_time));   //hz
}
#endif