//
// gpio.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thacc_raw[1]er, 1/22/16 5:08 PM
// 				Sage Thacc_raw[1]er
//
// Copyright 	(c) Sage Thacc_raw[1]er, 2016
// Licence		<#license#>
//
// See 			gpio.h and ReadMe.txt for references
//
// Library header
#include "gpio.h"

// Code

void initializeSystem() {
    
#ifdef ECHO
    Serial.begin(115200);
#endif
    
    Wire.begin();
    
    pinMode(motor_LED_test[0],OUTPUT);
    pinMode(motor_LED_test[1],OUTPUT);
    pinMode(motor_LED_test[2],OUTPUT);
    pinMode(motor_LED_test[3],OUTPUT);
    
    mpu.initialize();
    if (mpu.testConnection()) {
        IMU_online = true;
        mpu.calibrateGyro();   //DONT move the MPU when Calibrating
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    }
    
#ifdef ECHO
    if (IMU_online) {
        Serial.println("MPU_6050 Online & Calibrated");
    }
    else {
        Serial.println("MPU_6050 Not Responding");
    }
#endif
    
}

void processIO() {
    begin_of_loop = millis();
    
    //------ Get Attitude ---------//
    if(IMU_online) {
        mpu.getMotion6(&acc_raw[0], &acc_raw[1], &acc_raw[2], &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
        //mpu.getRotation(&gyro_raw[0], &gy, &gyro_raw[2]);
        
        
        acc_raw[0] = acc_raw[0]*2.0f/32768.0f; // 2 g full range for accelerometer
        acc_raw[1] = acc_raw[1]*2.0f/32768.0f;
        acc_raw[2] = acc_raw[2]*2.0f/32768.0f;
        
        gyro_raw[0] = gyro_raw[0]*250.0f/32768.0f; // 250 deg/s full range for gyroscope
        gyro_raw[1] = gyro_raw[1]*250.0f/32768.0f;
        gyro_raw[2] = gyro_raw[2]*250.0f/32768.0f;

        /*
         gyro_raw[0] = gyro_raw[0]*PI/180.0f;
         gyro_raw[1] = gyro_raw[1]*PI/180.0f;
         gyro_raw[2] = gyro_raw[2]*PI/180.0f;
         
         //update quanternion
         MadgwickAHRSupdateIMU( gyro_raw[0], gy, gyro_raw[2], acc_raw[0], acc_raw[1], acc_raw[2]);
         
         //calculate euler
         
         yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
         pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
         roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
         pitch = pitch * 180.0f / PI;
         yaw   = yaw *180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
         roll  = roll * 180.0f / PI;
         
         
         //integrate gyro rate to get angle
         */
        attitude[0] += gyro_raw[0]*1/measured_cycle_rate;   //pitch
        attitude[1] += gyro_raw[1]*1/measured_cycle_rate;   //roll
        attitude[2] += gyro_raw[2]*1/measured_cycle_rate;   //yaw
        
    }
    
    //---- update true attitude of the quad ----//
    alpha.setPosition(attitude);

    // ------ ESC Signal Handling ---------//
    //constrain signals to int
    int i;
    for (i=0; i<4; i++) {
        if ((int)signals[i] >= 65535) {
            mapped_signal[i] = 65535;
        }
        else {
            mapped_signal[i] = (int)signals[i];
        }
    }
    
    // map to 8bit PWM  for test
    mapped_signal[0] = map(mapped_signal[0],0,65535,0,255);
    mapped_signal[1] = map(mapped_signal[1],0,65535,0,255);
    mapped_signal[2] = map(mapped_signal[2],0,65535,0,255);
    mapped_signal[3] = map(mapped_signal[3],0,65535,0,255);

    analogWrite(motor_LED_test[0], mapped_signal[0]);
    analogWrite(motor_LED_test[1], mapped_signal[1]);
    analogWrite(motor_LED_test[2], mapped_signal[2]);
    analogWrite(motor_LED_test[3], mapped_signal[3]);
    
    //------Timer update -------//
    loop_time = millis() - begin_of_loop;     //milliseconds
    measured_cycle_rate = 1000*(1/loop_time);   //hz
    if (measured_cycle_rate > sampleFreq) {
        delay_time = ((float)(1/sampleFreq)*1000) - loop_time;
        delay(delay_time);
    }
    // remeasure
    loop_time = millis() - begin_of_loop;     //milliseconds
    measured_cycle_rate = 1000*(1/loop_time);   //hz
}
