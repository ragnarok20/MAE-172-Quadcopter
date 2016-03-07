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
    /*
    pinMode(motor_LED_test[0],OUTPUT);
    pinMode(motor_LED_test[1],OUTPUT);
    pinMode(motor_LED_test[2],OUTPUT);
    pinMode(motor_LED_test[3],OUTPUT);
    */
    
    
    mpu.initialize();
    if (mpu.testConnection()) {
        IMU_online = true;
        mpu.calibrateGyro();   //DONT move the MPU when Calibrating
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    }
    
    //AltitudeSonar.calibrate(10);
    
    // Set up motor signal wires with the servo library
    motor[0].attach(motor_LED_test[0]);
    motor[1].attach(motor_LED_test[1]);
    motor[2].attach(motor_LED_test[2]);
    motor[3].attach(motor_LED_test[3]);
    
    // Oneshot125 (125-250 microsecond pulses) protocol will be used. will arm on initialization
    motor[0].writeMicroseconds(1000);
    motor[1].writeMicroseconds(1000);
    motor[2].writeMicroseconds(1000);
    motor[3].writeMicroseconds(1000);
    
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
    begin_of_loop = micros();
    
    //Sonar
    if ((micros() - sonarTimer) > (1/sampleFreqSonar)*1000000 ) {
        Position[2] =  AltitudeSonar.read();
        sonarTimer = micros();
    }
    
    //------ Get Attitude ---------//
    if(IMU_online) {
        mpu.getMotion6(&acc_raw[0], &acc_raw[1], &acc_raw[2], &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
        //mpu.getRotation(&gyro_raw[0], &gy, &gyro_raw[2]);
        
        
        acc_raw[0] = acc_raw[0]*2.0f/32768.0f; // 2 g full range for accelerometer
        acc_raw[1] = acc_raw[1]*2.0f/32768.0f;
        acc_raw[2] = acc_raw[2]*2.0f/32768.0f;
        
        gyro_raw[0] = gyro_raw[0]*2000.0f/32768.0f; // 250 deg/s full range for gyroscope
        gyro_raw[1] = gyro_raw[1]*2000.0f/32768.0f;
        gyro_raw[2] = gyro_raw[2]*2000.0f/32768.0f;
        
        /*
        
         // this is for IMU madgwick algorithm
         gyro_raw[0] = gyro_raw[0]*PI/180.0f;
         gyro_raw[1] = gyro_raw[1]*PI/180.0f;
         gyro_raw[2] = gyro_raw[2]*PI/180.0f;
         
         //update quanternion
         MadgwickAHRSupdateIMU( gyro_raw[0], gyro_raw[1], gyro_raw[2], acc_raw[0], acc_raw[1], acc_raw[2]);
         
         //calculate euler
         
         Attitude[2]   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
         Attitude[0]  = -asin(2.0f * (q1 * q3 - q0 * q2));
         Attitude[1]   = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
         Attitude[0]  = Attitude[0] * 180.0f / PI;
         Attitude[2]    = Attitude[2] *180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
         Attitude[1]   = Attitude[1] * 180.0f / PI;
         */
        
        //integrate gyro rate to get angle if IMU algorithm not used
        Attitude[0] += gyro_raw[0]/measured_cycle_rate;   //pitch
        Attitude[1] += gyro_raw[1]/measured_cycle_rate;   //roll
        Attitude[2] += gyro_raw[2]/measured_cycle_rate;   //yaw
        
    }
    
    
    //---- update true Attitude of the quad & Fly----//

    //alpha.setPosition(Position);
    alpha.setAttitude(Attitude);
    alpha.steadyLevelFlight();

    // ------ ESC Signal Handling ---------//

    /*
    // map from 32 bit to 8bit PWM  for test
    mapped_signal[0] = map(*signals[0],-signed_16bits,signed_16bits,50,255);
    mapped_signal[1] = map(*signals[1],-signed_16bits,signed_16bits,50,255);
    mapped_signal[2] = map(*signals[2],-signed_16bits,signed_16bits,50,255);
    mapped_signal[3] = map(*signals[3],-signed_16bits,signed_16bits,50,255);

    
    analogWrite(motor_LED_test[0], mapped_signal[0]);
    analogWrite(motor_LED_test[1], mapped_signal[1]);
    analogWrite(motor_LED_test[2], mapped_signal[2]);
    analogWrite(motor_LED_test[3], mapped_signal[3]);
    */
    
    mapped_signal[0] = map(*signals[0],0,signed_16bits,1000,2000);
    mapped_signal[1] = map(*signals[1],0,signed_16bits,1000,2000);
    mapped_signal[2] = map(*signals[2],0,signed_16bits,1000,2000);
    mapped_signal[3] = map(*signals[3],0,signed_16bits,1000,2000);
    
    mapped_signal[0] = constrain(mapped_signal[0],1080,2000);
    mapped_signal[1] = constrain(mapped_signal[1],1080,2000);
    mapped_signal[2] = constrain(mapped_signal[2],1080,2000);
    mapped_signal[3] = constrain(mapped_signal[3],1080,2000);
    
    // Oneshot125 (125-250 microsecond pulses) protocol will be used
    motor[0].writeMicroseconds(mapped_signal[0]);
    motor[1].writeMicroseconds(mapped_signal[1]);
    motor[2].writeMicroseconds(mapped_signal[2]);
    motor[3].writeMicroseconds(mapped_signal[3]);
    
    
    
    //------ Echo to Screen if Defined ----------//
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
    
    //------Timer update -------//
    while (micros() - begin_of_loop < delayTime);
    
    // remeasure
    loop_time = micros() - begin_of_loop;     //microseconds
    dt = (float)loop_time/1000000;
    measured_cycle_rate = (1000000*(1/(float)loop_time));   //hz
}
