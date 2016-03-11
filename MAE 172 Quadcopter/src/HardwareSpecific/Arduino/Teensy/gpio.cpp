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

//#ifdef __TEENSY__
#include "gpio.h"

// Code

void initializeSystem() {
    
#ifdef ECHO
    Serial.begin(115200);
#endif
    
    Wire.begin();
    pinMode(15, INPUT);
    pinMode(13, OUTPUT);
    
    // Set up motor signal wires with the servo library
    motor[0].attach(motor_LED_test[0]);
    motor[1].attach(motor_LED_test[1]);
    motor[2].attach(motor_LED_test[2]);
    motor[3].attach(motor_LED_test[3]);
    
    // Oneshot125 (125-250 microsecond pulses) protocol will be used. will arm on initialization
    //motor[0].writeMicroseconds(1000);
    //motor[1].writeMicroseconds(1000);
    //motor[2].writeMicroseconds(1000);
    //motor[3].writeMicroseconds(1000);
    
    mpu.initialize();
    if (mpu.testConnection()) {
        IMU_online = true;
        
        digitalWrite(13, HIGH);
        mpu.calibrateGyro();   //DONT move the MPU when Calibrating
        digitalWrite(13, LOW);
        mpu.initialize();
        
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu.setDLPFMode(MPU6050_DLPF_BW_256);
        mpu.setRate(SAMPLE_RATE_DIV);
    }
    
    //AltitudeSonar.calibrate(10);
    //altimeter.begin();
    //altimeter.calibrate(0);   //100 ft
    
    
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
        //Position[2] =  AltitudeSonar.read();
        sonarTimer = micros();
        //Position[2] =  sonar.ping_cm();
    }
    
    //altimeter
   // Position[2] = altimeter.readAltitude();
    
    //------ Get Attitude ---------//
    if(IMU_online) {
        
        mpu.getMotion6(&acc_raw[0], &acc_raw[1], &acc_raw[2], &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
        
        acc_raw_float = (Vector3<float>)acc_raw * (2.0f/32768.0f);  // 2 g full range for accelerometer
        gyro_raw_float = (Vector3<float>)gyro_raw * (250.0f/32768.0f); // 250 deg/s full range for gyroscope
        
        //filter
        gyro_filtered = GyroLPF.update(gyro_raw_float);
        acc_filtered = AccLPF.update(acc_raw_float);
        
        // convert degrees to radians
        gyro_filtered = gyro_filtered * PI/180.0f;
        
        //update quanternion
        MadgwickAHRSupdateIMU( gyro_filtered[0], gyro_filtered[1], gyro_filtered[2], acc_raw_float[0], acc_raw_float[1], acc_raw_float[2]);
         
        //calculate euler
        
        // Madgwicks algorthm corrects pitch and roll really well but w/o mag, yaw will diverge
        // I will integrate yaw to achieve a more sable number not necessarily a true heading though.
        
        //Attitude[2]   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        Attitude[0]  = -asin(2.0f * (q1 * q3 - q0 * q2));
        Attitude[1]   = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        
        //convert back to degrees
        Attitude[0] = Attitude[0] * 180.0f / PI;
        Attitude[1] = Attitude[1] * 180.0f / PI;
        
        //integrate yaw axis
        Attitude[2] += gyro_raw_float[2]/measured_cycle_rate;   //yaw
        
        //AttitudeFiltered = lpf.update(Attitude);
        
    }
    
    
    //---- update true Attitude of the quad & Fly----//

    //alpha.setPosition(Position);
    alpha.setAttitude(Attitude);
    alpha.steadyLevelFlight();

    // ------ ESC Signal Handling ---------//

    
#ifdef oneshot125
    
    // Oneshot125 (125-250 microsecond pulses) protocol will be used
    // mapping the signal ouptuts to microseconds
    mapped_signal[0] = map(*signals[0],0,signed_16bits,125,250);
    mapped_signal[1] = map(*signals[1],0,signed_16bits,125,250);
    mapped_signal[2] = map(*signals[2],0,signed_16bits,125,250);
    mapped_signal[3] = map(*signals[3],0,signed_16bits,125,250);
    
    mapped_signal[0] = constrain(mapped_signal[0],125,250);
    mapped_signal[1] = constrain(mapped_signal[1],125,250);
    mapped_signal[2] = constrain(mapped_signal[2],125,250);
    mapped_signal[3] = constrain(mapped_signal[3],125,250);
#endif
    // normal mapping
    
     mapped_signal[0] = map(*signals[0],0,10*signed_16bits,1000,2000);
     mapped_signal[1] = map(*signals[1],0,10*signed_16bits,1000,2000);
     mapped_signal[2] = map(*signals[2],0,10*signed_16bits,1000,2000);
     mapped_signal[3] = map(*signals[3],0,10*signed_16bits,1000,2000);
     
     mapped_signal[0] = constrain(mapped_signal[0],1000,2000);
     mapped_signal[1] = constrain(mapped_signal[1],1000,2000);
     mapped_signal[2] = constrain(mapped_signal[2],1000,2000);
     mapped_signal[3] = constrain(mapped_signal[3],1000,2000);
     
     motor[0].writeMicroseconds(mapped_signal[0]);
     motor[1].writeMicroseconds(mapped_signal[1]);
     motor[2].writeMicroseconds(mapped_signal[2]);
     motor[3].writeMicroseconds(mapped_signal[3]);
     

  /*  cal_pot = analogRead(15);
    
    mapped_signal[0] = map(cal_pot,0,1024,1000,2000);
    mapped_signal[1] = map(cal_pot,0,1024,1000,2000);
    mapped_signal[2] = map(cal_pot,0,1024,1000,2000);
    mapped_signal[3] = map(cal_pot,0,1024,1000,2000);
    
    motor[0].writeMicroseconds(mapped_signal[0]);
    motor[1].writeMicroseconds(mapped_signal[1]);
    motor[2].writeMicroseconds(mapped_signal[2]);
    motor[3].writeMicroseconds(mapped_signal[3]);*/
    //------ Echo to Screen if Defined ----------//
#ifdef ECHO
    
    if (IMU_online) {
        
        
        Serial.print("yaw: ");
        Serial.print(Attitude[2]);
        Serial.print("\t pitch: ");
        Serial.print(Attitude[1]);
        Serial.print("\t roll 3: ");
        Serial.print(Attitude[0]);
        
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


// IMU algorithm update
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}




//#endif
