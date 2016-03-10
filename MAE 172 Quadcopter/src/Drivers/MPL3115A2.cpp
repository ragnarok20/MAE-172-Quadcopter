//
// MPL3115A2.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		MAE 172 Quadcopter
//
// Created by 	Sage Thayer, 3/9/16 12:47 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			MPL3115A2.h and ReadMe.txt for references
//


// Library header
#include "MPL3115A2.h"

// Code
//------------------------------------------------------------------------------//
//--------------------------MPL3115A2 Barometer Begin---------------------------//
//------------------------------------------------------------------------------//

// default constructor: will put into altimeter mode and polling mode
Barometer::Barometer() {
    its_ALT_MODE == true ;
    data_read = false;
    
}

// constructor overload, allow more choices
Barometer::Barometer(bool alt_mode) {
    // mode true will put in altitude mode otherwise in pressure mode
    // will go throught the power and initiate procedure indicated in the register: http://www.adafruit.com/datasheets/1893_datasheet.pdf
    its_ALT_MODE = alt_mode;
    data_read = false;

}

void Barometer::calibrate(float known_param) {
    // will go throught the power and initiate procedure indicated in the register: http://www.adafruit.com/datasheets/1893_datasheet.pdf
    
    //Set to Altimeter with an OSR = 128:
    I2C_Write(MPL3115A2_ADDRESS, BAR_CTRL_REG1, 0xB8);
    delay(70);
    //Enable Data Flags in PT_DATA_CFG
    I2C_Write(MPL3115A2_ADDRESS, BAR_PT_DATA_CFG, 0x07);
    delay(70);
    //Set Active
    I2C_Write(MPL3115A2_ADDRESS, BAR_CTRL_REG1, 0xB9);
    delay(70);
    // ready to poll now
    
    // our known parameter to calibrate in this case will be height we start at
    
    float buff[4];
    
    I2C_Write(MPL3115A2_ADDRESS, BAR_OFF_H, 0); //write altitude offset=0 (because calculation below is based on offset=0)
    
    for (byte i=0;i<4;i++){
        I2C_Write(MPL3115A2_ADDRESS, BAR_CTRL_REG1, 0b00111011); //bit 2 is one shot mode, bits 4-6 are 128x oversampling
        I2C_Write(MPL3115A2_ADDRESS, BAR_CTRL_REG1, 0b00111001); //must clear oversampling (OST) bit, otherwise update will be once per second
        delay(550); //wait for sensor to read pressure (512ms in datasheet)
        
        //buff[i] = this->read(); //read pressure
        buff[i] = Barometer::read(); //read pressure
        if (buff[i] == -555) {
            i =0;
        }
    }
    
    float currpress=(buff[0]+buff[1]+buff[2]+buff[3])/4; //average over two seconds
    
    //calculate pressure at mean sea level based on a given altitude
    //float seapress = currpress/pow(1-known_param*0.0000225577,5.255877);
    float seapress = currpress;
    
    // This configuration option calibrates the sensor according to
    // the sea level pressure for the measurement location (2 Pa per LSB)
    I2C_Write(MPL3115A2_ADDRESS, BAR_IN_MSB, (unsigned int)(seapress / 2)>>8);
    I2C_Write(MPL3115A2_ADDRESS, BAR_IN_LSB, (unsigned int)(seapress / 2)&0xFF);
    
}

float Barometer::read() {
    
    char msb, csb, lsb, temperature_msb, temperature_lsb ;
    
    
    //Set Active and take immediate measurement
    if (!data_read){
        //trigger OST to get an immediate measurement
        I2C_Write(MPL3115A2_ADDRESS, BAR_CTRL_REG1, 0b10111011);
        // clear OST
        I2C_Write(MPL3115A2_ADDRESS, BAR_CTRL_REG1, 0b10111001);
    }
        
        
    //Wait for PDR bit, indicates we have new pressure data
    if (I2C_Read(MPL3115A2_ADDRESS,BAR_STATUS) & 0x08 ) {
        //burst read registers
        // Read pressure registers
        Wire.beginTransmission(MPL3115A2_ADDRESS);
        Wire.write(BAR_OUT_P_MSB);  // Address of data to get
        Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
        Wire.requestFrom(MPL3115A2_ADDRESS, (int)3); // Request 3 bytes
            
        
        msb = Wire.read();
        csb = Wire.read();
        lsb = Wire.read();
            
            
        // The least significant bytes l_altitude and l_temp are 4-bit,
        // fractional values, so you must cast the calulation in (float),
        // shift the value over 4 spots to the right and divide by 16 (since
        // there are 16 values in 4-bits).
        float tempcsb = (float)(lsb>>4)/16.0;
        
        float altitude = (float)( (msb << 8) | csb) + tempcsb;
        itsAltitude = altitude;
            
        //data_read = true;
            
        return(itsAltitude);
    }
    else {
        return itsAltitude;  // error out
    }
    
    
}


/* ------------- I2C functions -------------------------------*/

uint8_t I2C_Read(uint8_t slaveAddr, uint8_t regAddr)
{
    // This function reads one byte over I2C
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);  // Address to read from
    Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
    Wire.requestFrom(slaveAddr, (uint8_t)1); // Request the data...
    return Wire.read();
}

void I2C_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t value)
{
    // This function writes one byte over I2C
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);
    Wire.write(value);
    Wire.endTransmission(true);
}


