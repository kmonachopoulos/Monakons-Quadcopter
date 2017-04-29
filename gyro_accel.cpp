/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  University of Patras - Electronics and information processing Division of Electronics and Computers (Department of physics) 
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  Project       : Monakons Quadcopter
  File          : MPU_6050_Control.ino
  Description   : This code calibrates, test and debug MPU-6050 sensor
  Author        : Monachopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include <Wire.h>
#include "gyro_accel.h"

/*
  --------------------------------------------------------------------------------------------------
  DEFINITIONS
  --------------------------------------------------------------------------------------------------
*/

#define MPU6050_address 0x69

// ------ SELF Test Trim Factors ----------------------
#define MPU6050_self_test_x 13       // R/W
#define MPU6050_self_test_y 14       // R/W
#define MPU6050_self_test_z 15       // R/W
#define MPU6050_self_test_A 16       // R/W 

// ----- Sample Divider -------------------------------
#define MPU6050_sample_div 25        // R/W
/*       Sample Divider Discription
  Sample rate = 8/(1 + Sample rate divider) [kHz]  if DLPF is disabled
  Sample rate = 1/(1 + Sample rate divider) [kHz]  if DLPF is enabled
*/

// ----- Configration ---------------------------------
#define MPU6050_config 26            // R/W
#define MPU6050_gyro_config 27       // R/W
#define MPU6050_accel_config 28      // R/W

// ----- Data ---------------------------------------------
#define MPU6050_data_start 59

// ----- Power Management --------------------------------- 
#define MPU6050_PWR1 107
#define MPU6050_PWR2 108

// ----- Defining Constant --------------------------------
#define g 9.81                       // Gravitational accelration

/*
  --------------------------------------------------------------------------------------------------
  Function Prototyping
  --------------------------------------------------------------------------------------------------
*/

// ------------------------------------------------
//       Constructor function
// ------------------------------------------------

MPU6050_CLASS::MPU6050_CLASS(void){
  
accel_x_scalled,accel_y_scalled,accel_z_scalled,gyro_x_scalled,gyro_y_scalled,gyro_z_scalled=0;
angle_x_gyro,angle_y_gyro,angle_z_gyro,angle_x_accel,angle_y_accel,angle_z_accel,angle_x,angle_y,angle_z=0;
accel_x_OC, accel_y_OC, accel_z_OC, gyro_x_OC ,gyro_y_OC, gyro_z_OC,temp_scalled,read_bytes=0;
gyro_byte,accel_byte=0;
x,y,z,i,temp=0;
}

// ------------------------------------------------
//       Read Data From MPU6050
// ------------------------------------------------

void MPU6050_CLASS::MPU6050_ReadData(void){
  
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_data_start);
  Wire.endTransmission();

  read_bytes = 14;

  Wire.requestFrom(MPU6050_address,read_bytes);

  if(Wire.available() == read_bytes){
  
    accel_x = Wire.read()<<8 | Wire.read();
    accel_y = Wire.read()<<8 | Wire.read();
    accel_z = Wire.read()<<8 | Wire.read();
  
    temp = Wire.read()<<8 | Wire.read();
  
    gyro_x = Wire.read()<<8 | Wire.read();
    gyro_y = Wire.read()<<8 | Wire.read();
    gyro_z = Wire.read()<<8 | Wire.read(); 
  }
    
  accel_x_scalled = (double)(accel_x-accel_x_OC)*accel_scale_fact/1000; // divided by 1000 as the Scale factor is in milli units
  accel_y_scalled = (double)(accel_y-accel_y_OC)*accel_scale_fact/1000;
  accel_z_scalled = (double)(accel_z-accel_z_OC)*accel_scale_fact/1000;
  
  gyro_x_scalled = (double)(gyro_x-gyro_x_OC)*gyro_scale_fact/1000;
  gyro_y_scalled = (double)(gyro_y-gyro_y_OC)*gyro_scale_fact/1000;
  gyro_z_scalled = ((double)(gyro_z-gyro_z_OC)*gyro_scale_fact/1000);
  
  temp_scalled = (double)temp/340+36.53;
}

// ------------------------------------------------
//       Reset and wake up the MPU6050
// ------------------------------------------------

void MPU6050_CLASS::MPU6050_ResetWake(){
  
  Serial.println("Resetting MPU6050 and waking it up.....");
  Serial.println(" ");
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_PWR1);
  Wire.write(0b10000000);
  Wire.endTransmission();
  
  delay(100); // Waiting for the reset to complete
   
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_PWR1);
 
  Wire.write(0b00000000);
  Wire.endTransmission(); 
}

// ------------------------------------------------
//       Setting up the DLFP (LOW PASS FILTER)
// ------------------------------------------------

void MPU6050_CLASS::MPU6050_SetDLPF(int BW){
  
     if (BW < 0 || BW > 6){
      BW = 0;
    }
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_config); // Address to the configuration register
/*       config Discription ---- x x 0 0 0 F2 F1 F0 
  I am only intrested in the Digital Low Pass Filter (DLPF)
  F2 F1 F0    Bandwidth [Hz]
  0  0  0        
  0  0  1      184
  0  1  0      94
  0  1  1      44
  1  0  0      21  
  1  0  1      10  
  1  1  0      5
*/

  Wire.write(BW);
  Wire.endTransmission(); 
  
}

// ------------------------------------------------
//       Setting up the Accelrometer and Gyro Gains
// ------------------------------------------------  

void MPU6050_CLASS::MPU6050_SetGains(int gyro,int accel){
  
// Setting up Gyro
    Wire.beginTransmission(MPU6050_address);
    Wire.write(MPU6050_gyro_config); // Address to the configuration register
    if (gyro==0)
    {
      gyro_scale_fact =(double)250*0.0305; // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00000000;
    }else if (gyro == 1)
    {
      gyro_scale_fact = 500*0.0305; // each data is of 16 bits that means, 500 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00001000;
    }else if (gyro == 2)
    {
      gyro_scale_fact = 1000*0.0305;// each data is of 16 bits that means, 1000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00010000;
    }else if (gyro == 3)
    {
      gyro_scale_fact = 2000*0.0305;  // each data is of 16 bits that means, 2000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00011000;
    }else
    {
      gyro_scale_fact = 1;
    }  
      
    Wire.write(gyro_byte);
    Wire.endTransmission();
    
    Serial.print("The gyro scale is set to ");
    Serial.print(gyro_scale_fact);
    Serial.println(" milli Degree/s");
    Serial.println(" ");
     
    // Setting up Accel
    Wire.beginTransmission(MPU6050_address);
    Wire.write(MPU6050_accel_config); // Address to the configuration register
    if (accel==0)
    {
      accel_scale_fact =(double)2*g*0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767       
      accel_byte = 0b00000000;
    }else if (accel == 1)
    {
      accel_scale_fact = 4*g*0.0305; // each data is of 16 bits that means, 4g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      accel_byte = 0b00001000;
    }else if (accel == 2)
    {
      accel_scale_fact = 8*g*0.0305;// each data is of 16 bits that means, 8g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      accel_byte = 0b00010000;
    }else if (accel == 3)
    {
      accel_scale_fact = 16*g*0.0305; // each data is of 16 bits that means, 16g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      accel_byte = 0b00011000;
    }else
    {
      accel_scale_fact = 1;
    }  
      
    Wire.write(accel_byte);
    Wire.endTransmission();
    
    Serial.print("The accel scale is set to ");
    Serial.print(accel_scale_fact);
    Serial.println(" milli m/s^2");
    Serial.println(" ");
    
}

// ------------------------------------------------
//       offset calibration
// ------------------------------------------------  

void MPU6050_CLASS::MPU6050_OffsetCal(){
  
 Serial.println("Calibrating gyroscope .... dont move the hardware ..........");
 Serial.println(" ");
  
  x=0,y=0,z=0,i;
  
  MPU6050_ReadData(); //First time Reads Zeros
  MPU6050_ReadData(); //Second time Reads Trash
  
  // Gyro Offset Calculation
  x=gyro_x;
  y=gyro_y;
  z=gyro_z;
  
  // Calculate mean value from 1000 gyro values  
  for (i=1;i<=1000;i++){
    MPU6050_ReadData();
    x=(x+gyro_x)/2;
    y=(y+gyro_y)/2;
    z=(z+gyro_z)/2;
  }

  gyro_x_OC=x;
  gyro_y_OC=y;
  gyro_z_OC=z;
  
  Serial.print("gyro_x register offset = ");
  Serial.println(x);
 
  
  Serial.print("gyro_y register offect = ");
  Serial.println(y);
 
  
  Serial.print("gyro_z register offset = ");
  Serial.println(z);
  Serial.println(" ");
  
  // Accel Offset Calculation
  Serial.println("Calibrating accelrometer .... dont move the hardware ..........");
  x=accel_x;
  y=accel_y;
  z=accel_z;
  
  // Calculate mean accel from 1000 accel values      
  for (i=1;i<=1000;i++){
    MPU6050_ReadData();
    x=(x+accel_x)/2;
    y=(y+accel_y)/2;
    z=(z+accel_z)/2;
  }
  Serial.println(" ");
  accel_x_OC=x;
  accel_y_OC=y;
  accel_z_OC=z-(double)g*1000/accel_scale_fact;
  
  Serial.print("Accel_x register offset = ");
  Serial.println(x);
  
  Serial.print("Accel_y register offect = ");
  Serial.println(y);
  
  Serial.print("Accel_z register offset = ");
  Serial.println(z);
  
  Serial.println(" ");
  
}

// ------------------------------------------------
//       offset Pre-Calibrated Values
// ------------------------------------------------

void MPU6050_CLASS::MPU6050_Set_PreCalibration_offset(void){ 

 
  gyro_x_OC=46;
  gyro_y_OC=-81;
  gyro_z_OC=-9;
 
  accel_x_OC=-96;
  accel_y_OC=-305;
  accel_z_OC=8648-(double)g*1000/accel_scale_fact;
  
 /*
 
  gyro_x_OC=23;
  gyro_y_OC=-100;
  gyro_z_OC=-16;
 
  accel_x_OC=-149;
  accel_y_OC=-153;
  accel_z_OC=8097-(double)g*1000/accel_scale_fact;
  */
  }
