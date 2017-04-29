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
#ifndef GYRO_ACCEL_H
#define GYRO_ACCEL_H

/*
  --------------------------------------------------------------------------------------------------
   GYRO AND ACCELEROMETER CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/

class MPU6050_CLASS
{
public:

  MPU6050_CLASS(void);   //class constructor
  
  double accel_x_scalled,accel_y_scalled,accel_z_scalled,gyro_x_scalled,gyro_y_scalled,gyro_z_scalled; //Scalled Data varaibles  
  double angle_x_gyro,angle_y_gyro,angle_z_gyro,angle_x_accel,angle_y_accel,angle_z_accel,angle_x,angle_y,angle_z;// angle variables
  void MPU6050_ReadData();
  void MPU6050_ResetWake();
  void MPU6050_SetDLPF(int BW);
  void MPU6050_SetGains(int gyro,int accel);
  void MPU6050_OffsetCal();
  void MPU6050_Set_PreCalibration_offset(void);

private:
 
  double accel_scale_fact, gyro_scale_fact;  // Scale factor variables 
  int  accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;  // Raw values varaibles 
  int accel_x_OC, accel_y_OC, accel_z_OC, gyro_x_OC ,gyro_y_OC, gyro_z_OC; // offset variables
  double temp_scalled;
  int read_bytes;
  byte gyro_byte,accel_byte;
  int x,y,z,i,temp;
};
#endif





