/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  University of Patras - Electronics and information processing Division of Electronics and Computers (Department of physics) 
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  Project       : Monakons Quadcopter
  File          : Motors_Control.ino
  Description   : This code defines Motor Speed process
  Author        : Monahopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/

#include <Servo.h> 

/*
  --------------------------------------------------------------------------------------------------
  DEFINITIONS
  --------------------------------------------------------------------------------------------------
*/

#ifndef MOTORS_H
  #define MOTORS_H

/*
  --------------------------------------------------------------------------------------------------
   MOTOR CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/

  class MOTORS_CLASS
{
    
    public:
    MOTORS_CLASS(void); //class constructor
    void setSpeed(int MotorFLCurretnspeed,int MotorFRCurretnspeed,int MotorRLCurretnspeed,int MotorRRCurretnspeed,double pitch,double roll,double yaw,double altitude,Servo &MotorFL ,Servo &MotorFR, Servo &MotorRL,Servo &MotorRR);
    void arm(Servo &MotorFL ,Servo &MotorFR, Servo &MotorRL,Servo &MotorRR);
    int MotorSpeed[3];
    
    private:
    int MotorSpeedPitch;
    int MotorSpeedRoll;
    
};
 
#endif

