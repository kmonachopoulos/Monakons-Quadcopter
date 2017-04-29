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
  Author        : Monachopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include "Motors.h"

/*
  --------------------------------------------------------------------------------------------------
  DEFINITIONS
  --------------------------------------------------------------------------------------------------
*/

#define MAX_SIGNAL 2000 // maximum us at esc 
#define MIN_SIGNAL 800  // minimum us at esc 

/*
  --------------------------------------------------------------------------------------------------
  Function Prototyping
  --------------------------------------------------------------------------------------------------
*/

// ------------------------------------------------
//       Constructor function
// ------------------------------------------------
MOTORS_CLASS::MOTORS_CLASS(void){

  MotorSpeed[0]=1000; 
  MotorSpeed[1]=1000; 
  MotorSpeed[2]=1000; 
  MotorSpeed[3]=1000; 
}

// ------------------------------------------------
// Set Speed to the motors defined by the PID
// ------------------------------------------------
void MOTORS_CLASS::setSpeed(int MotorFLCurretnspeed,int MotorFRCurretnspeed,int MotorRLCurretnspeed,int MotorRRCurretnspeed,double pitch,double roll,double yaw,double altitude,Servo &MotorFL ,Servo &MotorFR, Servo &MotorRL,Servo &MotorRR){ 

  /* The desired roll or pitch force will
    be translated into the thrust ratio between the 2 motors that can effect that force.

    // Without Yaw
    MotorFL.writeMicroseconds(round(MotorFLCurretnspeed - roll - pitch ));   
    MotorFR.writeMicroseconds(round(MotorFRCurretnspeed + roll - pitch ));
    MotorRL.writeMicroseconds(round(MotorRLCurretnspeed - roll + pitch ));
    MotorRR.writeMicroseconds(round(MotorRRCurretnspeed + roll + pitch ));
    */
   /*  
  Serial.print(" FL : ");Serial.print(round(MotorFLCurretnspeed - roll));
  Serial.print(" RL : ");Serial.print(round(MotorFLCurretnspeed - roll));   
  Serial.print(" FR : ");Serial.print(round(MotorFLCurretnspeed + roll));
  Serial.print(" RR : ");Serial.println(round(MotorFLCurretnspeed + roll));
*/
  // With Yaw      
    MotorFL.writeMicroseconds(round(MotorFLCurretnspeed - roll - pitch - yaw));   
    MotorFR.writeMicroseconds(round(MotorFRCurretnspeed + roll - pitch + yaw));
    MotorRL.writeMicroseconds(round(MotorRLCurretnspeed - roll + pitch + yaw));
    MotorRR.writeMicroseconds(round(MotorRRCurretnspeed + roll + pitch - yaw));

}

// ------------------------------------------------
// Send MIN_SIGNAL to esc for ARMing Procedure 
// ------------------------------------------------
void MOTORS_CLASS::arm(Servo &MotorFL ,Servo &MotorFR, Servo &MotorRL,Servo &MotorRR){
  MotorFL.writeMicroseconds(MIN_SIGNAL);
  MotorFR.writeMicroseconds(MIN_SIGNAL);
  MotorRL.writeMicroseconds(MIN_SIGNAL);
  MotorRR.writeMicroseconds(MIN_SIGNAL);
  delay(2000); 
}







