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
  Author        : Monahopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/
#include <Arduino.h>
#include "debug.h"

/*
  --------------------------------------------------------------------------------------------------
  Function Prototyping
  --------------------------------------------------------------------------------------------------
*/

// ------------------------------------------------
// Print Data To Serial for Matlab Data acquisition
// ------------------------------------------------
void DEBUG_CLASS::Debug_XYZ_matlab_Print(double XYZ_matlab_data_logging[]){
  
  Debug_printDouble(XYZ_matlab_data_logging[0],100);
  Serial.print(" ");
  
  Debug_printDouble(XYZ_matlab_data_logging[1],100);
  Serial.print(" ");
  
  Debug_printDouble(XYZ_matlab_data_logging[2],100);
  Serial.println(" ");
}

// ------------------------------------------------
// Prints into Serial Double Values
// ------------------------------------------------
void DEBUG_CLASS::Debug_printDouble( double val, unsigned int precision){
  
  /* prints val with number of decimal places determine by precision
   NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
   example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)*/

    Serial.print (int(val));  //prints the int part
    Serial.print("."); // print the decimal point
    
    if(val >= 0)
      frac = (val - int(val)) * precision;
    else
       frac = (int(val)- val ) * precision;
       frac1 = frac;
    while( frac1 /= 10 )
        precision /= 10;
    precision /= 10;
    while(  precision /= 10)
        Serial.print("0");

    Serial.print(frac,DEC) ;
}


