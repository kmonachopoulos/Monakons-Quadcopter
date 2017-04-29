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

/*
  --------------------------------------------------------------------------------------------------
   DEBUG CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/

#ifndef DEBUG_H
  #define DEBUG_H
  
  class DEBUG_CLASS
{
    
    public:
    double SensorArray[3]; 
    DEBUG_CLASS() : SensorArray(),frac(0),frac1(0) { }  //class constructor
    void Debug_XYZ_matlab_Print(double XYZ_matlab_data_logging[]); 
    void Debug_printDouble( double val, unsigned int precision);
    
    private:
    unsigned int frac;
    int frac1;  
};

#endif
