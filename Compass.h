/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  University of Patras - Electronics and information processing Division of Electronics and Computers (Department of physics) 
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  Project       : Monakons Quadcopter
  File          : Compass_HMC5883L.ino
  Description   : This code test HMC5883L magnetometer sensor
  Author        : Monahopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  --------------------------------------------------------------------------------------------------
  COMPASS CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/

#ifndef COMPASS_H
  #define COMPASS_H

class HMC5883_CLASS{

public:

  HMC5883_CLASS(void); //class constructor
  void HMC5883_offset_calibration(int select);
  void HMC5883_init(int gain);
  void HMC5883_scalled_reading();
  void HMC5883_heading();
  void HMC5883_Set_PreCalibration_offset(void); 
  
  double bearing;
  double compass_x_scalled;
  double compass_y_scalled;
  double compass_z_scalled;
  
  double compass_x_offset, compass_y_offset, compass_z_offset;
  double compass_x_gainError,compass_y_gainError,compass_z_gainError;
  int compass_debug;
  double compass_gain_fact;
  
private:  

  int compass_x,compass_y,compass_z; 
  void HMC5883_read_XYZdata();

};  
  
#endif

