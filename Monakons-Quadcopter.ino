/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  University of Patras - Electronics and information processing Division of Electronics and Computers (Department of physics) 
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  Project       : Monakons Quadcopter
  File          : Monakons_QuadCopter.ino
  Description   : This code calibrates, test and debug the Quadcopter
  Author        : Monahopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
// Structure:
// ----------
//  Arduino  |               ---------------- 
//  UNO      |- 3.3 V ------ | MPU 6050     |
//           |- GND ---------| HMC5883,     |
//           |- SDA ---------| MS5611       |
//           |- SCL ---------|              |
//           |               ---------------- 
//-----------
//                       |__________ _____________|
//                                  V
//                           Integrated IMU sensor 


// MPU-6050 - Gyro [ITG3200] / Accel [BMA180]   - address (0x69)
// HMC5883  - Magnetometer - address (0x1E)
// MS5611   - Barometer    - address (0x77)

/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/

#include <Wire.h>
#include <Servo.h> 
#include "gyro_accel.h"
#include "debug.h"
#include "Motors.h"
#include "Compass.h"
#include "MS5611.h"
#include "PID.h"


/*
  --------------------------------------------------------------------------------------------------
  DEFINITIONS
  --------------------------------------------------------------------------------------------------
*/

/* DEFINE MPU6050_CALIBRATION TO CALIBRATE QUAD ON INITIALIZATION */ 
//#define MPU6050_CALIBRATION 

/* DEFINE MPU6050_CALIBRATION TO USE PRECALIBRATION OFFSET QUAD ON INITIALIZATION */
#define MPU6050_PRECALIBRATION

/* DEFINE HMC5883_CALIBRATION TO CALIBRATE QUAD ON INITIALIZATION */ 
//#define HMC5883_CALIBRATION 

/* DEFINE HMC5883_CALIBRATION TO USE PRECALIBRATION OFFSET QUAD ON INITIALIZATION */
#define HMC5883_PRECALIBRATION

/* DEFINE MATLAB_DATA_LOGGING TO DEBUG SENSOR VALUES TO MATLAB */
//#define MATLAB_DATA_LOGGING

/* DEFINE DEBUG TO DEBUG SENSOR VALUES TO ARDUINO */
//#define DEBUG

/* DEFINE PS2 CONTROLLER CONFIRMATION */
#define PS2_CONTROLLER

/* DEFINE PID COMPUTATION */
#define PID

/* DEFINE MOTORS OUTPUT */
#define MOTORS

#define rad2degree 57.3              // Radian to degree conversion

/* Lowering the Filter Gain means using  more the accelerometer.*/

#define Filter_gain_Gyro_Accel 0.90         // Gyroscope - Accelerometer trust (For Kalman filters)
#define Filter_gain_Gyro_Compass 0.90       // Gyroscope - Accelerometer trust (For Kalman filters)
#define MAX_SIGNAL 2000                     // maximum us at esc 
#define MIN_SIGNAL 900                      // minimum us at esc
#define MAX_AFRO_PWM 1800                   // From Afro esc manual 1860 (max)
#define MIN_AFRO_PWM 1060                   // From Afro esc manual 1060 (min)
#define MIN_PID_ENABLE 1200                 // minimum value to enable the PID controller 
#define PID_LED 8                           // Led in pin 8
#define PS2_STATUS_LED 12                   // Led in pin 12
#define SENSORS_STATUS_LED 13               // Led in pin 13
#define MIN_YAW_Deg 20                      // minimum degrees at yaw 
#define MAX_YAW_Deg 340                     // maximum degrees at yaw
#define MIN_MAX_ROLL_PITCH_Deg 85           // minimum - maximum degrees at ROLL - PITCH 
//#define dt 10                               // time diffrence in millis seconds - (loop time = 10  ms = 100 hz)
#define TrnPwr 3                            // How much you will the ps controller gives you in every press
/*
  --------------------------------------------------------------------------------------------------
  Global Variables
  --------------------------------------------------------------------------------------------------
*/
unsigned long dt=0;   
unsigned long t;                           // Time Variable
unsigned int PSControllerStatus=0;         // Status flag
String StringCommand;                      // Command from controller
int ServoValuesLR=90;
int ServoValuesUD=90;
String Acrobatic="OFF";                    // Acrobatic Mode is OFF at initialization
String AltitudeHold="OFF";                 // Altitude Hold is OFF at initialization
String QuadAtGround="ON";                  // While Quadcopter is still at ground QuadAtGround is ON
int led_count=1;                           // Show off with the leds waiting PS command :)
/*
  --------------------------------------------------------------------------------------------------
  Assign Classes
  --------------------------------------------------------------------------------------------------
*/ 

MPU6050_CLASS MPU6050obj;                    // Mpu-6050 Object
HMC5883_CLASS HMC5883obj;                    // HMC5883 Object
MS5611_CLASS MS5611obj;                      // MS5611 Object
DEBUG_CLASS DEBUGobj;                        // Debug Object
MOTORS_CLASS MOTORSobj;                      // Motors Object

// ------------------------------------------------
// PID Configuration
// ------------------------------------------------

/* Set PID values for Pitch (front - back) - Roll (left - right) - Yaw (Uturn left - Uturn right) */
PID_INFO PID_ROLL_info(0.859,0.0,0.319); //(0.859,0.618,0.319)
PID_INFO PID_PITCH_info(0.0,0.0,0.0); //(0.959,0.747,0.300)
PID_INFO PID_YAW_info(0.0,0.0,0.0); //(0.35,0.000,0.015)
PID_INFO PID_ALTITUDE_info(0.000,0.000,0.000);

/* Assign Pointer variables that will be constantlly updated in main loop*/
PID_EXEC PID_ROLL_exec(PID_ROLL_info.kp,PID_ROLL_info.ki,PID_ROLL_info.kd, DIRECT );
PID_EXEC PID_PITCH_exec(PID_PITCH_info.kp,PID_PITCH_info.ki,PID_PITCH_info.kd, DIRECT);
PID_EXEC PID_YAW_exec(PID_YAW_info.kp,PID_YAW_info.ki,PID_YAW_info.kd, DIRECT);
PID_EXEC PID_ALTITUDE_exec(PID_ALTITUDE_info.kp,PID_ALTITUDE_info.ki,PID_ALTITUDE_info.kd, DIRECT);

    

Servo myescFL;                             // Servo Front Left Object
Servo myescFR;                             // Servo Front Right Object
Servo myescRL;                             // Servo Rear Left Object
Servo myescRR;                             // Servo Rear Right Object 
Servo CameraServoLR;                       // Camera Left - Right Object
Servo CameraServoUD;                       // Camera Up - Down Object

/*
  --------------------------------------------------------------------------------------------------
  Main Code
  --------------------------------------------------------------------------------------------------
*/ 

void setup(){
  

  Serial.begin(9600);  
  Wire.begin();
  
 
// ------------------------------------------------
//  Camera Mount Configuration
// ------------------------------------------------

  CameraServoLR.attach(11);                // Attach camera pins
  CameraServoUD.attach(10);                // Attach camera pins
  CameraServoLR.write(ServoValuesLR);      // Initiate camera servo 1
  CameraServoUD.write(ServoValuesUD);      // Initiate camera servo 2

// ------------------------------------------------
//  Motor Configuration
// ------------------------------------------------  
  
  myescFL.attach(3);                       // Attach motor pins to esc
  myescFR.attach(5);                       // Attach motor pins to esc
  myescRL.attach(6);                       // Attach motor pins to esc
  myescRR.attach(9);                       // Attach motor pins to esc

  MOTORSobj.arm(myescFL,myescFR,myescRL,myescRR); // Arm the esc

// ------------------------------------------------
//  Led Configuration
// ------------------------------------------------ 

   pinMode(PS2_STATUS_LED, OUTPUT);             // Attach Led pin
   pinMode(SENSORS_STATUS_LED, OUTPUT);         // Attach Led pin
   pinMode(PID_LED, OUTPUT);                    // Attach Led pin
   
// ------------------------------------------------
//  MPU6050 Configuration
// ------------------------------------------------ 
 
  MPU6050obj.MPU6050_ResetWake();            // Wake up MPU - 6050
  MPU6050obj.MPU6050_SetGains(0,1);          // Setting the lows scale
  MPU6050obj.MPU6050_SetDLPF(0);             // Setting the DLPF to inf Bandwidth for calibration

// ------------------------------------------------
// HMC5883 Configuration
// ------------------------------------------------ 

  HMC5883obj.HMC5883_init(2);                 // Wake up HMC5883  */
  
  Serial.print("Compass Gain updated to  = ");
  Serial.print(HMC5883obj.compass_gain_fact);
  Serial.println(" mG/bit");
  Serial.println(" "); 

// ------------------------------------------------
// MS5611 Configuration
// ------------------------------------------------ 
  
  MS5611obj.init(MS561101BA_ADDR_CSB_LOW);

// ------------------------------------------------
// PID Configuration
// ------------------------------------------------ 
PID_ROLL_exec.SetMode(MANUAL);
PID_PITCH_exec.SetMode(MANUAL);
PID_YAW_exec.SetMode(MANUAL);
PID_ALTITUDE_exec.SetMode(MANUAL); 

// ------------------------------------------------
// Definitions Configuration
// ------------------------------------------------ 

#ifdef MPU6050_CALIBRATION
   MPU6050obj.MPU6050_OffsetCal(); 
#endif

#ifdef MPU6050_PRECALIBRATION  
    MPU6050obj.MPU6050_Set_PreCalibration_offset();
#endif

#ifdef HMC5883_CALIBRATION 
    HMC5883obj.compass_debug = 1;
    HMC5883obj.HMC5883_offset_calibration(3);
#endif

#ifdef HMC5883_PRECALIBRATION  
    HMC5883obj.HMC5883_Set_PreCalibration_offset();    
#endif

MPU6050obj.MPU6050_SetDLPF(6);                      // Set the DLPF decreasing the bandwidth by increasing this number. [MAX BW,MIN BW] = [0,6]

// ------------------------------------------------
// Set Initialization setpoint for Compass
// ------------------------------------------------

  //HMC5883obj.HMC5883_scalled_reading();
  HMC5883obj.HMC5883_heading(); 

  Serial.print("Initialization Yaw in degrees = ");
  Serial.println(HMC5883obj.bearing);
  Serial.println(" ");
  
 /* Check initial degrees at StartUp */ 
 while(HMC5883obj.bearing <= MIN_YAW_Deg || HMC5883obj.bearing >= MAX_YAW_Deg){ 
   
    HMC5883obj.HMC5883_heading();
    digitalWrite(SENSORS_STATUS_LED, HIGH);
    delay(150); 
    digitalWrite(SENSORS_STATUS_LED, LOW);
    delay(150); 
    Serial.print("Problem at initialization process in degrees = ");
    Serial.println(HMC5883obj.bearing);  
  
 } // Exit if degrees if between min and max deegres
 
   PID_YAW_info.Setpoint = HMC5883obj.bearing;        // Save the degrees as setpoint for yaw
   
// ------------------------------------------------
// Sensor Configuration Status
// ------------------------------------------------

 digitalWrite(SENSORS_STATUS_LED, HIGH);            // All Sensors are ready to use !
 delay(1000);
 digitalWrite(SENSORS_STATUS_LED, LOW);

// ------------------------------------------------
// PS2 controller identifier
// ------------------------------------------------
/* Check correct initialization for PS2 contoller*/

#ifdef PS2_CONTROLLER
  while (1)
  {    
      //Serial.println(" Waiting PS2 Controller configuration.. ");
      if (led_count==1){
       digitalWrite(PS2_STATUS_LED, HIGH);
      delay(100); 
      }else if(led_count==2){
       digitalWrite(PID_LED, HIGH);
       delay(100); 
      }else if(led_count==3){
       digitalWrite(SENSORS_STATUS_LED, HIGH);
       delay(100);
      led_count=0;
      }
         led_count+=1;
      
          if(Serial.available() > 0){     
          PSControllerStatus = Serial.parseInt();  
          
          if(PSControllerStatus == true){
          digitalWrite(PS2_STATUS_LED, HIGH);
          digitalWrite(PID_LED, HIGH);
          digitalWrite(SENSORS_STATUS_LED, HIGH);          
          delay(1000);
          digitalWrite(PS2_STATUS_LED, LOW);
          digitalWrite(PID_LED, LOW);
          digitalWrite(SENSORS_STATUS_LED, LOW);  
          break;
         }
       } 
   digitalWrite(PS2_STATUS_LED, LOW);
   digitalWrite(PID_LED, LOW);
   digitalWrite(SENSORS_STATUS_LED, LOW);
   delay(100);  
  } 
#endif




// ------------------------------------------------
// PID meta_info
// ------------------------------------------------

/* Start PID clocks for the first execution */ 
 PID_ROLL_exec.lastTime = millis()-PID_ROLL_exec.SampleTime;
 PID_PITCH_exec.lastTime = millis()-PID_PITCH_exec.SampleTime;
 PID_YAW_exec.lastTime = millis()-PID_YAW_exec.SampleTime;
 PID_ALTITUDE_exec.lastTime = millis()-PID_ALTITUDE_exec.SampleTime;
}


void loop(){ 
  
  /* AT THE BEGINNING FOR EVERY #number LOOPS I MUST CHECK THE STRENGTH OF THE XBEE RANGE. IF STRENGTH IS LOWER THAN A VALUE ... 
  LOWER THE MOTOR SPEED UNTIL SIGNALS EQUALS TO ...
  */
  
  t=millis(); //Number of milliseconds since the program started (unsigned long), This number will overflow (go back to zero), after approximately 50 days.
/*
  --------------------------------------------------------------------------------------------------
  MAIN PROCESS
  --------------------------------------------------------------------------------------------------
*/   

/*
  --------------------------------------------------------------------------------------------------
  MPU - 6050 implementation
  --------------------------------------------------------------------------------------------------
*/ 

  MPU6050obj.MPU6050_ReadData();

// ------------------------------------------------
//       Read Gyro Values From MPU6050
// ------------------------------------------------  

  MPU6050obj.angle_x_gyro = (MPU6050obj.gyro_x_scalled*((double)dt/1000)+MPU6050obj.angle_x);
  MPU6050obj.angle_y_gyro = (MPU6050obj.gyro_y_scalled*((double)dt/1000)+MPU6050obj.angle_y);
  MPU6050obj.angle_z_gyro = (MPU6050obj.gyro_z_scalled*((double)dt/1000)+MPU6050obj.angle_z);

// ------------------------------------------------
//       Read Accelerometer Values From MPU6050
// ------------------------------------------------ 

  MPU6050obj.angle_x_accel = atan(MPU6050obj.accel_y_scalled /(sqrt(MPU6050obj.accel_x_scalled*MPU6050obj.accel_x_scalled+MPU6050obj.accel_z_scalled*MPU6050obj.accel_z_scalled)))*(double)rad2degree;
  MPU6050obj.angle_y_accel = -atan(MPU6050obj.accel_x_scalled/(sqrt(MPU6050obj.accel_y_scalled*MPU6050obj.accel_y_scalled+MPU6050obj.accel_z_scalled*MPU6050obj.accel_z_scalled)))*(double)rad2degree;
  MPU6050obj.angle_z_accel = atan(MPU6050obj.accel_z_scalled/(sqrt(MPU6050obj.accel_y_scalled*MPU6050obj.accel_y_scalled+MPU6050obj.accel_x_scalled*MPU6050obj.accel_x_scalled)))*(double)rad2degree;

// ------------------------------------------------
//       Compute Overall Angle
// ------------------------------------------------ 

  MPU6050obj.angle_x = Filter_gain_Gyro_Accel*MPU6050obj.angle_x_gyro+(1-Filter_gain_Gyro_Accel)*MPU6050obj.angle_x_accel;
  MPU6050obj.angle_y = Filter_gain_Gyro_Accel*MPU6050obj.angle_y_gyro+(1-Filter_gain_Gyro_Accel)*MPU6050obj.angle_y_accel;
  MPU6050obj.angle_z = Filter_gain_Gyro_Accel*MPU6050obj.angle_z_gyro+(1-Filter_gain_Gyro_Accel)*MPU6050obj.angle_z_accel;

/*
  --------------------------------------------------------------------------------------------------
  HMC5883 implementation
  --------------------------------------------------------------------------------------------------
*/   

  HMC5883obj.HMC5883_heading();

  if (HMC5883obj.bearing > MAX_YAW_Deg ){
  HMC5883obj.bearing =MAX_YAW_Deg;
  }else if(HMC5883obj.bearing < MIN_YAW_Deg ){
  HMC5883obj.bearing = MIN_YAW_Deg;
  }  
 
/*
  --------------------------------------------------------------------------------------------------
  MS5611 implementation
  --------------------------------------------------------------------------------------------------
*/ 

  MS5611obj.temperature = MS5611obj.getTemperature(MS561101BA_OSR_4096);
  MS5611obj.pression = MS5611obj.getPressure(MS561101BA_OSR_4096); 
  MS5611obj.Absolute_Altitude=MS5611obj.getAltitude(MS5611obj.pression, MS5611obj.temperature);

/*
  --------------------------------------------------------------------------------------------------
  PID implementation
  --------------------------------------------------------------------------------------------------
*/

#ifdef PID

/* if quadcopter bocomes airborn enable PID*/
if ( (MOTORSobj.MotorSpeed[0]+ MOTORSobj.MotorSpeed[1]+ MOTORSobj.MotorSpeed[2]+ MOTORSobj.MotorSpeed[3])/4  > MIN_PID_ENABLE && QuadAtGround.equals("ON")){
        digitalWrite(PID_LED, HIGH);  
        PID_ROLL_exec.SetMode(AUTOMATIC);
        PID_PITCH_exec.SetMode(AUTOMATIC);
        PID_YAW_exec.SetMode(AUTOMATIC);
        PID_ALTITUDE_exec.SetMode(AUTOMATIC); 
        QuadAtGround="OFF";
        
}else if ( (MOTORSobj.MotorSpeed[0]+ MOTORSobj.MotorSpeed[1]+ MOTORSobj.MotorSpeed[2]+ MOTORSobj.MotorSpeed[3])/4  < MIN_PID_ENABLE && QuadAtGround.equals("OFF")){
        digitalWrite(PID_LED, LOW); 
        PID_ROLL_exec.SetMode(MANUAL);
        PID_PITCH_exec.SetMode(MANUAL);
        PID_YAW_exec.SetMode(MANUAL);
        PID_ALTITUDE_exec.SetMode(MANUAL); 
        QuadAtGround="ON"; 
}
/* Compute PID Output only if Quad left the ground*/

PID_ROLL_info.PIDOutput=PID_ROLL_exec.Compute(MPU6050obj.angle_y,PID_ROLL_info.Setpoint);
PID_PITCH_info.PIDOutput=PID_PITCH_exec.Compute(MPU6050obj.angle_x,PID_PITCH_info.Setpoint);
PID_YAW_info.PIDOutput=PID_YAW_exec.Compute(HMC5883obj.bearing,PID_YAW_info.Setpoint);
PID_ALTITUDE_info.PIDOutput=PID_ALTITUDE_exec.Compute(MS5611obj.Absolute_Altitude,PID_ALTITUDE_info.Setpoint);

#endif

// ------------------------------------------------
// Motors implementation
// ------------------------------------------------

#ifdef MOTORS

/* PID Output to Motors */

MOTORSobj.setSpeed(MOTORSobj.MotorSpeed[0],MOTORSobj.MotorSpeed[1],MOTORSobj.MotorSpeed[2],MOTORSobj.MotorSpeed[3],PID_PITCH_info.PIDOutput,PID_ROLL_info.PIDOutput,PID_YAW_info.PIDOutput,PID_ALTITUDE_info.PIDOutput,myescFL,myescFR,myescRL,myescRR);  

#endif


// ------------------------------------------------
// READ CONTROLLER COMMAND
// ------------------------------------------------

  if(Serial.available() > 0)
  {
    
    digitalWrite(PS2_STATUS_LED, HIGH);                   // Turn Led on
    delay(2);                                             // Watch out this delay .. 
    digitalWrite(PS2_STATUS_LED, LOW);                    // Turn Led off 
    StringCommand = Serial.readStringUntil('\n');         // Fetch Command
    
// ------------------------------------------------
// TURN CAMERA LEFT
// ------------------------------------------------ 

  if(StringCommand.equals("L1") && ServoValuesLR<160){
  ServoValuesLR+=5;
  CameraServoLR.write(ServoValuesLR); }
  
// ------------------------------------------------
// TURN CAMERA RIGHT
// ------------------------------------------------  

  if(StringCommand.equals("R1")&& ServoValuesLR>20){
    ServoValuesLR-=5;
    CameraServoLR.write(ServoValuesLR);       
  }
   
// ------------------------------------------------
// TURN CAMERA DOWN
// ------------------------------------------------ 

  if(StringCommand.equals("L2") && ServoValuesUD<160){
    ServoValuesUD+=5;
    CameraServoUD.write(ServoValuesUD);           
  }
  
// ------------------------------------------------
// TURN CAMERA UP
// ------------------------------------------------ 

  if(StringCommand.equals("R2")&& ServoValuesUD>20){
    ServoValuesUD-=5;
    CameraServoUD.write(ServoValuesUD);           
  }
  
// ------------------------------------------------
// ACROBATIC ON - Not supported yet
// ------------------------------------------------   

  if(StringCommand.equals("SELECT") && Acrobatic.equals("OFF")){
    Acrobatic="ON";
    StringCommand="VOID"; // Set command to VOID to avoid entering to another Mode
    
  /* Set the Kp,Ki,Kd of PID_PITCHobj and PID_ROLLobj to big values
  for AGGRESSIVE maneuvers OR change the step of angle set. Led on acrobatic led*/   
  }
  
// ------------------------------------------------
// ACROBATIC OFF - Not supported yet
// ------------------------------------------------       

  if(StringCommand.equals("SELECT") && Acrobatic.equals("ON")){
    Acrobatic="OFF";
    StringCommand="VOID"; // Set command to VOID to avoid enter to another Mode
           
  /* Set the Kp,Ki,Kd of PID_PITCHobj and PID_ROLLobj to small values
  for SMOOTH maneuvers OR change the step of angle set. Led off acrobatic led*/     
  }

// ------------------------------------------------
// EMERGENCY STOP
// ------------------------------------------------  

  if(StringCommand.equals("START")){ 
    
  while(1){
  MOTORSobj.setSpeed(0,0,0,0,0,0,PID_YAW_info.PIDOutput,PID_ALTITUDE_info.PIDOutput,myescFL,myescFR,myescRL,myescRR);  

  digitalWrite(PS2_STATUS_LED, HIGH);
  }        
}   

// ------------------------------------------------
// ALTITUDE UP
// ------------------------------------------------  

  if(StringCommand.equals("UP")){
   /* Maximum speed in writeMicroseconds is 1860, 
      MAX CORRECTION IS 60, 
      SETTING THE GAP TO 60, 
      1860(max) - Gap = 1800*/
 
    if  (MOTORSobj.MotorSpeed[0]>MAX_AFRO_PWM && MOTORSobj.MotorSpeed[1] >MAX_AFRO_PWM && MOTORSobj.MotorSpeed[2] >MAX_AFRO_PWM && MOTORSobj.MotorSpeed[3] >MAX_AFRO_PWM){
    MOTORSobj.MotorSpeed[0]=MAX_AFRO_PWM;
    MOTORSobj.MotorSpeed[1]=MAX_AFRO_PWM;
    MOTORSobj.MotorSpeed[2]=MAX_AFRO_PWM;
    MOTORSobj.MotorSpeed[3]=MAX_AFRO_PWM;    
    }else{
      
   /* Scale 0 - 255 to 0-25 and add it to the current motor speed*/
   MOTORSobj.MotorSpeed[0]+=round(5);
   MOTORSobj.MotorSpeed[1]+=round(5);
   MOTORSobj.MotorSpeed[2]+=round(5);
   MOTORSobj.MotorSpeed[3]+=round(5);
    }
  }        
  
// ------------------------------------------------
// ALTITUDE DOWN
// ------------------------------------------------   

  if(StringCommand.equals("DOWN")){
  
   /* min speed in writeMicroseconds is 1000 */
    if  (MOTORSobj.MotorSpeed[0]<MIN_AFRO_PWM && MOTORSobj.MotorSpeed[1] <MIN_AFRO_PWM && MOTORSobj.MotorSpeed[2] <MIN_AFRO_PWM && MOTORSobj.MotorSpeed[3] <MIN_AFRO_PWM){
    MOTORSobj.MotorSpeed[0]=MIN_AFRO_PWM;
    MOTORSobj.MotorSpeed[1]=MIN_AFRO_PWM;
    MOTORSobj.MotorSpeed[2]=MIN_AFRO_PWM;
    MOTORSobj.MotorSpeed[3]=MIN_AFRO_PWM;
   
    }else{
             
   /* Scale 0 - 255 to 0-25 and abstract it from the current motor speed*/
   MOTORSobj.MotorSpeed[0]-=round(5);
   MOTORSobj.MotorSpeed[1]-=round(5);
   MOTORSobj.MotorSpeed[2]-=round(5);
   MOTORSobj.MotorSpeed[3]-=round(5);    

   }
  }
      
// ------------------------------------------------
// YAW LEFT - Needs test
// ------------------------------------------------  

  if(StringCommand.equals("LEFT")){
  
   
    if  (PID_YAW_info.Setpoint< MIN_YAW_Deg){
    PID_YAW_info.Setpoint=MIN_YAW_Deg;
     }else{
     /*Only for the Command Compute the setpoint for the PID*/
     PID_YAW_info.Setpoint-=(TrnPwr); 
    } 
  }

// ------------------------------------------------
// YAW RIGHT - Needs test
// ------------------------------------------------ 

  if(StringCommand.equals("RIGHT")){
  
   
    if  (PID_YAW_info.Setpoint>MAX_YAW_Deg){
    PID_YAW_info.Setpoint=MAX_YAW_Deg;
     }else{
     /*Only for the Command Compute the setpoint for the PID*/
     PID_YAW_info.Setpoint+=(TrnPwr); 
    }
  }  

// ------------------------------------------------
// ROLL LEFT
// ------------------------------------------------ 

  if(StringCommand.equals("SQUARE")){
  
    if  (PID_ROLL_info.Setpoint>MIN_MAX_ROLL_PITCH_Deg){
    PID_ROLL_info.Setpoint=MIN_MAX_ROLL_PITCH_Deg;
     }else{
       
     /* in this step we increase the desired angle from the Play station controller*/  
     /*Only for the Command Compute the setpoint for the PID*/
     PID_ROLL_info.Setpoint += (TrnPwr);      
    }   
  }

// ------------------------------------------------
// ROLL RIGHT
// ------------------------------------------------

  if(StringCommand.equals("CIRCLE")){
     
   
    if  (PID_ROLL_info.Setpoint<-MIN_MAX_ROLL_PITCH_Deg){
      PID_ROLL_info.Setpoint=-MIN_MAX_ROLL_PITCH_Deg;
    }else{
      
     /* in this step we decrease the desired angle from the Play station controller*/   
     /*Only for the Command Compute the setpoint for the PID*/
     PID_ROLL_info.Setpoint-=(TrnPwr); 
    }
  }

// ------------------------------------------------
// PITCH FRONT
// ------------------------------------------------

  if(StringCommand.equals("TRIANGLE")){
     
    if  (PID_PITCH_info.Setpoint>MIN_MAX_ROLL_PITCH_Deg){
    PID_PITCH_info.Setpoint=MIN_MAX_ROLL_PITCH_Deg;
     }else{
       
     /* in this step we increase the desired angle from the Play station controller*/      
     /*Only for the Command Compute the setpoint for the PID*/
     PID_PITCH_info.Setpoint += (TrnPwr); 
    }
  }
  
// ------------------------------------------------
// PITCH BACK
// ------------------------------------------------     

  if(StringCommand.equals("X")){
   
   
    if  (PID_PITCH_info.Setpoint<-MIN_MAX_ROLL_PITCH_Deg){
      PID_PITCH_info.Setpoint=-MIN_MAX_ROLL_PITCH_Deg;
    }else{
      
     /* in this step we decrease the desired angle from the Play station controller*/     
     /*Only for the Command Compute the setpoint for the PID*/
     PID_PITCH_info.Setpoint-=(TrnPwr);
    }
  }
  
// ------------------------------------------------
// HOVER
// ------------------------------------------------    

  if(StringCommand.equals("L3")) { 
  /*Set Pitch and roll angle values to Zero*/
  
    int straightquad = round((MOTORSobj.MotorSpeed[0] + MOTORSobj.MotorSpeed[1])/2);
     
    MOTORSobj.MotorSpeed[0]= straightquad;      
    MOTORSobj.MotorSpeed[2]= straightquad;       
    MOTORSobj.MotorSpeed[1]= straightquad;       
    MOTORSobj.MotorSpeed[3]= straightquad; 
    
    PID_ROLL_info.Setpoint=0;
    PID_PITCH_info.Setpoint=0;
  }
      
// ------------------------------------------------
// Altitude Hold ON - Not supported
// ------------------------------------------------ 

  if(StringCommand.equals("R3") && AltitudeHold.equals("OFF")){

    /* Set Altitude hold on algorithm. Led on Altitude hold led*/
    AltitudeHold="ON";
    StringCommand="VOID"; // Set command to VOID to avoid enter to another Mode
  } 

// ------------------------------------------------
// Altitude Hold OFF - Not supported
// ------------------------------------------------ 

  if(StringCommand.equals("R3") && AltitudeHold.equals("ON")){
    
    /* Set Altitude hold off algorithm. Led off Altitude hold led*/
    AltitudeHold="OFF";
    StringCommand="VOID"; // Set command to VOID to avoid enter to another SELECT if
    }  
  } // End of Serial.available() - End of Command read from PS2 Cotroller
      

/*  --------------------------------------------------------------------------------------------------
  Matlab Data Acquisition for Debug
  --------------------------------------------------------------------------------------------------
*/ 
  /*You can Send to Matlab for plotting Only three variables at a time, you have a choice Between : */
  
  /* gyro_x_scalled - gyro_y_scalled - gyro_z_scalled     */
  /* accel_x_scalled - accel_y_scalled - accel_z_scalled  */
  /* angle_x_gyro - angle_y_gyro - angle_z_gyro           */
  /* angle_x_accel - angle_y_accel - accel_z_scalled      */
  /* angle_x - angle_y - angle_z                          */
  
#ifdef MATLAB_DATA_LOGGING

  DEBUGobj.SensorArray[0]=MPU6050obj.angle_x;
  DEBUGobj.SensorArray[1]=MPU6050obj.angle_y;
  DEBUGobj.SensorArray[2]=MPU6050obj.angle_z;
  DEBUGobj.Debug_XYZ_matlab_Print(DEBUGobj.SensorArray);
  delay(200); // Delay 200 so matlab can catch the values ..
  
#endif

dt=millis()-t;

/*
  --------------------------------------------------------------------------------------------------
  Debug to Arduino Serial implementation
  --------------------------------------------------------------------------------------------------
*/ 
#ifdef DEBUG
/*
  Serial.print("angle_x = ");
  Serial.print(MPU6050obj.angle_x);
  
  Serial.print("\t");
  Serial.print("angle_y = ");  
  Serial.print(MPU6050obj.angle_x);
  
  Serial.print("\t");
  Serial.print("angle_z = ");    
  Serial.println(MPU6050obj.angle_y);
 
  Serial.print("\t");
  Serial.print("Yaw = ");  
  Serial.print((Filter_gain_Gyro_Compass*MPU6050obj.angle_z_accel) + (1-Filter_gain_Gyro_Compass)*HMC5883obj.bearing);

  Serial.print("\t");   
  Serial.print("PID_Roll_Output= ");
  Serial.print(PID_ROLL_info.PIDOutput);

  Serial.print("\t");   
  Serial.print("PID_Pitch_Output= ");
  Serial.print(PID_PITCH_info.PIDOutput);
  
  Serial.print("\t");   
  Serial.print("PID_Yaw_Output= ");
  Serial.print(PID_YAW_info.PIDOutput);
  
  Serial.print("\t");
  Serial.print("Altitude = "); 
  Serial.print(MS5611obj.Absolute_Altitude); Serial.print(" m");
 */ 
  
  Serial.print("\t");
  Serial.print("Time = ");   
  Serial.println(millis()-t); 

#endif
}



