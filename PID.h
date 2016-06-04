/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  University of Patras - Electronics and information processing Division of Electronics and Computers (Department of physics) 
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  Project       : Monakons Quadcopter
  File          : PID_Control.ino
  Description   : This code defines PID process
  Author        : Monahopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  --------------------------------------------------------------------------------------------------
  DEFINITIONS
  --------------------------------------------------------------------------------------------------
*/

   #ifndef PID_h
   #define PID_h

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT        0
  #define REVERSE       1

/*
  --------------------------------------------------------------------------------------------------
   PID CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/   
class PID_INFO
{
  
    public:
    
    PID_INFO(double,double,double);    
    double PIDOutput;
    double Setpoint;
    double kp, ki, kd;

};

/*
  --------------------------------------------------------------------------------------------------
   PID CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/

class PID_EXEC
{
  
  public:

// ------------------------------------------------
// commonly used functions 
// ------------------------------------------------

    PID_EXEC(double, double, double, int);     /* constructor.  links the PID to the Input, Output, and Setpoint.  Initial tuning parameters are also set here*/
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    double Compute(double, double); // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
					  //it's likely the user will want to change this depending on
					  //the application
	
// ------------------------------------------------
// available but not commonly used functions 
// ------------------------------------------------

    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
                                          
    void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
					  //   means the output will increase when error is positive. REVERSE
					  //   means the opposite.  it's very unlikely that this will be needed
					  //   once it is set in the constructor.

    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
// ------------------------------------------------
// Display functions
// ------------------------------------------------										  
   
    double GetKp();						  // These functions query the pid for interal values.
    double GetKi();						  //  they were created mainly for the pid front-end,
    double GetKd();						  // where it's important to know what is actually 
    int GetMode();						  //  inside the PID.
    int GetDirection();					  //
    

    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    unsigned long lastTime, SampleTime;
    
  private:
     double PIDout; 
    double PIDin; 
    void Initialize();	
    double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
    double dispKi;				//   format for display purposes
    double dispKd;				//
    int controllerDirection;
    
   // double myInput;              // * Pointers to the Input, Output, and Setpoint variables
  //  double myOutput;             //   This creates a hard link between the variables and the 
  //  double mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know. 
                                  
		  
    double ITerm, lastInput;
    double outMin, outMax;
    bool inAuto;
};
#endif


