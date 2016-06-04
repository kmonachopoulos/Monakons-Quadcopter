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
  Includes
  --------------------------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include "PID.h"

/*
  --------------------------------------------------------------------------------------------------
  Function Prototyping
  --------------------------------------------------------------------------------------------------
*/

//  --------------------------------------------------------------------------------------------------
//        Information Constructor function
//  --------------------------------------------------------------------------------------------------

/*Constructor (...)*********************************************************
 *    The parameters specified here have been set by the user.
 ***************************************************************************/
PID_INFO::PID_INFO(double Pre_Def_Kp ,double Pre_Def_Ki, double Pre_Def_Kd)
{
   kp = Pre_Def_Kp;
   ki = Pre_Def_Ki;
   kd = Pre_Def_Kd;  
   Setpoint = 0;  // On start initialization of setpoint

}

//  --------------------------------------------------------------------------------------------------
//        Execution Constructor function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 *    The input parameters here are specified in PID_INFO class .
 ****************************************************************************/
PID_EXEC::PID_EXEC(double Kp, double Ki, double Kd, int ControllerDirection)
{

    inAuto = false;
    
    PID_EXEC::SetOutputLimits(-200,200);			        //default output limit corresponds to
								//the arduino pwm limits

    SampleTime = 5;						// -(miimum sample time)- default Controller Sample Time is 0.1 seconds (SampleTime = 100)

    PID_EXEC::SetControllerDirection(ControllerDirection);
    PID_EXEC::SetTunings(Kp, Ki, Kd);

				
}
 
//  --------------------------------------------------------------------------------------------------
//        Compute function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
double PID_EXEC::Compute(double Input, double Setpoint)
{   
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   
   /*Enter PID execution*/
   if(1)
   {
      /*Compute all the working error variables*/
      double PIDinput = Input;
      PIDin=PIDinput;
      
      double error = Setpoint - PIDinput;          
      ITerm+= (ki * error);      
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;  
   
       /*Windowing Iterm*/
     if(abs(error)>10) ITerm = 0; // CHECK IT
         
      double dInput = (PIDinput - lastInput);

      /*Compute PID Output*/
      double PIDoutput = kp * error + ITerm- kd * dInput;       
             PIDout= PIDoutput;
             
	  if(PIDoutput > outMax) PIDoutput = outMax;
      else if(PIDoutput < outMin) PIDoutput = outMin;
	  //Output = PIDoutput;
	  
      /*Remember some variables for next time*/
      lastInput = PIDinput;
      lastTime = now;
      return PIDoutput;
   }else{
   return 0;
   }
}

//  --------------------------------------------------------------------------------------------------
//        SetTunings function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_EXEC::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

//  --------------------------------------------------------------------------------------------------
//        SetSampleTime function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID_EXEC::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

//  --------------------------------------------------------------------------------------------------
//        SetOutputLimits function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_EXEC::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(PIDout > outMax) PIDout = outMax;
	   else if(PIDout < outMin) PIDout = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

//  --------------------------------------------------------------------------------------------------
//       SetMode function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
******************************************************************************/ 
void PID_EXEC::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID_EXEC::Initialize();
    }
    inAuto = newAuto;
}

//  --------------------------------------------------------------------------------------------------
//       Initialize function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_EXEC::Initialize()
{
   ITerm = PIDout;
   lastInput = PIDin;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

//  --------------------------------------------------------------------------------------------------
//       SetControllerDirection function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_EXEC::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}


//  --------------------------------------------------------------------------------------------------
//       Status function
//  --------------------------------------------------------------------------------------------------

/****************************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID_EXEC::GetKp(){ return  dispKp; }
double PID_EXEC::GetKi(){ return  dispKi;}
double PID_EXEC::GetKd(){ return  dispKd;}
int PID_EXEC::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID_EXEC::GetDirection(){ return controllerDirection;}


