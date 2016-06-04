Quadcopter is an autonomous, unmanned flying robotic system witch includes four onboard rotating propellers. 
It consists a field of interest both in research and in development of many applications. In this project, 
the Quadcopter that we develop is manufactured from scratch while the sensors and the components that it is 
constructed from are carefully selected to work in harmony, providing the desired results.The whole process is 
based on Arduino Uno that is responsible for undertaking both control and communication procedure of the Quadcopter, 
combining the implementation of independent components. The communication process is implemented through two xbees, 
one on the Quadcopter and one on the Play Station Controller. 

These code files concerns the Arduino Uno attached on the Quadcopter. The codes for the controller will be attached to another repository.
Parts of the code are copied from https://www.arduino.cc/, http://www.billporter.info/ and hobbylog's OMER Project.
 
The target of the main project is to create a full controllable and functional platform that will be appropriate to be easily programed and have autonomous ability.
The Quadcopter implements up to four degrees of freedom, as it consists of four motors,
that can be controlled using the appropriate amount of current. To complete a stable and successful flight,
several high accuracy sensors are coupled with a fast and efficient control system.
The sensor data must be estimated and then combined to produce a functionally correct flight. 
The balance control is done using a PID Controller through which we eliminate the error motion in each axis. 
After the completion of the individual necessary steps, we implement the necessery test flights, 
including both in interior and in exterior area. The total code consists of about 1500 
lines of code and has been developed in C ++ programming language through the development environment that Arduino provides.


// Basic Structure :
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
