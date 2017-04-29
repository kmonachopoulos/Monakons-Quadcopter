/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  University of Patras - Electronics and information processing Division of Electronics and Computers (Department of physics) 
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*
  ------------------------------------------------------------------------------------------------------------------------------------------
  Project       : Monakons Quadcopter
  File          : MS5611_Control.ino
  Description   : This code test MS5611 sensor
  Author        : Monachopoulos Konstantinos
  ------------------------------------------------------------------------------------------------------------------------------------------
*/
/*  NOTE - This File is different from the file that is described in Report. Changing the main sketch optimized the implementation */
/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/

#ifndef MS561101BA_h
#define MS561101BA_h

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/

#include "Arduino.h"
#include <Wire.h>


// addresses of the device
#define MS561101BA_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

/*
  --------------------------------------------------------------------------------------------------
   MS5611 CLASS DECLARATION
  --------------------------------------------------------------------------------------------------
*/

class MS5611_CLASS {
  
  public:
    MS5611_CLASS();    //class constructor
    
    void init(uint8_t addr);
    double getPressure(uint8_t OSR);
    double getTemperature(uint8_t OSR);
    int32_t getDeltaTemp(uint8_t OSR);
    uint32_t rawPressure(uint8_t OSR);
    uint32_t rawTemperature(uint8_t OSR);
    int readPROM();
    void reset();
    uint32_t lastPresConv, lastTempConv;
    double temperature;
    double pression;
    
    double Absolute_Altitude;
    double getAltitude(double Pressure, double Temperature); 
    
  private:
    
    double sea_press;   
    void startConversion(uint8_t command);
    uint32_t getConversion(uint8_t command);
    uint8_t _addr;
    uint16_t test[MS561101BA_PROM_REG_COUNT]; 
    uint32_t pressCache, tempCache;
};

#endif
