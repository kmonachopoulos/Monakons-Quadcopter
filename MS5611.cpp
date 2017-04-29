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
/*
  --------------------------------------------------------------------------------------------------
  Includes
  --------------------------------------------------------------------------------------------------
*/
#include "MS5611.h"

/*
  --------------------------------------------------------------------------------------------------
  DEFINITIONS
  --------------------------------------------------------------------------------------------------
*/

#define CONVERSION_TIME 10000l // conversion time in microseconds

/*
  --------------------------------------------------------------------------------------------------
  Function Prototyping
  --------------------------------------------------------------------------------------------------
*/

// ------------------------------------------------
//       Constructor function
// ------------------------------------------------
MS5611_CLASS::MS5611_CLASS(void) {
  
  sea_press = 1013.25;
  temperature=0;
  pression=0;
}

// ------------------------------------------------
//       MS5611 Initialization
// ------------------------------------------------

void MS5611_CLASS::init(uint8_t address) {  
  _addr =  address;
  
  // disable internal pullups of the ATMEGA which Wire enable by default
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
//    cbi(PORTD, 0);
//    cbi(PORTD, 1);
  #endif
 
  reset(); // reset the device to populate its internal PROM registers
  delay(1000); // some safety time
  readPROM(); // reads the PROM into object variables for later use
}

// ------------------------------------------------
//       Read Pressure Data From MS5611
// ------------------------------------------------

double MS5611_CLASS::getPressure(uint8_t OSR) {
  // see datasheet page 7 for formulas
  uint32_t rawPress = rawPressure(OSR);
  if(rawPress == NULL) {
    return NULL;
  }
  int32_t dT = getDeltaTemp(OSR);
  if(dT == NULL) {
    return NULL;
  }
  
  int64_t off  = ((uint32_t)test[1] <<16) + (((int64_t)dT * test[3]) >> 7);
  int64_t sens = ((uint32_t)test[0] <<15) + (((int64_t)dT * test[2]) >> 8);
  return ((( (rawPress * sens ) >> 21) - off) >> 15) / 100.0;
}

// ------------------------------------------------
//       Read Temperature Data From MS5611
// ------------------------------------------------

double MS5611_CLASS::getTemperature(uint8_t OSR) {
  // see datasheet page 7 for formulas
  int64_t dT = getDeltaTemp(OSR);
  
  if(dT != NULL) {
    return (2000 + ((dT * test[5]) >> 23)) / 100.0;
  }
  else {
    return NULL;
  }
}

// ------------------------------------------------
//       Read Temperature Data From MS5611
// ------------------------------------------------

int32_t MS5611_CLASS::getDeltaTemp(uint8_t OSR) {
  uint32_t rawTemp = rawTemperature(OSR);
  if(rawTemp != NULL) {
    return (int32_t)(rawTemp - ((uint32_t)test[4] << 8));
  }
  else {
    return NULL;
  }
}

// ------------------------------------------------
//       Read Pressure Raw Data From MS5611
// ------------------------------------------------

uint32_t MS5611_CLASS::rawPressure(uint8_t OSR) {
  unsigned long now = micros();
  if(lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME) {
    lastPresConv = 0;
    pressCache = getConversion(MS561101BA_D1 + OSR);
  }
  else {
    if(lastPresConv == 0 && lastTempConv == 0) {
      startConversion(MS561101BA_D1 + OSR);
      lastPresConv = now;
    }
    return pressCache;
  }
}

// ------------------------------------------------
//       Read Real Temperature From MS5611
// ------------------------------------------------

uint32_t MS5611_CLASS::rawTemperature(uint8_t OSR) {
  unsigned long now = micros();
  if(lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME) {
    lastTempConv = 0;
    tempCache = getConversion(MS561101BA_D2 + OSR);
  }
  else {
    if(lastTempConv == 0 && lastPresConv == 0) { // no conversions in progress
      startConversion(MS561101BA_D2 + OSR);
      lastTempConv = now;
    }
  }
  return tempCache;
}


// ------------------------------------------------
//       Start Data Exchange From MS5611
// ------------------------------------------------

void MS5611_CLASS::startConversion(uint8_t command) {
  // initialize pressure conversion
  Wire.beginTransmission(_addr);
  Wire.write(command);
  Wire.endTransmission();
}

// ------------------------------------------------
//     Get Feedback for Data Exchange From MS5611
// ------------------------------------------------

uint32_t MS5611_CLASS::getConversion(uint8_t command) {
  union {uint32_t val; uint8_t raw[4]; } conversion = {0};
  
  // start read sequence
  Wire.beginTransmission(_addr);
  Wire.write(0);
  Wire.endTransmission();
  
  Wire.beginTransmission(_addr);
  Wire.requestFrom(_addr, (uint8_t) MS561101BA_D1D2_SIZE);
  if(Wire.available()) {
    conversion.raw[2] = Wire.read();
    conversion.raw[1] = Wire.read();
    conversion.raw[0] = Wire.read();
  }
  else {
    conversion.val = -1;
  }
  
  return conversion.val;
}

// ------------------------------------------------
//       Read PROM Data From MS5611
// ------------------------------------------------
/* Reads factory calibration and store it into object variables. */

int MS5611_CLASS::readPROM() {
  for (int i=0;i<MS561101BA_PROM_REG_COUNT;i++) {
    Wire.beginTransmission(_addr);
    Wire.write(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    Wire.endTransmission();
    
    Wire.beginTransmission(_addr);
    Wire.requestFrom(_addr, (uint8_t) MS561101BA_PROM_REG_SIZE);
    if(Wire.available()) {
      test[i] = Wire.read() << 8 | Wire.read();
      
      //DEBUG_PRINT(test[i]);
    }
    else {
      return -1; // error reading the PROM or communicating with the device
    }
  }
  return 0;
}

// ------------------------------------------------
//       MS5611 Reset
// ------------------------------------------------

/* Send a reset command to the device. With the reset command the device
   populates its internal registers with the values read from the PROM.
*/
void MS5611_CLASS::reset() {
  Wire.beginTransmission(_addr);
  Wire.write(MS561101BA_RESET);
  Wire.endTransmission();
}


// ------------------------------------------------
//       MS5611 Altitude Calculation
// ------------------------------------------------

double MS5611_CLASS::getAltitude(double Pressure, double Temperature) {
  
 return ((pow((sea_press / Pressure), 1/5.257) - 1.0) * (Temperature + 273.15)) / 0.0065;
}

