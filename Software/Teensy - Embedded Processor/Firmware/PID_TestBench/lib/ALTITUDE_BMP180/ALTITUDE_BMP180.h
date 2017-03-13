#ifndef __ALTITUDE_BMP180_H__
#define __ALTITUDE_BMP180_H__

#include "Arduino.h"
#include <i2c_t3.h>
#include <math.h>


/* =====================================================================================
                    ALTITUDE AND PRESSURE SENSOR (BMP180)
====================================================================================== */

/*=========================================================================
    I2C ADDRESS
    -----------------------------------------------------------------------*/
    #define PRESSURE_ADDR  0x77
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    // Pressure sensor calibration data
    #define PRESSURE_REGISTER_CAL_AC1       0xAA  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_AC2       0xAC  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_AC3       0xAE  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_AC4       0xB0  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_AC5       0xB2  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_AC6       0xB4  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_B1        0xB6  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_B2        0xB8  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_MB        0xBA  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_MC        0xBC  // Read Only -  Calibration data (16 bits)
    #define PRESSURE_REGISTER_CAL_MD        0xBE  // Read Only -  Calibration data (16 bits)
    // Pressure sensor
    #define PRESSURE_REGISTER_CONTROL       0xF4
    #define PRESSURE_REGISTER_TEMPDATA      0xF6
    #define PRESSURE_REGISTER_PRESSUREDATA  0xF6
    // Sensor Commands
    #define PRESSURE_COMMAND_READTEMP       0x2E
    #define PRESSURE_COMMAND_READPRESSURE   0xF4  // Start the Pressure  measurement, with oss = 3.

/*=========================================================================*/


/*=========================================================================
    PARAMETERS
    -----------------------------------------------------------------------*/
    #define PRESSURE_OSS            3
    #define PRESSURE_SEA_LEVEL_HPA  1013.25 // hPa

/*=========================================================================*/



    typedef struct
    {
      int16_t  ac1;
      int16_t  ac2;
      int16_t  ac3;
      uint16_t ac4;
      uint16_t ac5;
      uint16_t ac6;
      int16_t  b1;
      int16_t  b2;
      int16_t  mb;
      int16_t  mc;
      int16_t  md;
    } bmp180_calibration_data;


class AltitudeSensor {

  private:
    // EEPROM calibration data
    bmp180_calibration_data calibrationData;
    int32_t B5;
    // State variables
    int32_t temperature;
    float pressure;
    float altitude;
    // Time variables
    elapsedMicros temperature_timer;
    elapsedMicros pressure_timer;
    // Original Altitude;
    float referenceAltitude;
    // State
    uint8_t state;
    //Brown linear exponential filter
    float _alfa;

    //I2C communication functions
    void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value);
    uint8_t i2c_read_byte(uint8_t address, uint8_t reg);
    uint16_t i2c_read_word(uint8_t address, uint8_t reg);

    //Preparation functions
    void readCoefficients(void);

    // State machine funcions
    // State 0
    void startTemperatureRead(void);        // wait 4.5ms
    // State 1
    void readCompensateTemperature(void);
    void startPressureRead(void);           //wait 25ms
    // State 2
    void readCompensatePressure(void);
    void calculateAltitude(void);           //Return to state 1

  public:
    //Constructor
    AltitudeSensor(float alfa);
    AltitudeSensor();
    //Initialize filter
    void begin(void);
    //Step the filter
    void step(void);

    float getAltitude(void);
    float getTemperature(void);
    float getPressure(void);
};

#endif
