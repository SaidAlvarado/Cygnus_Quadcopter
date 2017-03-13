/*######################################################################################################################
##############                  ALTITUDE & PRESSURE SENSOR AND FILTER (BMP180)                          ################
######################################################################################################################*/

#include <i2c_t3.h>
#include <math.h>
#include "ALTITUDE_BMP180.h"
using namespace std;



/* =====================================================================================
                             CONSTRUCTOR
====================================================================================== */


AltitudeSensor :: AltitudeSensor(float alfa){

    _alfa = alfa;
}


AltitudeSensor :: AltitudeSensor(){

    _alfa = 0.5;
}

/* =====================================================================================
                             I2C BASIC FUNCTIONS
====================================================================================== */

void AltitudeSensor :: i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value){

    //Transmit one byte (value) through the I2C bus to the 'reg' register in the 'address' device
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}


uint8_t AltitudeSensor :: i2c_read_byte(uint8_t address, uint8_t reg){

    //Receives de 'reg' register of the device 'address' through the I2C bus
    uint8_t value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(address, 1, I2C_STOP);
    value = Wire.readByte();
    Wire.endTransmission();

    return value;
}


uint16_t AltitudeSensor :: i2c_read_word(uint8_t address, uint8_t reg){

    //Receives the 'reg' and 'reg + 1' registers of the device 'address' through the I2C bus
    uint16_t value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(address, 2, I2C_STOP);
    value = (Wire.readByte() << 8) | Wire.readByte();
    Wire.endTransmission();

    return value;
}


/* =====================================================================================
                             SETUP FUNCTIONS
====================================================================================== */

void AltitudeSensor :: readCoefficients(void){

    // Read the EEPROM calibration data from the sensors.
    calibrationData.ac1 = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_AC1);    // AC1:  7511
    calibrationData.ac2 = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_AC2);    // AC2: -1150
    calibrationData.ac3 = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_AC3);    // AC3: -14488
    calibrationData.ac4 =           i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_AC4);    // AC4:  33573
    calibrationData.ac5 =           i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_AC5);    // AC5:  25610
    calibrationData.ac6 =           i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_AC6);    // AC6:  1465
    calibrationData.b1  = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_B1);     // B1:   6515
    calibrationData.b2  = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_B2);     // B2:   44
    calibrationData.mb  = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_MB);     // MB:  -32768
    calibrationData.mc  = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_MC);     // MC:  -11786
    calibrationData.md  = (int16_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_CAL_MD);     // MD:   2448
}



void AltitudeSensor :: begin(void){

    // Initialize I2C BUS
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

    // Read the EEPROM coefficient
    readCoefficients();

    // Start the state machines
    state = 0;

    // One quick run to get the initial values
    startTemperatureRead();
    delay(5);
    readCompensateTemperature();
    startPressureRead();
    delay(25);
    readCompensatePressure();
    calculateAltitude();

    //initialize the reference altitude.
    referenceAltitude = altitude;
}



/* =====================================================================================
                            STATE MACHINE FUNCTIONS
====================================================================================== */

// State 0
void AltitudeSensor :: startTemperatureRead(void){

    // Start the measurement
    i2c_write_byte(PRESSURE_ADDR, PRESSURE_REGISTER_CONTROL, PRESSURE_COMMAND_READTEMP);

    // Start temperature_timer
    temperature_timer = 0;  //We have to wait 4.5ms before picking up the value.

    // Move state
    state = 1;
}


// State 1
void AltitudeSensor :: readCompensateTemperature(void){

    int32_t uncompensated_temperature, X1, X2;

    // Take the temperature value
    uncompensated_temperature = (int32_t) i2c_read_word(PRESSURE_ADDR, PRESSURE_REGISTER_TEMPDATA);

    // We calculate the B5
    X1 = ( uncompensated_temperature - (int32_t)calibrationData.ac6 ) * ( (int32_t)calibrationData.ac5 ) >> 15;
    X2 = ( (int32_t)calibrationData.mc << 11 ) / ( X1 + (int32_t)calibrationData.md );
    B5 = X1 + X2;

    // Compensate temperature
    temperature = (B5 + 8) >> 4;
}


void AltitudeSensor :: startPressureRead(void){

    // Start the measurement
    i2c_write_byte(PRESSURE_ADDR, PRESSURE_REGISTER_CONTROL, PRESSURE_COMMAND_READPRESSURE);

    // Start temperature_timer
    pressure_timer = 0;  //We have to wait 25.5ms before picking up the value.

    // Move state
    state = 2;
}


// State 2
void AltitudeSensor :: readCompensatePressure(void){

    int32_t uncompensated_pressure, b6, x1, x2, x3, b3, p;
    uint32_t MSB, LSB, XLSB, b4, b7;

    //Read the values from the ADC registers.
    MSB = ((uint32_t) i2c_read_byte(PRESSURE_ADDR, PRESSURE_REGISTER_PRESSUREDATA)) << 16;
    LSB = ((uint32_t) i2c_read_byte(PRESSURE_ADDR, PRESSURE_REGISTER_PRESSUREDATA + 1)) << 8;
    XLSB = (uint32_t) i2c_read_byte(PRESSURE_ADDR, PRESSURE_REGISTER_PRESSUREDATA + 2);
    //Combine them to get the uncompensated pressure.
    uncompensated_pressure = (int32_t) (MSB + LSB + XLSB) >> (8 - PRESSURE_OSS);

    //Compensate temperature
    b6 = B5 - 4000;
    x1 = (calibrationData.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (calibrationData.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) calibrationData.ac1) * 4 + x3) << PRESSURE_OSS) + 2) >> 2;
    x1 = (calibrationData.ac3 * b6) >> 13;
    x2 = (calibrationData.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (calibrationData.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (uncompensated_pressure - b3) * (50000 >> PRESSURE_OSS));

    if (b7 < 0x80000000)    p = (b7 << 1) / b4;
    else                    p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    //store Pressure in hPa (hectoPascals)
    pressure = ((float)p)/100;

}


void AltitudeSensor :: calculateAltitude(void){

    float yt;

    //Calculate the altitude
    yt =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA_LEVEL_HPA, 0.1903));

    //brown exponential smoothing
     altitude = _alfa * yt  +  (1.0 - _alfa) * altitude;

    //Return to state 0
    state = 0;
}




/* =====================================================================================
                                    ITERATOR
====================================================================================== */


void AltitudeSensor :: step(void){

    //state machine implementation for the temperature, pressure and altitude update
    if (state == 0) startTemperatureRead();

    if (state == 1 && (temperature_timer > 4500) ){  //Perhaps should change this to elapsed Milis to avoid overflow cases.
        readCompensateTemperature();
        startPressureRead();
    }

    if (state == 2 && (pressure_timer > 25500) ){
        readCompensatePressure();
        calculateAltitude();
    }
}


/* =====================================================================================
                 DATA MANAGMENT FUNCTION  (SETS AND GETS)
====================================================================================== */

float AltitudeSensor :: getTemperature(void){

    return  ((float)temperature)/10.0;
}

float AltitudeSensor :: getPressure(void){

    return  pressure;
}

float AltitudeSensor :: getAltitude(void){

    return  altitude;
}
