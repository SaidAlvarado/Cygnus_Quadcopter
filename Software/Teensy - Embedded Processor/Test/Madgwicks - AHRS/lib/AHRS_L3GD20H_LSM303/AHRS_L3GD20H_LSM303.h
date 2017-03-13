#ifndef __AHRS_L3GD20H_LSM303_H__
#define __AHRS_L3GD20H_LSM303_H__

#include "Arduino.h"
#include <i2c_t3.h>
#include <math.h>


/*=========================================================================
    I2C ADDRESS
    -----------------------------------------------------------------------*/
    #define GYRO_ADDR  0x6B
    #define ACCEL_ADDR 0x19
    #define MAGN_ADDR  0x1E
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    //Accelerometer
    #define ACCEL_REGISTER_CTRL_REG1_A  0x20
    #define ACCEL_REGISTER_CTRL_REG4_A  0x23
    #define ACCEL_REGISTER_OUT_X_L_A    0x28
    //Magnetometer
    #define MAGN_REGISTER_CRA_REG_M     0x00
    #define MAGN_REGISTER_CRB_REG_M     0x01
    #define MAGN_REGISTER_MR_REG_M      0x02
    #define MAGN_REGISTER_OUT_X_H_M     0x03
    //Gyroscope
    #define GYRO_REGISTER_CTRL_REG1     0x20
    #define GYRO_REGISTER_CTRL_REG4     0x23
    #define GYRO_REGISTER_CTRL_REG5     0x24
    #define GYRO_REGISTER_OUT_X_L       0x28
/*=========================================================================*/



/*=========================================================================
    CONSTANTS
    -----------------------------------------------------------------------*/
    // #define PI 3.14159265358979323846264338328
/*=========================================================================*/


/*=========================================================================
    CLASS DEFINITION
    -----------------------------------------------------------------------*/
class MadgwicksFilter {

  private:
    //Sensor data storage
    int16_t _accelDataRaw [3];
    int16_t _gyroDataRaw  [3];
    int16_t _magnDataRaw  [3];
    //Converted data storage
    float _accelData [3]; // Normalized acceleration
    float _gyroData  [3]; // Angular Velocity in radians/seg
    float _magnData  [3]; // discrete magnetic field
    //Constants
    float _beta;
    float _zeta;
    //Earth Magnetic Flux
    float _b_x = 1;
    float _b_z = 0;
    //Gyroscopic Bias Error
    float _w_b[3] = {0,0,0};
    //Fixed Gyroscopic Data
    float _gyroDataFixed[3] = {0,0,0};
    //Attitude
    float _quater[4];
    //last micros
    float lastMicros;

    // Basic I2C communication functions
    void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value);
    uint8_t i2c_read_byte(uint8_t address, uint8_t reg);

    //Sensor setup functions
    void accelSetup(void);
    void gyroSetup(void);
    void magnSetup(void);

    //Sensor Data read
    void accelRead(int16_t accelerometerDataRaw[], float accelerometerData[]);
    void gyroRead(int16_t gyroscopeDataRaw[], float gyroscopeData[]);
    void magnRead(int16_t magnetometerDataRaw[], float magnetometerData[]);

    //filter
    void filter(float gyro[], float accel[], float magn[], float dt );

    //Data transformation
    void quat2Euler(float quater[], float euler[]);

    //Quickstart function
    void initialEstimation(float acceleration[], float magnetism[]);



  public:
    //Constructor
    MadgwicksFilter(float beta, float zeta);
    MadgwicksFilter();
    //Initialize filter
    void begin(void);
    //Step the filter
    void step(void);

    //Sets and Gets
    void getAccelData(int16_t accelerometerData[]);
    void getGyroData(int16_t gyroscopeData[]);
    void getMagnData(int16_t magnetometerData[]);

    void setBeta(float beta);
    void setZeta(float zeta);

    void getAttitude(float attitude[]);
    void getAttitudeEuler(float eulerAngle[]);
    void getAngularVelocity(float w[]);

};

/*=========================================================================*/
#endif
