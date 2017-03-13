/*######################################################################################################################
##############                ATTITUDE & HEADING REFERENCE SYSTEM  (L3GD20H & LSM303)                   ################
######################################################################################################################*/

#include <i2c_t3.h>
#include <math.h>
#include "AHRS_L3GD20H_LSM303.h"
using namespace std;



/* =====================================================================================
                             CONSTRUCTOR
====================================================================================== */

MadgwicksFilter :: MadgwicksFilter(float beta, float zeta){

    // Assign constants
    _beta = sqrt(3.0/4.0) * PI/180 * beta;          // gyro measurment error rad/s  (5 deg/s)
    _zeta = sqrt(3.0/4.0) * PI/180 * zeta;          // gyro drift error rad/s/s   (0.2 deg/s/s)
}

MadgwicksFilter :: MadgwicksFilter(){

    float beta = 5;
    float zeta = 0.2;

    // Initialize I2C BUS
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_600);

    // Assign constants
    _beta = sqrt(3.0/4.0) * PI/180 * beta;          // gyro measurment error rad/s  (5 deg/s)
    _zeta = sqrt(3.0/4.0) * PI/180 * zeta;          // gyro drift error rad/s/s   (0.2 deg/s/s)
}




/* =====================================================================================
                             I2C BASIC FUNCTIONS
====================================================================================== */

void MadgwicksFilter :: i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value){

    //Transmit one byte (value) through the I2C bus to the 'reg' register in the 'address' device

    uint8_t data;
    Serial.begin(115200);

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    data = Wire.endTransmission();
    // digitalWriteFast(LED_BUILTIN, LOW);
    // for (size_t i = 0; i < 50; i++) {
    //     Serial.println(data);
    //     delay(1000);
    //     /* code */
    // }
}


uint8_t MadgwicksFilter :: i2c_read_byte(uint8_t address, uint8_t reg){

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



/* =====================================================================================
                             SETUP FUNCTIONS
====================================================================================== */

void MadgwicksFilter :: accelSetup(void){

    i2c_write_byte(ACCEL_ADDR, ACCEL_REGISTER_CTRL_REG4_A, 0x88);         // Turn on High Resolution, and Hold Update.
    i2c_write_byte(ACCEL_ADDR, ACCEL_REGISTER_CTRL_REG1_A, 0x27);         // Turn on Accelerometer.
}

void MadgwicksFilter :: gyroSetup(void){

    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG4, 0xA0);         // Scale 2000dps, BlockUpdates.
    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG5, 0x02);         // OutSel = 10h, use HPF and LPF2, HPen = 0.
    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG1, 0x8F);         // DataRate 400Hz, BW 20Hz, All Axis enabled, Gyro ON.
}

void MadgwicksFilter :: magnSetup(void){

    i2c_write_byte(MAGN_ADDR, MAGN_REGISTER_CRA_REG_M, 0x1C);         // Data Rate = 220Hz (originally it was 15Hz --> 0x10)
    i2c_write_byte(MAGN_ADDR, MAGN_REGISTER_CRB_REG_M, 0x20);         // Scale +-1.3gauss
    i2c_write_byte(MAGN_ADDR, MAGN_REGISTER_MR_REG_M,  0x00);         // Turn on Magnetometer
}



/* =====================================================================================
                            DATA READ FUNCTIONS
====================================================================================== */

void MadgwicksFilter :: accelRead(int16_t accelerometerDataRaw[], float accelerometerData[]){

    // Read the accelerometer
    Wire.beginTransmission(ACCEL_ADDR);
    Wire.write(ACCEL_REGISTER_OUT_X_L_A | 0x80);   // this 'OR' activates the continous read function
    Wire.endTransmission();
    Wire.requestFrom(ACCEL_ADDR, 6, I2C_NOSTOP);

    // Wait around until enough data is available
    while (Wire.available() < 6);

        uint8_t xlo = Wire.readByte();
        uint8_t xhi = Wire.readByte();
        uint8_t ylo = Wire.readByte();
        uint8_t yhi = Wire.readByte();
        uint8_t zlo = Wire.readByte();
        uint8_t zhi = Wire.readByte();

    // Shift values to create properly formed integer (low byte first)
    accelerometerDataRaw[0] = (int16_t)(xlo | (xhi << 8)) >> 4;
    accelerometerDataRaw[1] = (int16_t)(ylo | (yhi << 8)) >> 4;
    accelerometerDataRaw[2] = (int16_t)(zlo | (zhi << 8)) >> 4;

    //We normalize the values
    float magnitude = sqrt(accelerometerDataRaw[0] * accelerometerDataRaw[0] + accelerometerDataRaw[1] * accelerometerDataRaw[1] + accelerometerDataRaw[2] * accelerometerDataRaw[2]);

    accelerometerData[0] = accelerometerDataRaw[0]/magnitude;
    accelerometerData[1] = accelerometerDataRaw[1]/magnitude;
    accelerometerData[2] = accelerometerDataRaw[2]/magnitude;
}



void MadgwicksFilter :: magnRead(int16_t magnetometerDataRaw[], float magnetometerData[]){

  // Read the magnetometer
  Wire.beginTransmission(MAGN_ADDR);
  Wire.write(MAGN_REGISTER_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(MAGN_ADDR, 6, I2C_NOSTOP);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Note high before low (different than accel)
    uint8_t xhi = Wire.readByte();
    uint8_t xlo = Wire.readByte();
    uint8_t zhi = Wire.readByte();
    uint8_t zlo = Wire.readByte();
    uint8_t yhi = Wire.readByte();
    uint8_t ylo = Wire.readByte();

  // Shift values to create properly formed integer (low byte first)
  magnetometerDataRaw[0] = (int16_t)(xlo | ((int16_t)xhi << 8));
  magnetometerDataRaw[1] = (int16_t)(ylo | ((int16_t)yhi << 8));
  magnetometerDataRaw[2] = (int16_t)(zlo | ((int16_t)zhi << 8));

  // We calibrate the magnetometer measure
  magnetometerData[0] = (((float) magnetometerDataRaw[0]) - 35.0) * 1.0;
  magnetometerData[1] = (((float) magnetometerDataRaw[1]) + 35.0) * 1.02702702703;
  magnetometerData[2] = (((float) magnetometerDataRaw[2]) - 3.0 ) * 0.974358974359;

  // We normalize the values
  float magnitude = sqrt(magnetometerData[0] * magnetometerData[0] + magnetometerData[1] * magnetometerData[1] + magnetometerData[2] * magnetometerData[2]);

  magnetometerData[0] = magnetometerData[0]/magnitude;
  magnetometerData[1] = magnetometerData[1]/magnitude;
  magnetometerData[2] = magnetometerData[2]/magnitude;
}


void MadgwicksFilter :: gyroRead(int16_t gyroscopeDataRaw[], float gyroscopeData[]){

    // Read the gyroscope
    Wire.beginTransmission(GYRO_ADDR);
    Wire.write(GYRO_REGISTER_OUT_X_L | 0x80);  // The 'OR' sets the auto-increment bit
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 6, I2C_NOSTOP);

    // Wait around until enough data is available
    while (Wire.available() < 6);

    uint8_t xlo = Wire.readByte();
    uint8_t xhi = Wire.readByte();
    uint8_t ylo = Wire.readByte();
    uint8_t yhi = Wire.readByte();
    uint8_t zlo = Wire.readByte();
    uint8_t zhi = Wire.readByte();


  // Shift values to create properly formed integer (low byte first)
  gyroscopeDataRaw[0] = (int16_t)(xlo | (xhi << 8));
  gyroscopeDataRaw[1] = (int16_t)(ylo | (yhi << 8));
  gyroscopeDataRaw[2] = (int16_t)(zlo | (zhi << 8));

  // Store the values in radians per second
  gyroscopeData[0] = gyroscopeDataRaw[0] * 0.0012217304764;   // 0.0012217304764 = 70/1000 * PI /180
  gyroscopeData[1] = gyroscopeDataRaw[1] * 0.0012217304764;
  gyroscopeData[2] = gyroscopeDataRaw[2] * 0.0012217304764;
}


/* =====================================================================================
                                   MADGWICKS FILTER
====================================================================================== */

void MadgwicksFilter :: filter(float gyro[], float accel[], float magn[], float dt ){

    // local system variables
    float norm;                                                             // vector norm
    float SEqDot_omega[4];                                                  // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6;                                     // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; // objective function Jacobian elements
    float SEqHatDot[4];                                                     // estimated direction of the gyroscope error
    float w_err[3];                                                         // estimated direction of the gyroscope error (angular)
    float h[3];                                                             // computed flux in the earth frame

    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * _quater[0];
    float halfSEq_2 = 0.5f * _quater[1];
    float halfSEq_3 = 0.5f * _quater[2];
    float halfSEq_4 = 0.5f * _quater[3];
    float twoSEq_1 = 2.0f * _quater[0];
    float twoSEq_2 = 2.0f * _quater[1];
    float twoSEq_3 = 2.0f * _quater[2];
    float twoSEq_4 = 2.0f * _quater[3];
    float twob_x = 2.0f * _b_x;
    float twob_z = 2.0f * _b_z;
    float twob_xSEq_1 = 2.0f * _b_x * _quater[0];
    float twob_xSEq_2 = 2.0f * _b_x * _quater[1];
    float twob_xSEq_3 = 2.0f * _b_x * _quater[2];
    float twob_xSEq_4 = 2.0f * _b_x * _quater[3];
    float twob_zSEq_1 = 2.0f * _b_z * _quater[0];
    float twob_zSEq_2 = 2.0f * _b_z * _quater[1];
    float twob_zSEq_3 = 2.0f * _b_z * _quater[2];
    float twob_zSEq_4 = 2.0f * _b_z * _quater[3];
    float SEq_1SEq_2;
    float SEq_1SEq_3 = _quater[0] * _quater[2];
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = _quater[1] * _quater[3];
    float SEq_3SEq_4;
    float twom_x = 2.0f * magn[0];
    float twom_y = 2.0f * magn[1];
    float twom_z = 2.0f * magn[2];

    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * _quater[3] - twoSEq_1 * _quater[2] - accel[0];
    f_2 = twoSEq_1 * _quater[1] + twoSEq_3 * _quater[3] - accel[1];
    f_3 = 1.0f - twoSEq_2 * _quater[1] - twoSEq_3 * _quater[2] - accel[2];
    f_4 = twob_x * (0.5f - _quater[2] * _quater[2] - _quater[3] * _quater[3]) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - magn[0];
    f_5 = twob_x * (_quater[1] * _quater[2] - _quater[0] * _quater[3]) + twob_z * (_quater[0] * _quater[1] + _quater[2] * _quater[3]) - magn[1];
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - _quater[1] * _quater[1] - _quater[2] * _quater[2]) - magn[2];
    J_11or24 = twoSEq_3;                                                    // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * _quater[3];
    J_13or22 = twoSEq_1;                                                    // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21;                                                 // negated in matrix multiplication
    J_33 = 2.0f * J_11or24;                                                 // negated in matrix multiplication
    J_41 = twob_zSEq_3;                                                     // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;                                // negated in matrix multiplication
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;                                // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2;                                       // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3;                                       // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;

    // compute the gradient (matrix multiplication)
    SEqHatDot[0] = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot[1] = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot[2] = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot[3] = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot[0] * SEqHatDot[0] + SEqHatDot[1] * SEqHatDot[1] + SEqHatDot[2] * SEqHatDot[2] + SEqHatDot[3] * SEqHatDot[3]);
    SEqHatDot[0] = SEqHatDot[0] / norm;
    SEqHatDot[1] = SEqHatDot[1] / norm;
    SEqHatDot[2] = SEqHatDot[2] / norm;
    SEqHatDot[3] = SEqHatDot[3] / norm;

    // compute angular estimated direction of the gyroscope error
    w_err[0] = twoSEq_1 * SEqHatDot[1] - twoSEq_2 * SEqHatDot[0] - twoSEq_3 * SEqHatDot[3] + twoSEq_4 * SEqHatDot[2];
    w_err[1] = twoSEq_1 * SEqHatDot[2] + twoSEq_2 * SEqHatDot[3] - twoSEq_3 * SEqHatDot[0] - twoSEq_4 * SEqHatDot[1];
    w_err[2] = twoSEq_1 * SEqHatDot[3] - twoSEq_2 * SEqHatDot[2] + twoSEq_3 * SEqHatDot[1] - twoSEq_4 * SEqHatDot[0];

    // compute and remove the gyroscope baises
    _w_b[0] += w_err[0] * dt * _zeta;
    _w_b[1] += w_err[1] * dt * _zeta;
    _w_b[2] += w_err[2] * dt * _zeta;
    gyro[0] -= _w_b[0];
    gyro[1] -= _w_b[1];
    gyro[2] -= _w_b[2];

    //Copy the corrected angular velocity
    _gyroDataFixed[0] = gyro[0];
    _gyroDataFixed[1] = gyro[1];
    _gyroDataFixed[2] = gyro[2];

    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega[0] = -halfSEq_2 * gyro[0] - halfSEq_3 * gyro[1] - halfSEq_4 * gyro[2];
    SEqDot_omega[1] = halfSEq_1 * gyro[0] + halfSEq_3 * gyro[2] - halfSEq_4 * gyro[1];
    SEqDot_omega[2] = halfSEq_1 * gyro[1] - halfSEq_2 * gyro[2] + halfSEq_4 * gyro[0];
    SEqDot_omega[3] = halfSEq_1 * gyro[2] + halfSEq_2 * gyro[1] - halfSEq_3 * gyro[0];

    // compute then integrate the estimated quaternion rate
    _quater[0] += (SEqDot_omega[0] - (_beta * SEqHatDot[0])) * dt;
    _quater[1] += (SEqDot_omega[1] - (_beta * SEqHatDot[1])) * dt;
    _quater[2] += (SEqDot_omega[2] - (_beta * SEqHatDot[2])) * dt;
    _quater[3] += (SEqDot_omega[3] - (_beta * SEqHatDot[3])) * dt;

    // normalise quaternion
    norm = sqrt(_quater[0] * _quater[0] + _quater[1] * _quater[1] + _quater[2] * _quater[2] + _quater[3] * _quater[3]);
    _quater[0] /= norm;
    _quater[1] /= norm;
    _quater[2] /= norm;
    _quater[3] /= norm;

    // compute flux in the earth frame
    SEq_1SEq_2 = _quater[0] * _quater[1];                                             // recompute axulirary variables
    SEq_1SEq_3 = _quater[0] * _quater[2];
    SEq_1SEq_4 = _quater[0] * _quater[3];
    SEq_3SEq_4 = _quater[2] * _quater[3];
    SEq_2SEq_3 = _quater[1] * _quater[2];
    SEq_2SEq_4 = _quater[1] * _quater[3];
    h[0] = twom_x * (0.5f - _quater[2] * _quater[2] - _quater[3] * _quater[3]) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h[1] = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - _quater[1] * _quater[1] - _quater[3] * _quater[3]) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h[2] = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - _quater[1] * _quater[1] - _quater[2] * _quater[2]);

    // normalise the flux vector to have only components in the x and z
    _b_x = sqrt((h[0] * h[0]) + (h[1] * h[1]));
    _b_z = h[2];
}


/* =====================================================================================
                 DATA MANAGMENT FUNCTION  (SETS AND GETS)
====================================================================================== */


void MadgwicksFilter :: quat2Euler(float quater[], float euler[]){

    //57.2957795131 = 180/PI for radians to degrees conversion.
    euler[0] = 57.2957795131*atan2(2*(quater[0]*quater[1] + quater[2]*quater[3]),quater[0]*quater[0] - quater[1]*quater[1] - quater[2]*quater[2] + quater[3]*quater[3]);
    euler[1] = 57.2957795131*asin(-2*((quater[0]*quater[2] - quater[1]*quater[3]))/(quater[0]*quater[0] + quater[1]*quater[1] + quater[2]*quater[2] + quater[3]*quater[3]));
    euler[2] = 57.2957795131*atan2(2*(quater[1]*quater[2] + quater[0]*quater[3]),-quater[0]*quater[0] - quater[1]*quater[1] + quater[2]*quater[2] + quater[3]*quater[3]);

    // //Roll
    // euler[0] = -57.2957795131*atan2(2*(quater[0]*quater[1] + quater[2]*quater[3]),1.0 - 2.0*(quater[1]*quater[1] + quater[2]*quater[2]));
    // //Pitch
    // euler[1] = -57.2957795131*asin(2*(quater[0]*quater[2] + quater[3]*quater[1]));
    // //Yaw
    // euler[2] = -57.2957795131*atan2(2*(quater[0]*quater[3] + quater[1]*quater[2]),1.0 - 2.0*(quater[2]*quater[2] + quater[3]*quater[3]));

}


void MadgwicksFilter :: getAccelData(int16_t accelerometerData[]){

    // This returns the current Accelerometer data
    accelerometerData[0] = _accelDataRaw[0];
    accelerometerData[1] = _accelDataRaw[1];
    accelerometerData[2] = _accelDataRaw[2];
}


void MadgwicksFilter :: getGyroData(int16_t gyroscopeData[]){

    // This returns the current Gyroscope data
    gyroscopeData[0] = _gyroDataRaw[0];
    gyroscopeData[1] = _gyroDataRaw[1];
    gyroscopeData[2] = _gyroDataRaw[2];
}

void MadgwicksFilter :: getMagnData(int16_t magnetometerData[]){

    // This returns the current Magnetometer data
    magnetometerData[0] = _magnDataRaw[0];
    magnetometerData[1] = _magnDataRaw[1];
    magnetometerData[2] = _magnDataRaw[2];
}

void MadgwicksFilter ::  setBeta(float beta) { _beta = beta; }

void MadgwicksFilter ::  setZeta(float zeta) { _zeta = zeta; }

void MadgwicksFilter :: getAttitude(float attitude[]){

    attitude[0] = _quater[0];
    attitude[1] = _quater[1];
    attitude[2] = _quater[2];
    attitude[3] = _quater[3];
}

void MadgwicksFilter :: getAttitudeEuler(float eulerAngle[]) { quat2Euler( _quater, eulerAngle); }


void MadgwicksFilter :: getAngularVelocity(float w[]){

    w[0] = _gyroDataFixed[0];
    w[1] = _gyroDataFixed[1];
    w[2] = _gyroDataFixed[2];
}


/* =====================================================================================
                              INITIALIZATION FUNCTIONS
====================================================================================== */

void MadgwicksFilter :: initialEstimation(float acceleration[], float magnetism[]){

    //This function provides a method for fast convergenvce of the Madgwicks Algorithm

    float down[] = {acceleration[0], acceleration[1], acceleration[2]};
    float east[3];
    float north[3];
    float norm;

    //East == cross product between down and magnetism
    east[0] = down[1] * magnetism[2] - down[2] * magnetism[1];
    east[1] = down[2] * magnetism[0] - down[0] * magnetism[2];
    east[2] = down[0] * magnetism[1] - down[1] * magnetism[0];

    //North == cross product between east and down
    north[0] = east[1] * down[2] - east[2] * down[1];
    north[1] = east[2] * down[0] - east[0] * down[2];
    north[2] = east[0] * down[1] - east[1] * down[0];

    //Normalize 'down' vector
    norm = sqrt(down[0] * down[0] + down[1] * down[1] + down[2] * down[2]);
    down[0] /= norm;
    down[1] /= norm;
    down[2] /= norm;

    //Normalize 'north' vector
    norm = sqrt(north[0] * north[0] + north[1] * north[1] + north[2] * north[2]);
    north[0] /= norm;
    north[1] /= norm;
    north[2] /= norm;

    //Normalize 'east' vector
    norm = sqrt(east[0] * east[0] + east[1] * east[1] + east[2] * east[2]);
    east[0] /= norm;
    east[1] /= norm;
    east[2] /= norm;

    //Now we take the matrix [[north], [east], [down]] and create a quaternion drom it.
    float tr = north[0] + east[1] + down[2];

    float S = 0.0;
    if (tr > 0) {
        S = sqrt(tr+1.0) * 2;
        _quater[0] = 0.25 * S;
        _quater[1] = (down[1] - east[2]) / S;
        _quater[2] = (north[2] - down[0]) / S;
        _quater[3] = (east[0] - north[1]) / S;
    }
    else if ((north[0] < east[1])&(north[0] < down[2])) {
        S = sqrt(1.0 + north[0] - east[1] - down[2]) * 2;
        _quater[0] = (down[1] - east[2]) / S;
        _quater[1] = 0.25 * S;
        _quater[2] = (north[1] + east[0]) / S;
        _quater[3] = (north[2] + down[0]) / S;
    }
    else if (east[1] < down[2]) {
        S = sqrt(1.0 + east[1] - north[0] - down[2]) * 2;
        _quater[0] = (north[2] - down[0]) / S;
        _quater[1] = (north[1] + east[0]) / S;
        _quater[2] = 0.25 * S;
        _quater[3] = (east[2] + down[1]) / S;
    }
    else {
        S = sqrt(1.0 + down[2] - north[0] - east[1]) * 2;
        _quater[0] = (east[0] - north[1]) / S;
        _quater[1] = (north[2] + down[0]) / S;
        _quater[2] = (east[2] + down[1]) / S;
        _quater[3] = 0.25 * S;
    }
}


void  MadgwicksFilter :: begin(void){

    //Initialization sequence of the filter object.

    // Initialize I2C BUS
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    //Sensor configuration
    accelSetup();
    gyroSetup();
    magnSetup();

    //Time variables initialization
    lastMicros = micros();

    //We make the first readings
    accelRead(_accelDataRaw, _accelData);
    gyroRead(_gyroDataRaw, _gyroData);
    magnRead(_magnDataRaw, _magnData);

    //Initialze the quaternion with a Guess
    initialEstimation(_accelData, _magnData);
}





void MadgwicksFilter :: step(void){


    // We read the sensors
    accelRead(_accelDataRaw, _accelData);
    gyroRead(_gyroDataRaw, _gyroData);
    magnRead(_magnDataRaw, _magnData);

    // Actualize time variables
    float dt = micros() - lastMicros;
    lastMicros = micros();

    // If not enough time has passed, don't iterate
    dt /= 1000000.0;
    if(dt == 0) return;

    filter(_gyroData, _accelData, _magnData, dt);
}
