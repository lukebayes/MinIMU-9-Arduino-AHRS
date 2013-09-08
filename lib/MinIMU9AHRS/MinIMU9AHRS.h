/*
MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MinIMU9AHRS_h
#define MinIMU9AHRS_h

#include "LSM303.h"
#include "L3G.h"

#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// LSM303 accelerometer: 8g sensitivity
// 3.8 mg/digit; 1 g = 256
// This is equivalent to 1G in the raw data coming from the accelerometer.
#define GRAVITY 256

// Minimum timeout in milliseconds that must elapse between readings.
#define DEFAULT_MIN_TIMEOUT_MILLIS 10

// Convert provided value to radians
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
// Convert provided value to degrees
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 // X axis Gyro gain
#define Gyro_Gain_Y 0.07 // Y axis Gyro gain
#define Gyro_Gain_Z 0.07 // Z axis Gyro gain
// Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) 
// Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) 
// Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) 

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

#define OUTPUTMODE 1

/**
 * Interpreted Euler angle from the raw values.
 */
typedef struct EulerAngle {
  float roll, pitch, yaw;
};


class MinIMU9AHRS {
  public:
    typedef struct vector
    {
      float x, y, z;
    };

    /**
     * The Attitude, Heading Reference System.
     */
    MinIMU9AHRS();
    
    /**
     * Initialize the AHRS.
     */
    void init(void);

    /**
     * Get the most recent Euler angle (roll, pitch and yaw) from the AHRS.
     */
    EulerAngle getEuler(void);

    /**
     * Update the readings from all inputs.
     */
    void updateReadings(void);

  private:

    /**
     * Initialize default instance values.
     */
    void _initValues(void);
    
    /**
     * Initialize gyroscope.
     */
    void _initGyro(void);

    /**
     * Initialize accelerometer.
     */
    void _initAccelerometer(void);

    /**
     * Initialize readings.
     */
    void _initOffsets(void);

    /**
     * Read the gyroscope and update values accordingly.
     */
    void _readGyro(void);

    /**
     * Read the accelerometer and update values accordingly.
     */
    void _readAccelerometer(void);

    /**
     * Read the compass and update values accordingly.
     */
    void _readCompass(void);

    /**
     * Update the compass heading after reading values.
     */
    void _updateCompassHeading(void);

    /**
     * Update the Euler angles.
     */
    void _updateEulerAngles(void);

    /**
     * Multiply two 3x3 matrixs. This function developed by Jordi can be easily
     * adapted to multiple n*n matrix's. (Pero me da flojera!). 
     */
    void _matrixMultiply(float a[3][3], float b[3][3], float mat[3][3]);

    /**
     * Update the data matrices.
     */
    void _matrixUpdate(void);

    /**
     * Compute the dot product of two vectors and put the result into vectorOut.
     */
    float _vectorDotProduct(float vector1[3],float vector2[3]);

    /**
     * Compute the cross product of two vectors and put the result into vectorOut.
     */
    void _vectorCrossProduct(float vectorOut[3], float v1[3], float v2[3]);

    /**
     * Multiply the provided vector by a scalar and put result into vectorOut.
     */
    void _vectorScale(float vectorOut[3], float vectorIn[3], float scale2);

    /**
     * Add the povided vectors and put result into vectorOut.
     */
    void _vectorAdd(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);

    /**
     * Normalize the matrices.
     */
    void _normalize(void);

    /**
     * Correct matrices for drift.
     */
    void _driftCorrection(void);

    /**
     * Accelerometer instance.
     */
    LSM303 _accelerometer;

    /**
     * Gyroscope instance.
     */
    L3G _gyroscope;

    /**
     * Accelerometer values as a vector.
     */
    vector _accelValue;

    /**
     * Accelerometer values as a vector.
     */
    float _accelVector[3];

    /**
     * Compass values as a vector.
     */
    vector _compassValue;

    /**
     * Compass values as a vector.
     */
    float _compassVector[3];

    /**
     * Gyroscope values as a vector.
     */
    vector _gyroValue;

    /**
     * Gyroscope values as a vector.
     */
    float _gyroVector[3];

    /**
     * Corrected gyro vector data.
     */
    float _omegaVector[3];

    /**
     * Proportional correction.
     */
    float _omegaP[3];

    /**
     * Omega integration.
     */
    float _omegaI[3];

    /**
     * Omega result.
     */
    float _omega[3];

    float _rawValues[6];

    /**
     * Array that stores the offsets of the sensors.
     */
    int _offsets[6];

    /**
     * Array that indicates the direction (or sign) of each axis for each
     * of the sensors (Gyro and Accelerometer).
     */
    int _sensorDirection[9];

    /**
     * Time of last reading in milliseconds.
     */
    unsigned long _lastReadingTime;

    /**
     * Time of current reading in milliseconds.
     */
    unsigned long _currentReadingTime;

    /**
     * Seconds since last reading.
     */
    unsigned long _secondsSinceLastReading;

    /**
     * Minimum reading timeout in milliseconds.
     */
    int _minGyroAndAccelTimeoutMillis;

    /**
     * True if the readings have finished initialization.
     */
    bool _isInitialized;

    /**
     * Integration time in seconds for the DCM algorithm. We will run the
     * integration loop at 50Hz if possible.
     */
    float _integrationTime;

    /**
     * Matrix for DCM values.
     */
    float _dcmMatrix[3][3];

    /**
     * Temporary matrix to use for multiplication.
     */
    float _tempMatrix[3][3];

    /**
     * Matrix to update.
     *
     * NOTE(lbayes): These values were globally assigned before:
     * {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
     */
    float _updateMatrix[3][3];

    /**
     * Error roll pitch.
     */
    float _errorRollPitch[3];

    /**
     * Error yaw.
     */
    float _errorYaw[3];

    /**
     * Magnetometer heading.
     */
    float _magHeading;

    /**
     * The current Euler angle of the device (roll, pitch and yaw).
     */
    EulerAngle _euler;
};

#endif

