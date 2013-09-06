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
    vector _accelVector;

    /**
     * Compass values as a vector.
     */
    vector _compassVector;

    /**
     * Gyroscope values as a vector.
     */
    vector _gyroVector;

    /**
     * Corrected gyro vector data.
     */
    vector _omegaVector;

    /**
     * Proportional correction.
     */
    vector _omegaP;

    /**
     * Omega integration.
     */
    vector _omegaI;

    /**
     * Omega result.
     */
    vector _omega;

    int _rawValues[6];

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
};

#endif

