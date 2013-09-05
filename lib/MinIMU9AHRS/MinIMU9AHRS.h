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

/**
 * Interpreted Euler angle from the raw values.
 */
typedef struct EulerAngle {
  float roll, pitch, yaw;
};

/**
 * Raw values from the accelerometer, gyroscope and magnetometer.
 */
typedef struct IMUValues {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
};

class MinIMU9AHRS {
  public:
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
     * Get the raw values from the AHRS.
     */
    IMUValues getIMUValues(void);

  private:
    LSM303 _accelerometer;
    L3G _gyro;
};

#endif

