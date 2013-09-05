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

#include <MinIMU9AHRS.h>
#include <LSM303.h>
#include <L3G.h>



/**
 * Library to wrap Pololu MinIMU9 v2 Inertial Measurement Unit (IMU).
 */
MinIMU9AHRS::MinIMU9AHRS()
{
  float _accelVector[3] = {0,0,0};
  float _gyroVector[3] = {0,0,0};
  float _omegaVector[3] = {0,0,0};
  float _omegaP[3] = {0,0,0};
  float _omegaI[3] = {0,0,0};
  float _omega[3] = {0,0,0};
};


/**
 * Initialize the AHRS.
 */
void MinIMU9AHRS::init(void)
{
};


/**
 * Get the most recent Euler angle (roll, pitch and yaw) from the AHRS.
 */
EulerAngle MinIMU9AHRS::getEuler(void)
{
};


/**
 * Get the raw values from the AHRS.
 */
IMUValues MinIMU9AHRS::getIMUValues(void)
{
};

