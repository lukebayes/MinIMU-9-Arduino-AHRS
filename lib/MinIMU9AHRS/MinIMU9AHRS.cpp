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
#include <Wire.h>



/**
 * Library to wrap Pololu MinIMU9 v2 Inertial Measurement Unit (IMU).
 */
MinIMU9AHRS::MinIMU9AHRS()
{
};


/**
 * Initialize the AHRS.
 */
void MinIMU9AHRS::init(void)
{
  _initValues();
  _initGyro();
  _initAccelerometer();
  _initReadings();
};


/**
 * Initialize default instance values.
 */
void MinIMU9AHRS::_initValues(void)
{
  int _offsets[6] = {0, 0, 0, 0, 0, 0};

  // NOTE(lbayes): Invert the sign of any of these values to flip that axis
  // for the respective device. (gyro x, y, z, accel x, y, z, compass x, y, z).
  int _sensorDirection[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1};

  // Example of some inverted values.
  //int _sensorDirection[9] = {1, -1, -1, -1, 1, 1, 1, -1, -1};
};
  

/**
 * Initialize gyroscope.
 */
void MinIMU9AHRS::_initGyro()
{
  // TODO(lbayes): Expose configurable device addresses to external init
  // method.
  _gyroscope.init(L3GD20_DEVICE, L3GD20_ADDRESS_SA0_HIGH);
  _gyroscope.enableDefault();
};


/**
 * Initialize accelerometer.
 */
void MinIMU9AHRS::_initAccelerometer()
{
  _accelerometer.init(LSM303DLHC_DEVICE, ACC_ADDRESS_SA0_A_HIGH);
  _accelerometer.enableDefault();

  //_accelerometer.writeAccReg(LSM303_CTRL_REG1_A, 0x47); // normal power mode, all axes enabled, 50 Hz
  //_accelerometer.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode

  // NOTE(lbayes): Continuous conversion mode
  _accelerometer.writeMagReg(LSM303_MR_REG_M, 0x00);
};


void MinIMU9AHRS::_initReadings()
{
  lastReadingTime = millis();

  for (int i = 0; i < 32; i++) {
    _updateRawValues();
    for (int y = 0; y < 6; y++) {
      _offsets[y] += _rawValues[y];
    }
    delay(20);
  }
    
  for(int y = 0; y < 6; y++) {
    _offsets[y] = _offsets[y] / 32;
  }
    
  _offsets[5] -= GRAVITY * _sensorDirection[5];
  
  // Serial.println("Offset:");
  // for (int y = 0; y < 6; y++) {
    // Serial.println(_offsets[y]);
  // }
  
  // delay(2000);
  // delay(20);
  // counter = 0;
};

/**
 * Get the most recent Euler angle (roll, pitch and yaw) from the AHRS.
 */
EulerAngle MinIMU9AHRS::getEuler(void)
{
  updateReadings();
};


/**
 * Get the raw values from the AHRS.
 */
void MinIMU9AHRS::updateReadings(void)
{
  _updateRawValues();

  _gyroVector.x = _gyroscope.g.x;
  _gyroVector.y = _gyroscope.g.y;
  _gyroVector.z = _gyroscope.g.z;

  _accelVector.x = _accelerometer.a.x;
  _accelVector.y = _accelerometer.a.y;
  _accelVector.z = _accelerometer.a.z;

  _compassVector.x = _accelerometer.m.x;
  _compassVector.y = _accelerometer.m.y;
  _compassVector.z = _accelerometer.m.z;
};


void MinIMU9AHRS::_updateRawValues(void)
{
  _gyroscope.read();
  _accelerometer.readAcc();
  _accelerometer.readMag();

  _rawValues[0] = _gyroscope.g.x;
  _rawValues[1] = _gyroscope.g.y;
  _rawValues[2] = _gyroscope.g.z;
  _rawValues[3] = _accelerometer.a.x;
  _rawValues[4] = _accelerometer.a.y;
  _rawValues[5] = _accelerometer.a.z;
  _rawValues[6] = _accelerometer.m.x;
  _rawValues[7] = _accelerometer.m.y;
  _rawValues[8] = _accelerometer.m.z;
};

