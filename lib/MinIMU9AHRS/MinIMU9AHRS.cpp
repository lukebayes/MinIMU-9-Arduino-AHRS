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
MinIMU9AHRS::MinIMU9AHRS() {};


/**
 * Initialize the AHRS.
 */
void MinIMU9AHRS::init(void)
{
  Serial.println("init minimu");
  _initValues();
  _initGyro();
  _initAccelerometer();
  _initOffsets();
};


/**
 * Initialize default instance values.
 */
void MinIMU9AHRS::_initValues(void)
{

  // TODO(lbayes): Figure out how to initialize this matrix.
  // float _dcmMatrix[3][3] = {
    // {1, 0, 0}, 
    // {0, 1, 0},
    // {0, 0, 1}
  // }; 

  _integrationTime = 0.02;
  _minGyroAndAccelTimeoutMillis = DEFAULT_MIN_TIMEOUT_MILLIS;
  _lastReadingTime = -_minGyroAndAccelTimeoutMillis;

  _offsets[0] = 0; // gyro x offset
  _offsets[1] = 0; // gyro y offset
  _offsets[2] = 0; // gyro z offset
  _offsets[3] = 0; // accel x offset
  _offsets[4] = 0; // accel y offset
  _offsets[5] = 0; // accell z offset

  // NOTE(lbayes): Invert the sign of any of these values to invert that axis
  // for the respective device.
  _sensorDirection[0] = 1; // gyro x
  _sensorDirection[1] = 1; // gyro y
  _sensorDirection[2] = 1; // gyro z
  _sensorDirection[3] = -1; // accel x
  _sensorDirection[4] = -1; // accel y
  _sensorDirection[5] = -1; // accel z
  _sensorDirection[6] = 1; // mag x
  _sensorDirection[7] = 1; // mag y
  _sensorDirection[8] = 1; // mag z
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


/**
 * Initialize default offsets for each sensor.
 */
void MinIMU9AHRS::_initOffsets()
{
  // NOTE(lbayes): This feature makes me sad because it increases system start
  // time.
  for (int i = 0; i < 32; i++) {
    updateReadings();
    for (int y = 0; y < 6; y++) {
      _offsets[y] += _rawValues[y];
    }
    delay(20);
  }
    
  for(int y = 0; y < 6; y++) {
    _offsets[y] = _offsets[y] / 32;
  }
    
  _offsets[5] -= GRAVITY * _sensorDirection[5];

  _isInitialized = true;
};

/**
 * Get the most recent Euler angle (roll, pitch and yaw) from the AHRS.
 */
EulerAngle MinIMU9AHRS::getEuler(void)
{
  updateReadings();

  EulerAngle euler = {};
  euler.roll = _accelValue.x;
  euler.pitch = _accelValue.y;
  euler.yaw = _accelValue.z;

  return euler;
};


/**
 * Get the raw values from the AHRS.
 */
void MinIMU9AHRS::updateReadings(void)
{
  // NOTE(lbayes): We have a problem with timing here. We want to collect 
  // readings from the gyro and accelerometer at one maximum rate, and
  // from the compass at a different (slower) maximum rate.

  _currentReadingTime = millis();

  unsigned long millisecondsSinceLastReading = _currentReadingTime - _lastReadingTime;

  // NOTE(lbayes): Do not reach down to the hardware too frequently.
  if (_isInitialized && millisecondsSinceLastReading < _minGyroAndAccelTimeoutMillis) {
    Serial.println("bailing");
    return;
  }

  _secondsSinceLastReading = millisecondsSinceLastReading / 1000.0;

  _readGyro();
  _readAccelerometer();
  _readCompass();

  _lastReadingTime = _currentReadingTime;
};


/**
 * Read the gyro and update values accordingly.
 */
void MinIMU9AHRS::_readGyro(void)
{
  _gyroscope.read();
  _rawValues[0] = _gyroscope.g.x;
  _rawValues[1] = _gyroscope.g.y;
  _rawValues[2] = _gyroscope.g.z;

  _gyroValue.x = _sensorDirection[0] * (_rawValues[0] - _offsets[0]);
  _gyroValue.y = _sensorDirection[1] * (_rawValues[1] - _offsets[1]);
  _gyroValue.z = _sensorDirection[2] * (_rawValues[2] - _offsets[2]);
};


/**
 * Read the accelerometer and update values accordingly.
 */
void MinIMU9AHRS::_readAccelerometer(void)
{
  _accelerometer.readAcc();
  _rawValues[3] = _accelerometer.a.x;
  _rawValues[4] = _accelerometer.a.y;
  _rawValues[5] = _accelerometer.a.z;

  _accelValue.x = _sensorDirection[3] * (_rawValues[3] - _offsets[3]);
  _accelValue.y = _sensorDirection[4] * (_rawValues[4] - _offsets[4]);
  _accelValue.z = _sensorDirection[5] * (_rawValues[5] - _offsets[5]);
};


/**
 * Read the compass and update values accordingly.
 */
void MinIMU9AHRS::_readCompass(void)
{
  _accelerometer.readMag();
  _rawValues[6] = _accelerometer.m.x;
  _rawValues[7] = _accelerometer.m.y;
  _rawValues[8] = _accelerometer.m.z;

  _compassValue.x = _sensorDirection[6] * _rawValues[6];
  _compassValue.y = _sensorDirection[7] * _rawValues[7];
  _compassValue.z = _sensorDirection[8] * _rawValues[8];
};


void MinIMU9AHRS::_matrixUpdate(void)
{
  _gyroVector[0] = Gyro_Scaled_X(_gyroValue.x); //gyro x roll
  _gyroVector[1] = Gyro_Scaled_Y(_gyroValue.y); //gyro y pitch
  _gyroVector[2] = Gyro_Scaled_Z(_gyroValue.z); //gyro Z yaw
  
  _accelVector[0] = _accelValue.x;
  _accelVector[1] = _accelValue.y;
  _accelVector[2] = _accelValue.z;
    
  _vectorAdd(&_omega[0], &_gyroVector[0], &_omegaI[0]);  //adding proportional term
  _vectorAdd(&_omegaVector[0], &_omega[0], &_omegaP[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 #if OUTPUTMODE == 1         
  _updateMatrix[0][0] = 0;
  _updateMatrix[0][1] = -_integrationTime * _omegaVector[2]; //-z
  _updateMatrix[0][2] = _integrationTime * _omegaVector[1]; //y
  _updateMatrix[1][0] = _integrationTime * _omegaVector[2]; //z
  _updateMatrix[1][1] = 0;
  _updateMatrix[1][2] = -_integrationTime * _omegaVector[0]; //-x
  _updateMatrix[2][0] = -_integrationTime * _omegaVector[1]; //-y
  _updateMatrix[2][1] = _integrationTime * _omegaVector[0]; //x
  _updateMatrix[2][2] = 0;
 #else // Uncorrected data (no drift correction)
  _updateMatrix[0][0] = 0;
  _updateMatrix[0][1] = -_integrationTime * _gyroVector[2]; //-z
  _updateMatrix[0][2] = _integrationTime * _gyroVector[1]; //y
  _updateMatrix[1][0] = _integrationTime * _gyroVector[2]; //z
  _updateMatrix[1][1] = 0;
  _updateMatrix[1][2] = -_integrationTime * _gyroVector[0];
  _updateMatrix[2][0] = -_integrationTime * _gyroVector[1];
  _updateMatrix[2][1] = _integrationTime * _gyroVector[0];
  _updateMatrix[2][2] = 0;
 #endif

  _matrixMultiply(_dcmMatrix, _updateMatrix, _tempMatrix);

  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      _dcmMatrix[x][y] += _tempMatrix[x][y];
    } 
  }
};


/**
 * Multiply two 3x3 matrixs. This function developed by Jordi can be easily
 * adapted to multiple n*n matrix's. (Pero me da flojera!). 
 */
void MinIMU9AHRS::_matrixMultiply(float a[3][3], float b[3][3],
    float mat[3][3])
{
  float op[3]; 
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      for (int w = 0; w < 3; w++) {
       op[w] = a[x][w] * b[w][y];
      } 
      mat[x][y] = 0;
      mat[x][y] = op[0] + op[1] + op[2];
      float test = mat[x][y];
    }
  }
}


/**
 * Compute the dot product of two vectors and put the result into vectorOut.
 */
float MinIMU9AHRS::_vectorDotProduct(float vector1[3],float vector2[3])
{
  float op = 0;
  for(int c = 0; c < 3; c++) {
    op += vector1[c] * vector2[c];
  }
  return op; 
};

/**
 * Compute the cross product of two vectors and put the result into vectorOut.
 */
void MinIMU9AHRS::_vectorCrossProduct(float vectorOut[3], float v1[3],
    float v2[3])
{
  vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
};


/**
 * Multiply the provided vector by a scalar and put result into vectorOut.
 */
void MinIMU9AHRS::_vectorScale(float vectorOut[3], float vectorIn[3],
    float scale2)
{
  for(int c = 0; c < 3; c++) {
    vectorOut[c] = vectorIn[c] * scale2; 
  }
};

/**
 * Add the povided vectors and put result into vectorOut.
 */
void MinIMU9AHRS::_vectorAdd(float vectorOut[3], float vectorIn1[3],
    float vectorIn2[3])
{
  for(int c = 0; c < 3; c++) {
     vectorOut[c] = vectorIn1[c] + vectorIn2[c];
  }
};

