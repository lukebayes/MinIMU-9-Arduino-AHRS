
#include <MinIMU9AHRS.h>

// Instantiate the AHRS
MinIMU9AHRS ahrs;

void setup()
{
  // Set up the AHRS with the I2C pins.
  ahrs.init();
};

void loop()
{
  // Loop the AHRS at system interval.
  EulerAngle euler = ahrs.getEuler();

  Serial.print("roll: ");
  Serial.print(euler.roll);
  Serial.print("pitch: ");
  Serial.print(euler.pitch);
  Serial.print("yaw: ");
  Serial.print(euler.yaw);
  Serial.println();
  delay(500);
};

