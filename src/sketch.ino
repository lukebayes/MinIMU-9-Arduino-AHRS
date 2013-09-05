
#include <MinIMU9AHRS.h>
#include <Wire.h>

// Instantiate the AHRS
MinIMU9AHRS ahrs;

void setup()
{
  // Set up the AHRS with the I2C pins.
  ahrs.init();
};

void loop()
{
  Serial.begin(115200);
  Wire.begin();

  EulerAngle euler = ahrs.getEuler();

  // Print the Euler values:
  Serial.print("roll: ");
  Serial.print(euler.roll);
  Serial.print("pitch: ");
  Serial.print(euler.pitch);
  Serial.print("yaw: ");
  Serial.print(euler.yaw);
  Serial.println();

  delay(500);
};

