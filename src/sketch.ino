
#include <MinIMU9AHRS.h>
#include <Wire.h>

// Instantiate the AHRS
MinIMU9AHRS ahrs;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  // Set up the AHRS with the I2C pins.
  ahrs.init();
  Serial.println("SETUP COMPLETE");
};

void loop()
{
  ahrs.updateReadings();
  EulerAngle euler = ahrs.getEuler();

  // Print the Euler values:
  Serial.print("roll: ");
  Serial.print(euler.roll);
  Serial.print(" pitch: ");
  Serial.print(euler.pitch);
  Serial.print(" yaw: ");
  Serial.print(euler.yaw);
  Serial.println();

  delay(100);
};

