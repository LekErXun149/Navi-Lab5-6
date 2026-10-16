
#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;

// false = Serial Monitor with labels
// true  = MATLAB data format
const bool matlabMode = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 not detected!");
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  if (mpu.update()) {

    float roll = mpu.getRoll();
    float pitch = mpu.getPitch();
    float yaw = mpu.getYaw();

    if (matlabMode) {

      // Send numbers only for MATLAB
      Serial.print(roll, 2);
      Serial.print(",");
      Serial.print(pitch, 2);
      Serial.print(",");
      Serial.println(yaw, 2);

    } else {

      // Display labels in Serial Monitor
      Serial.print("Roll: ");
      Serial.print(roll, 2);
      Serial.print(" deg");

      Serial.print(" | Pitch: ");
      Serial.print(pitch, 2);
      Serial.print(" deg");

      Serial.print(" | Yaw: ");
      Serial.print(yaw, 2);
      Serial.println(" deg");
    }
  }

  delay(20);
}
