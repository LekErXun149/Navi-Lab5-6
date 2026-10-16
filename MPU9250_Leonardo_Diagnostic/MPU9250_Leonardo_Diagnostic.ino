// MPU9250 hardware check for Arduino Leonardo (no library needed)
// Wiring: VCC->5V, GND->GND, SDA->D2, SCL->D3   (NOT A4/A5 on Leonardo!)
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;   // 0x69 if AD0 is pulled high
const uint8_t MAG_ADDR = 0x0C;   // AK8963 magnetometer inside MPU9250

uint8_t readReg(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(dev, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}             // Leonardo: wait for Serial Monitor to open
  Wire.begin();
  delay(100);

  // ---- Step 1: I2C scan ----
  Serial.println(F("--- Step 1: I2C scan ---"));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Device found at 0x"));
      Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) {
    Serial.println(F("FAIL: nothing on I2C. Check SDA->D2, SCL->D3, VCC, GND."));
    while (1) {}
  }

  // ---- Step 2: WHO_AM_I ----
  Serial.println(F("--- Step 2: WHO_AM_I (reg 0x75) ---"));
  uint8_t who = readReg(MPU_ADDR, 0x75);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  if      (who == 0x71) Serial.println(F("OK: genuine MPU-9250"));
  else if (who == 0x73) Serial.println(F("OK: MPU-9255"));
  else if (who == 0x70) Serial.println(F("WARNING: MPU-6500 (no magnetometer)"));
  else                  Serial.println(F("FAIL: unexpected ID - check address/chip"));

  // ---- Step 3: wake up + magnetometer check ----
  writeReg(MPU_ADDR, 0x6B, 0x00);  // PWR_MGMT_1: wake from sleep
  delay(100);
  writeReg(MPU_ADDR, 0x37, 0x02);  // INT_PIN_CFG: I2C bypass -> expose AK8963
  delay(10);
  Serial.println(F("--- Step 3: Magnetometer AK8963 (expect 0x48) ---"));
  uint8_t wia = readReg(MAG_ADDR, 0x00);
  Serial.print(F("AK8963 WIA = 0x")); Serial.println(wia, HEX);
  Serial.println(wia == 0x48 ? F("OK: magnetometer responding")
                             : F("FAIL/ABSENT: no magnetometer response"));

  Serial.println(F("--- Step 4: live data ---"));
  Serial.println(F("ax_g ay_g az_g gx_dps gy_dps gz_dps temp_C"));
  delay(1000);
}

void loop() {
  // Read 14 bytes starting at ACCEL_XOUT_H (0x3B): accel xyz, temp, gyro xyz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();      // read separately: evaluation order matters
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }

  // Default ranges after reset: +/-2 g (16384 LSB/g), +/-250 dps (131 LSB/dps)
  Serial.print(v[0] / 16384.0, 3); Serial.print(' ');
  Serial.print(v[1] / 16384.0, 3); Serial.print(' ');
  Serial.print(v[2] / 16384.0, 3); Serial.print(' ');
  Serial.print(v[4] / 131.0, 2);   Serial.print(' ');
  Serial.print(v[5] / 131.0, 2);   Serial.print(' ');
  Serial.print(v[6] / 131.0, 2);   Serial.print(' ');
  Serial.println(v[3] / 333.87 + 21.0, 2);   // MPU9250 temp formula

  delay(100);
}
