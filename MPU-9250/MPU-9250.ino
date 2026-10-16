#include <Wire.h>
#include <Mouse.h>
#include <math.h>

// MPU9250 I2C address
// Use 0x69 if your AD0 pin is HIGH
const byte MPU_ADDRESS = 0x68;

// MPU9250 registers
const byte PWR_MGMT_1  = 0x6B;
const byte CONFIG_REG  = 0x1A;
const byte GYRO_CONFIG = 0x1B;
const byte SMPLRT_DIV  = 0x19;
const byte GYRO_XOUT_H = 0x43;
const byte WHO_AM_I    = 0x75;

// Optional buttons
const byte ENABLE_PIN = 4;
const byte LEFT_PIN   = 7;
const byte RIGHT_PIN  = 8;

// Set to true if you connect an enable button
// Set to false to allow movement without a button
const bool USE_ENABLE_BUTTON = false;

// Gyroscope setting: ±250 degrees/second
const float GYRO_SENSITIVITY = 131.0;

// Adjust these values to control cursor speed
const float PIXELS_PER_DEGREE = 4.0;
const float DEAD_ZONE = 1.5;

// ---------------- Serial Monitor settings ----------------
// Set to false to turn off all Serial Monitor output
const bool SERIAL_DEBUG = true;
const unsigned long SERIAL_BAUD = 115200;
// How often live readings are printed (milliseconds)
const unsigned long PRINT_INTERVAL_MS = 100;

// Gyroscope bias
float biasY = 0;
float biasZ = 0;

// Cursor movement remainder
float cursorX = 0;
float cursorY = 0;

bool leftWasPressed = false;
bool rightWasPressed = false;


byte readRegister(byte reg)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  if (Wire.requestFrom(MPU_ADDRESS, (byte)1, true) != 1)
    return 0xFF;

  return Wire.read();
}


void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


bool readGyroscope(int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(GYRO_XOUT_H);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom(MPU_ADDRESS, (byte)6, true) != 6)
    return false;

  gyroX = (int16_t)((Wire.read() << 8) | Wire.read());
  gyroY = (int16_t)((Wire.read() << 8) | Wire.read());
  gyroZ = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}


void initialiseMPU9250()
{
  // Wake up the MPU9250
  writeRegister(PWR_MGMT_1, 0x01);

  // Low-pass filter
  writeRegister(CONFIG_REG, 0x03);

  // Sample rate
  writeRegister(SMPLRT_DIV, 0x07);

  // Gyroscope range: ±250 degrees/second
  writeRegister(GYRO_CONFIG, 0x00);

  delay(100);

  if (SERIAL_DEBUG)
  {
    byte id = readRegister(WHO_AM_I);
    Serial.print(F("WHO_AM_I = 0x"));
    Serial.print(id, HEX);

    if (id == 0x71)
      Serial.println(F("  (MPU9250 found)"));
    else if (id == 0x73)
      Serial.println(F("  (MPU9255 found)"));
    else if (id == 0x70)
      Serial.println(F("  (MPU6500 found)"));
    else if (id == 0xFF)
      Serial.println(F("  (NO RESPONSE - check wiring / I2C address)"));
    else
      Serial.println(F("  (unexpected ID - sensor may be a clone)"));
  }
}


void calibrateGyroscope()
{
  long sumY = 0;
  long sumZ = 0;
  int validSamples = 0;

  if (SERIAL_DEBUG)
    Serial.println(F("Calibrating gyroscope - keep the sensor still..."));

  // Keep the MPU9250 completely still
  delay(1000);

  for (int i = 0; i < 400; i++)
  {
    int16_t gx, gy, gz;

    if (readGyroscope(gx, gy, gz))
    {
      sumY += gy;
      sumZ += gz;
      validSamples++;
    }

    delay(5);
  }

  if (validSamples > 0)
  {
    biasY = (sumY / (float)validSamples) / GYRO_SENSITIVITY;
    biasZ = (sumZ / (float)validSamples) / GYRO_SENSITIVITY;
  }

  if (SERIAL_DEBUG)
  {
    Serial.print(F("Calibration done. Valid samples: "));
    Serial.print(validSamples);
    Serial.println(F(" / 400"));
    Serial.print(F("Bias Y: "));
    Serial.print(biasY, 3);
    Serial.print(F(" deg/s   Bias Z: "));
    Serial.print(biasZ, 3);
    Serial.println(F(" deg/s"));

    if (validSamples == 0)
      Serial.println(F("WARNING: no gyroscope readings received!"));
  }
}


float applyDeadZone(float value)
{
  if (fabs(value) < DEAD_ZONE)
    return 0;

  return value;
}


int getWholePixels(float &value)
{
  if (value > 127)
  {
    value -= 127;
    return 127;
  }

  if (value < -127)
  {
    value += 127;
    return -127;
  }

  int wholeValue = (int)value;
  value -= wholeValue;

  return wholeValue;
}


void updateMouseButtons()
{
  bool leftPressed = digitalRead(LEFT_PIN) == LOW;
  bool rightPressed = digitalRead(RIGHT_PIN) == LOW;

  if (leftPressed && !leftWasPressed)
  {
    Mouse.press(MOUSE_LEFT);
    if (SERIAL_DEBUG) Serial.println(F("Left button pressed"));
  }

  if (!leftPressed && leftWasPressed)
  {
    Mouse.release(MOUSE_LEFT);
    if (SERIAL_DEBUG) Serial.println(F("Left button released"));
  }

  if (rightPressed && !rightWasPressed)
  {
    Mouse.press(MOUSE_RIGHT);
    if (SERIAL_DEBUG) Serial.println(F("Right button pressed"));
  }

  if (!rightPressed && rightWasPressed)
  {
    Mouse.release(MOUSE_RIGHT);
    if (SERIAL_DEBUG) Serial.println(F("Right button released"));
  }

  leftWasPressed = leftPressed;
  rightWasPressed = rightPressed;
}


void releaseMouseButtons()
{
  if (leftWasPressed)
    Mouse.release(MOUSE_LEFT);

  if (rightWasPressed)
    Mouse.release(MOUSE_RIGHT);

  leftWasPressed = false;
  rightWasPressed = false;
}


void setup()
{
  if (SERIAL_DEBUG)
  {
    Serial.begin(SERIAL_BAUD);

    // Wait up to 3 seconds for the Serial Monitor to open
    // (so the sketch still runs when no computer monitor is open)
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) { }

    Serial.println();
    Serial.println(F("=== MPU9250 Air Mouse ==="));
  }

  pinMode(ENABLE_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);

  initialiseMPU9250();

  // Keep the sensor still during calibration
  calibrateGyroscope();

  Mouse.begin();

  if (SERIAL_DEBUG)
  {
    Serial.println(F("Mouse started."));
    Serial.println(F("Columns: rawY rawZ | gyroY gyroZ (deg/s) | dx dy | enabled"));
  }
}


void loop()
{
  static unsigned long previousTime = 0;

  unsigned long currentTime = micros();

  // Run at approximately 100 Hz
  if (currentTime - previousTime < 10000)
    return;

  if (previousTime == 0)
  {
    previousTime = currentTime;
    return;
  }

  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  bool motionEnabled =
    !USE_ENABLE_BUTTON || digitalRead(ENABLE_PIN) == LOW;

  static bool lastMotionEnabled = true;

  if (SERIAL_DEBUG && motionEnabled != lastMotionEnabled)
  {
    Serial.println(motionEnabled ? F("Motion ENABLED") : F("Motion DISABLED"));
    lastMotionEnabled = motionEnabled;
  }

  if (!motionEnabled)
  {
    releaseMouseButtons();
    cursorX = 0;
    cursorY = 0;
    return;
  }

  updateMouseButtons();

  int16_t rawX, rawY, rawZ;

  if (!readGyroscope(rawX, rawY, rawZ))
  {
    if (SERIAL_DEBUG)
    {
      static unsigned long lastErrorPrint = 0;
      if (millis() - lastErrorPrint >= 1000)
      {
        Serial.println(F("ERROR: failed to read gyroscope (I2C)"));
        lastErrorPrint = millis();
      }
    }
    return;
  }

  // Convert raw readings to degrees per second
  float gyroY = (rawY / GYRO_SENSITIVITY) - biasY;
  float gyroZ = (rawZ / GYRO_SENSITIVITY) - biasZ;

  gyroY = applyDeadZone(gyroY);
  gyroZ = applyDeadZone(gyroZ);

  // Z-axis rotation controls horizontal cursor movement
  // Y-axis rotation controls vertical cursor movement
  cursorX += gyroZ * dt * PIXELS_PER_DEGREE;
  cursorY += gyroY * dt * PIXELS_PER_DEGREE;

  int dx = getWholePixels(cursorX);
  int dy = getWholePixels(cursorY);

  if (dx != 0 || dy != 0)
  {
    Mouse.move(dx, -dy, 0);
  }

  // Print live readings at a limited rate so Serial doesn't slow the loop
  if (SERIAL_DEBUG)
  {
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= PRINT_INTERVAL_MS)
    {
      lastPrint = millis();

      Serial.print(F("raw Y: "));
      Serial.print(rawY);
      Serial.print(F("  Z: "));
      Serial.print(rawZ);
      Serial.print(F("  | gyro Y: "));
      Serial.print(gyroY, 2);
      Serial.print(F("  Z: "));
      Serial.print(gyroZ, 2);
      Serial.print(F("  | dx: "));
      Serial.print(dx);
      Serial.print(F("  dy: "));
      Serial.print(-dy);
      Serial.print(F("  | enabled: "));
      Serial.println(motionEnabled ? F("yes") : F("no"));
    }
  }
}