// Orientation (NED convention) for MPU-6500 (MPU9250 clone, no magnetometer)
// Arduino Leonardo / Pro Micro.   Wiring: VCC->5V, GND->GND, SDA->D2, SCL->D3
// Output packet for MATLAB getData.m: 111,yaw,pitch,roll,temp,FFF   (20 Hz)
//
// NED CONVENTION
//   Body frame : X = forward (nose), Y = right, Z = down
//   Euler (ZYX): roll  +  = right side down
//                pitch +  = nose up
//                yaw   +  = clockwise seen from above (heading)
//   At rest, level: accelerometer reads (0, 0, -1 g)
//
// AUTOMATIC MOUNTING CALIBRATION (works however the MPU is plugged in):
//   At power-up / reset, hold the breadboard LEVEL with the VCC end of the MPU
//   header row pointing FORWARD, and keep it STILL for ~2 s.
//   That pose becomes roll = 0, pitch = 0, yaw = 0.
//
//   pitch/roll : complementary filter (gyro + accelerometer) -> stable
//   yaw        : gyro integration only (no magnetometer) -> drifts slowly
#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;
const float ALPHA   = 0.98;        // complementary filter weight on gyro
const float RAD2DEG = 57.2957795;
const float DEG2RAD = 0.01745329;

float M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // sensor -> NED body (rows: fwd, right, down)
float gBias[3] = {0, 0, 0};                          // gyro bias, sensor frame
float roll = 0, pitch = 0, yaw = 0;                  // degrees
uint32_t lastMicros;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Raw sensor-frame readings: accel (g), gyro (deg/s), temp (C)
void readRaw(float a[3], float g[3], float &t) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  a[0] = v[0] / 16384.0;  a[1] = v[1] / 16384.0;  a[2] = v[2] / 16384.0;  // +/-2 g
  t    = v[3] / 333.87 + 21.0;
  g[0] = v[4] / 131.0;    g[1] = v[5] / 131.0;    g[2] = v[6] / 131.0;    // +/-250 dps
}

void rotateToBody(const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
}

float norm3(const float v[3]) { return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

float wrap180(float x) {
  while (x > 180)  x -= 360;
  while (x < -180) x += 360;
  return x;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(400000);

  writeReg(0x6B, 0x00);   // wake up
  delay(100);
  writeReg(0x1A, 0x03);   // gyro digital low-pass ~41 Hz
  writeReg(0x1B, 0x00);   // gyro +/-250 dps
  writeReg(0x1C, 0x00);   // accel +/-2 g
  writeReg(0x1D, 0x03);   // accel digital low-pass ~44 Hz
  delay(500);             // time to let go of the reset button

  Serial.println(F("Calibrating: hold LEVEL, VCC end FORWARD, keep STILL..."));
  const int N = 500;
  float aSum[3] = {0, 0, 0}, gSum[3] = {0, 0, 0}, a[3], g[3], t;
  for (int n = 0; n < N; n++) {
    readRaw(a, g, t);
    for (int i = 0; i < 3; i++) { aSum[i] += a[i]; gSum[i] += g[i]; }
    delay(3);
  }
  for (int i = 0; i < 3; i++) gBias[i] = gSum[i] / N;

  // DOWN = opposite of the measured specific force (accel reads +1 g "up" at rest)
  float dn[3] = {-aSum[0] / N, -aSum[1] / N, -aSum[2] / N};
  float nn = norm3(dn);
  for (int i = 0; i < 3; i++) dn[i] /= nn;

  // FORWARD = sensor +Y (toward VCC pin) projected onto the horizontal plane.
  // If the VCC axis is (nearly) vertical, fall back to the printed X arrow.
  float f[3] = {0, 1, 0};
  float d = f[0] * dn[0] + f[1] * dn[1] + f[2] * dn[2];
  for (int i = 0; i < 3; i++) f[i] -= d * dn[i];
  if (norm3(f) < 0.3) {
    f[0] = 1; f[1] = 0; f[2] = 0;
    d = dn[0];
    for (int i = 0; i < 3; i++) f[i] -= d * dn[i];
    Serial.println(F("Note: VCC end was vertical - using the X arrow as forward."));
  }
  float fn = norm3(f);
  for (int i = 0; i < 3; i++) f[i] /= fn;

  // RIGHT = DOWN x FORWARD  (right-handed NED body: fwd x right = down)
  float r[3] = {dn[1] * f[2] - dn[2] * f[1],
                dn[2] * f[0] - dn[0] * f[2],
                dn[0] * f[1] - dn[1] * f[0]};

  for (int j = 0; j < 3; j++) { M[0][j] = f[j]; M[1][j] = r[j]; M[2][j] = dn[j]; }

  roll = pitch = yaw = 0;     // reference pose: level, heading 0
  lastMicros = micros();
}

void loop() {
  float a[3], g[3], t, fb[3], w[3];
  readRaw(a, g, t);
  for (int i = 0; i < 3; i++) g[i] -= gBias[i];
  rotateToBody(a, fb);        // specific force in NED body frame (level: 0,0,-1)
  rotateToBody(g, w);         // body rates p, q, r (deg/s)
  float p = w[0], q = w[1], rr = w[2];

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;

  // ---- Tilt from gravity (NED) ----
  float accRoll  = atan2(-fb[1], -fb[2]) * RAD2DEG;
  float accPitch = atan2(fb[0], sqrt(fb[1] * fb[1] + fb[2] * fb[2])) * RAD2DEG;

  // ---- Euler angle rates from body rates (ZYX kinematics) ----
  float sr = sin(roll * DEG2RAD),  cr = cos(roll * DEG2RAD);
  float sp = sin(pitch * DEG2RAD), cp = cos(pitch * DEG2RAD);
  if (fabs(cp) < 0.05) cp = (cp >= 0) ? 0.05 : -0.05;   // avoid /0 at pitch = +/-90
  float rollDot  = p + (q * sr + rr * cr) * sp / cp;
  float pitchDot = q * cr - rr * sr;
  float yawDot   = (q * sr + rr * cr) / cp;

  // ---- Complementary filter (wrap-safe for roll) ----
  float rollG  = roll  + rollDot  * dt;
  float pitchG = pitch + pitchDot * dt;
  roll  = wrap180(rollG + (1 - ALPHA) * wrap180(accRoll - rollG));
  pitch = pitchG + (1 - ALPHA) * (accPitch - pitchG);

  // ---- Yaw: gyro only (drifts without magnetometer) ----
  yaw = wrap180(yaw + yawDot * dt);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 50) {   // ~20 Hz output
    lastPrint = millis();

    // Column header row, printed whenever a program opens the port
    static bool wasConnected = false;
    bool connected = Serial;
    if (connected && !wasConnected) {
      Serial.println(F("start,yaw_deg,pitch_deg,roll_deg,temp_C,end   (NED: X fwd, Y right, Z down)"));
    }
    wasConnected = connected;

    // Packet for MATLAB getData.m:  111,yaw,pitch,roll,temp,FFF
    Serial.print(F("111,"));
    Serial.print(yaw, 2);   Serial.print(',');
    Serial.print(pitch, 2); Serial.print(',');
    Serial.print(roll, 2);  Serial.print(',');
    Serial.print(t, 2);
    Serial.println(F(",FFF"));
  }
}
