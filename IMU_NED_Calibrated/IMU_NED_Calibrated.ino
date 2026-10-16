// =====================================================================
//  IMU_NED_Calibrated.ino
//  RSE4501 Mini Project 2 - calibrated orientation (NED) for
//  Arduino Leonardo + GY-9250 module (MPU-6500 chip, no magnetometer)
//
//  Wiring: VCC->5V, GND->GND, SDA->D2, SCL->D3
//
//  What this sketch does
//   1. Reads raw accel + gyro from the MPU-6500 (I2C, 0x68)
//   2. Applies the six-position ACCELEROMETER CALIBRATION
//        a_cal = (a_raw - bias) / scale        (constants below)
//   3. Measures GYRO BIAS at start-up (board must be still ~2 s)
//   4. Rotates sensor axes into the NED body frame
//        X = forward (VCC end of header), Y = right, Z = down
//   5. Runs TWO complementary filters side by side:
//        - CALIBRATED  (used for the normal output)
//        - RAW/UNCALIBRATED (for the before/after verification test)
//   6. Sends packets to MATLAB (getData.m) at 20 Hz
//
//  NED angle convention (ZYX Euler)
//     roll  +  = right side down      (range -180..180)
//     pitch +  = nose (VCC end) up    (range  -90..90)
//     yaw   +  = clockwise seen from above (gyro only -> drifts)
//
//  Output packets
//   normal mode   : 111,yaw,pitch,roll,temp,FFF
//   verify mode   : 111,yaw,pitch,roll,temp,pitch_raw,roll_raw,g_cal,g_raw,FFF
//                   (yaw/pitch/roll = calibrated, *_raw = no accel calibration,
//                    g_* = gravity magnitude |a| in g, should be 1.000 at rest)
//
//  Serial commands (Serial Monitor "Newline", or MATLAB writeline):
//     v1 / v0   verification output on / off
//     c1 / c0   use calibration for the MAIN output on / off
//     z         zero yaw
//     b         re-measure gyro bias (keep the board still ~2 s)
//     i         print settings and calibration constants
// =====================================================================
#include <Wire.h>
#include <math.h>

// ---------------- USER SETTINGS ----------------
// 0 = FIXED mounting: body frame = sensor frame mapped to NED
//     (MPU flat, component side up, VCC end = forward).
//     Use 0 for the CALIBRATION VERIFICATION TEST: angles are absolute,
//     so calibration errors are not hidden.
// 1 = AUTO-LEVEL at start-up: the pose at reset becomes roll = pitch = 0
//     (any mounting). Use 1 for the APPLICATION.
#define AUTO_LEVEL 0

// ---------------- CALIBRATION CONSTANTS ----------------
// From six-position test (MPU_Data_calibration.xlsx, sheet "Calibration")
// Sensor axes as printed on the board: X away from pins, Y toward VCC, Z up
const float ACC_BIAS[3]  = { 0.001665f, -0.008621f,  0.010054f };  // g
const float ACC_SCALE[3] = { 0.998615f,  0.999471f,  1.006751f };  // -

// ---------------- SENSOR / FILTER CONSTANTS ----------------
const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB = 16384.0f;   // LSB per g     (+/-2 g)
const float GYR_LSB = 131.0f;     // LSB per deg/s (+/-250 deg/s)
const float ALPHA   = 0.98f;      // complementary filter weight on gyro
const float RAD2DEG = 57.2957795f;
const float DEG2RAD = 0.01745329f;
const uint16_t OUT_PERIOD_MS = 50; // 20 Hz output

// ---------------- STATE ----------------
float M[3][3];                     // sensor -> NED body rotation
float gBias[3] = {0, 0, 0};        // gyro bias (sensor frame, deg/s)
struct Att { float roll, pitch, yaw; };
Att attCal = {0, 0, 0};            // calibrated filter
Att attRaw = {0, 0, 0};            // uncalibrated filter
bool useCal = true;                // main output uses calibration
bool verifyMode = false;           // extended packet
uint32_t lastMicros;
uint32_t badReads = 0;
String cmd = "";

// =====================================================================
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Raw read. Returns false for an invalid sample (I2C dropout = all zeros).
bool readRaw(float a[3], float g[3], float &t) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t v[7];
  bool allZero = true;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
    if (i != 3 && v[i] != 0) allZero = false;
  }
  if (allZero) return false;
  a[0] = v[0] / ACC_LSB;  a[1] = v[1] / ACC_LSB;  a[2] = v[2] / ACC_LSB;
  t    = v[3] / 333.87f + 21.0f;
  g[0] = v[4] / GYR_LSB;  g[1] = v[5] / GYR_LSB;  g[2] = v[6] / GYR_LSB;
  return true;
}

void applyAccCal(const float a[3], float out[3]) {
  for (int i = 0; i < 3; i++) out[i] = (a[i] - ACC_BIAS[i]) / ACC_SCALE[i];
}

void toBody(const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
}

float norm3(const float v[3]) { return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }

float wrap180(float x) {
  while (x > 180)  x -= 360;
  while (x < -180) x += 360;
  return x;
}

// Tilt from gravity (NED). f = specific force in body frame (level: 0,0,-1)
void accAngles(const float f[3], float &roll, float &pitch) {
  roll  = atan2(-f[1], -f[2]) * RAD2DEG;
  pitch = atan2(f[0], sqrt(f[1]*f[1] + f[2]*f[2])) * RAD2DEG;
}

// One complementary-filter step with ZYX Euler kinematics
void filterStep(Att &s, const float f[3], const float w[3], float dt) {
  float accRoll, accPitch;
  accAngles(f, accRoll, accPitch);
  float p = w[0], q = w[1], r = w[2];
  float sr = sin(s.roll * DEG2RAD),  cr = cos(s.roll * DEG2RAD);
  float sp = sin(s.pitch * DEG2RAD), cp = cos(s.pitch * DEG2RAD);
  if (fabs(cp) < 0.05f) cp = (cp >= 0) ? 0.05f : -0.05f;   // near gimbal lock
  float rollDot  = p + (q * sr + r * cr) * sp / cp;
  float pitchDot = q * cr - r * sr;
  float yawDot   = (q * sr + r * cr) / cp;
  float rollG  = s.roll  + rollDot  * dt;
  float pitchG = s.pitch + pitchDot * dt;
  s.roll  = wrap180(rollG + (1 - ALPHA) * wrap180(accRoll - rollG));
  s.pitch = pitchG + (1 - ALPHA) * (accPitch - pitchG);
  s.yaw   = wrap180(s.yaw + yawDot * dt);
}

// ---------------- Start-up: gyro bias + body frame ----------------
void measureBiasAndFrame(bool buildFrame) {
  Serial.println(F("# Keep the board STILL: measuring gyro bias..."));
  const int N = 500;
  float aSum[3] = {0, 0, 0}, gSum[3] = {0, 0, 0}, a[3], g[3], t;
  int n = 0;
  while (n < N) {
    if (readRaw(a, g, t)) {
      float ac[3]; applyAccCal(a, ac);
      for (int i = 0; i < 3; i++) { aSum[i] += ac[i]; gSum[i] += g[i]; }
      n++;
    }
    delay(3);
  }
  for (int i = 0; i < 3; i++) gBias[i] = gSum[i] / N;
  if (!buildFrame) { Serial.println(F("# Gyro bias updated")); return; }

#if AUTO_LEVEL
  // DOWN = opposite of measured gravity reaction; FORWARD = sensor +Y levelled
  float dn[3] = {-aSum[0] / N, -aSum[1] / N, -aSum[2] / N};
  float nn = norm3(dn);
  for (int i = 0; i < 3; i++) dn[i] /= nn;
  float f[3] = {0, 1, 0};
  float d = f[0]*dn[0] + f[1]*dn[1] + f[2]*dn[2];
  for (int i = 0; i < 3; i++) f[i] -= d * dn[i];
  if (norm3(f) < 0.3f) {                     // VCC end vertical -> use X arrow
    f[0] = 1; f[1] = 0; f[2] = 0;
    d = dn[0];
    for (int i = 0; i < 3; i++) f[i] -= d * dn[i];
  }
  float fn = norm3(f);
  for (int i = 0; i < 3; i++) f[i] /= fn;
  float r[3] = {dn[1]*f[2] - dn[2]*f[1], dn[2]*f[0] - dn[0]*f[2], dn[0]*f[1] - dn[1]*f[0]};
  for (int j = 0; j < 3; j++) { M[0][j] = f[j]; M[1][j] = r[j]; M[2][j] = dn[j]; }
  attCal = {0, 0, 0};
  attRaw = {0, 0, 0};
#else
  // FIXED mapping (MPU flat, component side up, VCC end forward):
  //   X_body(fwd) = +Y_sensor,  Y_body(right) = +X_sensor,  Z_body(down) = -Z_sensor
  float Mf[3][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) M[i][j] = Mf[i][j];
  // Start both filters at the accelerometer angles (absolute, no zeroing)
  float aRaw[3], aCal[3], fRaw[3], fCal[3];
  for (int i = 0; i < 3; i++) aCal[i] = aSum[i] / N;
  for (int i = 0; i < 3; i++) aRaw[i] = aCal[i] * ACC_SCALE[i] + ACC_BIAS[i];
  toBody(aCal, fCal); toBody(aRaw, fRaw);
  accAngles(fCal, attCal.roll, attCal.pitch);
  accAngles(fRaw, attRaw.roll, attRaw.pitch);
  attCal.yaw = attRaw.yaw = 0;
#endif
}

void printInfo() {
  Serial.print(F("# Mode: ")); Serial.println(AUTO_LEVEL ? F("AUTO_LEVEL") : F("FIXED mounting"));
  Serial.print(F("# Main output uses calibration: ")); Serial.println(useCal ? F("YES") : F("NO"));
  Serial.print(F("# Verify packet: ")); Serial.println(verifyMode ? F("ON") : F("OFF"));
  Serial.print(F("# Acc bias  (g): "));
  for (int i = 0; i < 3; i++) { Serial.print(ACC_BIAS[i], 6); Serial.print(' '); }
  Serial.print(F("\n# Acc scale    : "));
  for (int i = 0; i < 3; i++) { Serial.print(ACC_SCALE[i], 6); Serial.print(' '); }
  Serial.print(F("\n# Gyro bias (deg/s): "));
  for (int i = 0; i < 3; i++) { Serial.print(gBias[i], 3); Serial.print(' '); }
  Serial.print(F("\n# Invalid I2C reads so far: ")); Serial.println(badReads);
}

void printHeader() {
  if (verifyMode)
    Serial.println(F("start,yaw_deg,pitch_deg,roll_deg,temp_C,pitch_raw_deg,roll_raw_deg,g_cal,g_raw,end   (NED)"));
  else
    Serial.println(F("start,yaw_deg,pitch_deg,roll_deg,temp_C,end   (NED)"));
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  if      (s == "v1") { verifyMode = true;  Serial.println(F("# verify packet ON"));  printHeader(); }
  else if (s == "v0") { verifyMode = false; Serial.println(F("# verify packet OFF")); printHeader(); }
  else if (s == "c1") { useCal = true;  Serial.println(F("# calibration ON")); }
  else if (s == "c0") { useCal = false; Serial.println(F("# calibration OFF")); }
  else if (s == "z")  { attCal.yaw = attRaw.yaw = 0; Serial.println(F("# yaw zeroed")); }
  else if (s == "b")  { measureBiasAndFrame(false); lastMicros = micros(); }
  else if (s == "i")  printInfo();
  else { Serial.print(F("# unknown command: ")); Serial.println(s); }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(400000);

  writeReg(0x6B, 0x00);   // wake up
  delay(100);
  writeReg(0x1A, 0x03);   // gyro DLPF ~41 Hz
  writeReg(0x1B, 0x00);   // gyro +/-250 deg/s (same as calibration data)
  writeReg(0x1C, 0x00);   // accel +/-2 g      (same as calibration data)
  writeReg(0x1D, 0x03);   // accel DLPF ~44 Hz
  delay(500);

  measureBiasAndFrame(true);
  printInfo();
  lastMicros = micros();
}

void loop() {
  // ---- commands ----
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(cmd); cmd = ""; }
    else if (cmd.length() < 20) cmd += ch;
  }

  // ---- read + filter ----
  float a[3], g[3], t;
  if (!readRaw(a, g, t)) { badReads++; return; }   // skip invalid sample

  float aCal[3]; applyAccCal(a, aCal);
  float gc[3];   for (int i = 0; i < 3; i++) gc[i] = g[i] - gBias[i];
  float fCal[3], fRaw[3], w[3];
  toBody(aCal, fCal);
  toBody(a, fRaw);
  toBody(gc, w);

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt > 0.1f) dt = 0.1f;                        // guard after a pause

  filterStep(attCal, fCal, w, dt);
  filterStep(attRaw, fRaw, w, dt);

  // gravity magnitude, low-pass filtered for display (tau ~0.2 s)
  static float gCalF = 1.0f, gRawF = 1.0f;
  gCalF += 0.1f * (norm3(aCal) - gCalF);
  gRawF += 0.1f * (norm3(a)    - gRawF);

  // ---- output ----
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= OUT_PERIOD_MS) {
    lastPrint = millis();

    static bool wasConnected = false;      // header whenever a program connects
    bool connected = Serial;
    if (connected && !wasConnected) printHeader();
    wasConnected = connected;

    const Att &m = useCal ? attCal : attRaw;
    Serial.print(F("111,"));
    Serial.print(m.yaw, 2);   Serial.print(',');
    Serial.print(m.pitch, 2); Serial.print(',');
    Serial.print(m.roll, 2);  Serial.print(',');
    Serial.print(t, 2);
    if (verifyMode) {
      Serial.print(',');
      Serial.print(attRaw.pitch, 2); Serial.print(',');
      Serial.print(attRaw.roll, 2);  Serial.print(',');
      Serial.print(gCalF, 4);        Serial.print(',');
      Serial.print(gRawF, 4);
    }
    Serial.println(F(",FFF"));
  }
}
