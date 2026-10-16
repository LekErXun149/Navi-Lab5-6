// =====================================================================
//  TrafficSignal_IMU.ino
//  RSE4501 Mini Project 2 - Part 2 application
//  Traffic Police Hand Signals -> OBU screen translator
//
//  Hardware : Arduino Leonardo + GY-9250 module (MPU-6500), one IMU,
//             breadboard held in the officer's RIGHT hand:
//             MPU flat, chip side UP, VCC end of the header pointing
//             BACK toward the officer (sensor -Y = along the arm).
//  Wiring   : VCC->5V, GND->GND, SDA->D2, SCL->D3
//
//  Body frame (NED, attached to the arm)
//     X forward = along the arm toward the hand  = sensor -Y
//     Y right                                    = sensor -X
//     Z down    (chip side faces up)             = sensor -Z
//
//  Orientation: quaternion complementary filter (Mahony style).
//     - gyro integration (no gimbal lock, works with the arm vertical)
//     - accelerometer corrects tilt (pitch/roll) when |a| is close to 1 g
//     - gyro bias measured at start-up and slowly updated whenever the
//       arm is held still (zero-velocity update) to reduce heading drift
//  Arm angles:
//     arm pitch   = elevation of the arm (+ up, -90..+90)
//     arm heading = direction the arm points, relative to the reference
//                   set with 'z' (arm straight forward = 0, + = clockwise,
//                   i.e. toward the officer's right)
//
//  Signals (right arm only)
//     1 UP     : pitch > +60                    -> right traffic STOP
//     2 DOWN   : pitch < -60                    -> right traffic GO
//     3 SIDE   : |pitch| < 25, heading +45..+135 -> right traffic PREPARE TO STOP
//     4 ACROSS : |pitch| < 25, heading -135..-45 -> front traffic may TURN RIGHT
//     A pose must be held 0.5 s before it is shown. The shown signal is
//     latched (kept) until another valid pose is held; 0 = "---" at start-up.
//
//  Packet to MATLAB (getData.m, 20 Hz):
//     111,heading,pitch,roll,temp,signal,candidate,FFF
//
//  Serial commands (MATLAB buttons or Serial Monitor, "Newline"):
//     z  set heading reference (hold the arm STRAIGHT FORWARD, level)
//     r  reset displayed signal to "---"
//     b  re-measure gyro bias (keep the board STILL ~2 s)
//     i  print settings
// =====================================================================
#include <Wire.h>
#include <math.h>

// ---------------- CALIBRATION (six-position test, sensor axes) ----------------
const float ACC_BIAS[3]  = { 0.001665f, -0.008621f,  0.010054f };  // g
const float ACC_SCALE[3] = { 0.998615f,  0.999471f,  1.006751f };  // -

// ---------------- GESTURE SETTINGS ----------------
const float UP_DEG      = 60.0f;   // arm up   if pitch >  60
const float DOWN_DEG    = -60.0f;  // arm down if pitch < -60
const float LEVEL_DEG   = 25.0f;   // arm level if |pitch| < 25
const float HEAD_MIN    = 45.0f;   // side / across heading window 45..135 deg
const float HEAD_MAX    = 135.0f;
const float HYST_DEG    = 5.0f;    // hysteresis once a pose is detected
const uint16_t HOLD_MS  = 500;     // pose must be held this long
const float MOVE_DPS    = 60.0f;   // pose only accepted if rotating slower than this

// ---------------- SENSOR / FILTER ----------------
const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB = 16384.0f;    // +/-2 g
const float GYR_LSB = 65.5f;       // +/-500 deg/s (fast arm motion, no clipping)
const float KP      = 2.0f;        // accelerometer correction gain (1/s)
const float RAD2DEG = 57.2957795f;
const float DEG2RAD = 0.01745329f;
const uint16_t OUT_PERIOD_MS = 50; // 20 Hz to MATLAB

// sensor -> body (NED) rotation for the RIGHT-hand grip
const float M[3][3] = {{ 0, -1,  0},   // forward = -Y sensor
                       {-1,  0,  0},   // right   = -X sensor
                       { 0,  0, -1}};  // down    = -Z sensor

// ---------------- STATE ----------------
float q[4] = {1, 0, 0, 0};          // body -> world quaternion (w,x,y,z)
float gBias[3] = {0, 0, 0};         // sensor frame, deg/s
float headRef = 0;                  // heading reference (deg)
float armPitch = 0, armHead = 0, armRoll = 0;
int shown = 0;                      // latched signal (0 = ---)
int cand = 0;                       // current candidate pose
uint32_t candSince = 0;
uint32_t stillSince = 0;
uint32_t lastMicros;
uint32_t badReads = 0;
uint16_t badInRow = 0;              // consecutive failed reads
bool sensorOK = true;
String cmd = "";

const char* NAMES[] = {"---", "UP", "DOWN", "SIDE", "ACROSS"};

// =====================================================================
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

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
  if (allZero) return false;                         // I2C dropout
  for (int i = 0; i < 3; i++)
    a[i] = (v[i] / ACC_LSB - ACC_BIAS[i]) / ACC_SCALE[i];   // calibrated
  t = v[3] / 333.87f + 21.0f;
  g[0] = v[4] / GYR_LSB;  g[1] = v[5] / GYR_LSB;  g[2] = v[6] / GYR_LSB;
  return true;
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

// Initialise quaternion from gravity (heading = 0)
void initFromAccel(const float f[3]) {
  float roll  = atan2(-f[1], -f[2]);
  float pitch = atan2(f[0], sqrt(f[1]*f[1] + f[2]*f[2]));
  float cr = cos(roll / 2), sr = sin(roll / 2), cp = cos(pitch / 2), sp = sin(pitch / 2);
  q[0] = cr * cp;  q[1] = sr * cp;  q[2] = cr * sp;  q[3] = -sr * sp;
}

// One filter step. f = specific force (body, g), w = body rate (rad/s)
void filterStep(const float f[3], float w[3], float dt) {
  float fn = norm3(f);
  if (fn > 0.85f && fn < 1.15f) {                    // trust accel only near 1 g
    float ax = f[0] / fn, ay = f[1] / fn, az = f[2] / fn;
    // estimated specific-force direction at rest: R^T * (0,0,-1)
    float vx = -2 * (q[1]*q[3] - q[0]*q[2]);
    float vy = -2 * (q[2]*q[3] + q[0]*q[1]);
    float vz = -(1 - 2 * (q[1]*q[1] + q[2]*q[2]));
    w[0] += KP * (ay * vz - az * vy);
    w[1] += KP * (az * vx - ax * vz);
    w[2] += KP * (ax * vy - ay * vx);
  }
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  q[0] += 0.5f * dt * (-qx*w[0] - qy*w[1] - qz*w[2]);
  q[1] += 0.5f * dt * ( qw*w[0] + qy*w[2] - qz*w[1]);
  q[2] += 0.5f * dt * ( qw*w[1] - qx*w[2] + qz*w[0]);
  q[3] += 0.5f * dt * ( qw*w[2] + qx*w[1] - qy*w[0]);
  float n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// Arm direction = body X axis in the world frame
void armAngles() {
  float fN = 1 - 2 * (q[2]*q[2] + q[3]*q[3]);
  float fE = 2 * (q[1]*q[2] + q[0]*q[3]);
  float fD = 2 * (q[1]*q[3] - q[0]*q[2]);
  armPitch = atan2(-fD, sqrt(fN*fN + fE*fE)) * RAD2DEG;
  armHead  = wrap180(atan2(fE, fN) * RAD2DEG - headRef);
  armRoll  = atan2(2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2 * (q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
}

int classify(float pitch, float head, int prev) {
  float hu = (prev == 1) ? HYST_DEG : 0;
  float hd = (prev == 2) ? HYST_DEG : 0;
  float hl = (prev == 3 || prev == 4) ? HYST_DEG : 0;
  if (pitch > UP_DEG - hu)   return 1;
  if (pitch < DOWN_DEG + hd) return 2;
  if (fabs(pitch) < LEVEL_DEG + hl) {
    float hs = (prev == 3) ? HYST_DEG : 0;
    if (head > HEAD_MIN - hs && head < HEAD_MAX + hs) return 3;
    float ha = (prev == 4) ? HYST_DEG : 0;
    if (head < -HEAD_MIN + ha && head > -HEAD_MAX - ha) return 4;
  }
  return 0;
}

// ---------------- start-up / commands ----------------
void measureBias(bool initAttitude) {
  Serial.println(F("# Keep the board STILL: measuring gyro bias..."));
  const int N = 500;
  float aS[3] = {0, 0, 0}, gS[3] = {0, 0, 0}, a[3], g[3], t;
  int n = 0;
  while (n < N) {
    if (readRaw(a, g, t)) {
      for (int i = 0; i < 3; i++) { aS[i] += a[i]; gS[i] += g[i]; }
      n++;
    }
    delay(2);
  }
  for (int i = 0; i < 3; i++) gBias[i] = gS[i] / N;
  if (initAttitude) {
    float am[3] = {aS[0] / N, aS[1] / N, aS[2] / N}, f[3];
    toBody(am, f);
    initFromAccel(f);
    headRef = 0;
  }
  Serial.println(F("# Gyro bias done"));
}

void printInfo() {
  Serial.println(F("# TrafficSignal_IMU - right-hand grip, VCC end toward officer"));
  Serial.print(F("# Gyro bias (deg/s): "));
  for (int i = 0; i < 3; i++) { Serial.print(gBias[i], 3); Serial.print(' '); }
  Serial.print(F("\n# Shown signal: ")); Serial.println(NAMES[shown]);
  Serial.print(F("# Invalid I2C reads: ")); Serial.println(badReads);
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  if (s == "z") {
    if (fabs(armPitch) < 30) {
      headRef = wrap180(armHead + headRef);
      Serial.println(F("# Heading reference set (arm forward = 0)"));
    } else {
      Serial.println(F("# Hold the arm LEVEL and straight forward, then zero again"));
    }
  }
  else if (s == "r") { shown = 0; Serial.println(F("# Display reset to ---")); }
  else if (s == "b") { measureBias(false); lastMicros = micros(); }
  else if (s == "i") printInfo();
  else { Serial.print(F("# unknown command: ")); Serial.println(s); }
}

// =====================================================================
// (Re)start I2C and configure the MPU. Called at start-up and after a
// connection fault (e.g. a jumper briefly pulled during an arm movement).
void initMPU() {
  Wire.end();
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);   // never hang if the I2C bus locks up
#endif
  writeReg(0x6B, 0x00);   // wake up
  delay(50);
  writeReg(0x1A, 0x03);   // gyro DLPF ~41 Hz
  writeReg(0x1B, 0x08);   // gyro +/-500 deg/s
  writeReg(0x1C, 0x00);   // accel +/-2 g
  writeReg(0x1D, 0x03);   // accel DLPF ~44 Hz
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  initMPU();
  delay(500);
  measureBias(true);
  printInfo();
  lastMicros = micros();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(cmd); cmd = ""; }
    else if (cmd.length() < 20) cmd += ch;
  }

  float a[3], g[3], t;
  if (!readRaw(a, g, t)) {
    badReads++;
    if (++badInRow >= 20) {                 // ~20 failures in a row -> recover
      if (sensorOK) Serial.println(F("# SENSOR ERROR: I2C lost, re-initialising (check jumpers)"));
      sensorOK = false;
      initMPU();
      badInRow = 0;
      lastMicros = micros();
    }
    delay(1);
    return;
  }
  if (!sensorOK) { Serial.println(F("# Sensor recovered")); sensorOK = true; }
  badInRow = 0;

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt > 0.05f) dt = 0.05f;

  float gc[3]; for (int i = 0; i < 3; i++) gc[i] = g[i] - gBias[i];
  float f[3], wd[3], w[3];
  toBody(a, f);
  toBody(gc, wd);
  for (int i = 0; i < 3; i++) w[i] = wd[i] * DEG2RAD;
  float rate = norm3(wd);                              // deg/s

  // ---- zero-velocity update: arm still -> gyro should read zero ----
  float fn = norm3(f);
  if (rate < 2.5f && fabs(fn - 1.0f) < 0.03f) {
    if (stillSince == 0) stillSince = millis();
    if (millis() - stillSince > 1000)
      for (int i = 0; i < 3; i++) gBias[i] += 0.002f * (g[i] - gBias[i]);
  } else {
    stillSince = 0;
  }

  filterStep(f, w, dt);
  armAngles();

  // ---- gesture detection with hold time + latch ----
  int c = classify(armPitch, armHead, cand);
  if (c != cand) { cand = c; candSince = millis(); }
  else if (cand != 0 && cand != shown && rate < MOVE_DPS &&
           millis() - candSince >= HOLD_MS) {
    shown = cand;
  }

  // ---- output ----
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= OUT_PERIOD_MS) {
    lastPrint = millis();
    static bool wasConnected = false;
    bool connected = Serial;
    if (connected && !wasConnected)
      Serial.println(F("start,heading_deg,pitch_deg,roll_deg,temp_C,signal,candidate,end"));
    wasConnected = connected;

    Serial.print(F("111,"));
    Serial.print(armHead, 1);  Serial.print(',');
    Serial.print(armPitch, 1); Serial.print(',');
    Serial.print(armRoll, 1);  Serial.print(',');
    Serial.print(t, 2);        Serial.print(',');
    Serial.print(shown);       Serial.print(',');
    Serial.print(cand);
    Serial.println(F(",FFF"));
  }
}
