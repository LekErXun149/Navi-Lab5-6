// IMU RAW DATA LOGGER - no calibration, no filtering, no axis remapping
// Board : Arduino Leonardo     Sensor: GY-9250 module (MPU-6500 chip)
// Wiring: VCC->5V, GND->GND, SDA->D2, SCL->D3
//
// Purpose: collect raw sensor-frame data in batches (default 50 samples)
//          for the six-position / yaw calibration tests.
//
// Serial Monitor: 115200 baud, line ending = "Newline"
// Commands:
//   s [label]   start a batch, e.g.  s flat   s upside_down   s roll_p90
//   x           stop the current batch early
//   n <count>   samples per batch          (default 50)
//   p <ms>      sample period in ms        (default 20 ms = 50 Hz)
//   i           sensor info / settings
//   h           help
//
// Output example:
//   BEGIN,batch=1,label=flat,samples=50,period_ms=20
//   N=1,t=0,ax=0.01234,ay=-0.00456,az=1.00210,gx=0.531,gy=-1.204,gz=0.312,temp=28.12
//   ...
//   END,batch=1,label=flat,count=50
//
// Units: accel in g, gyro in deg/s, temp in degC, t in ms from batch start.
// Scale factors are the datasheet nominal values only (16384 LSB/g, 131 LSB/(deg/s)).
// Axes are the raw SENSOR axes printed on the board (X away from pins, Y toward VCC, Z up).

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB = 16384.0;   // LSB per g     at +/-2 g     (datasheet)
const float GYR_LSB = 131.0;     // LSB per deg/s at +/-250 dps (datasheet)

uint16_t samplesPerBatch = 50;
uint16_t periodMs        = 20;

uint32_t batchNo    = 0;
bool     running    = false;
uint16_t sampleNo   = 0;
uint32_t batchStart = 0;
uint32_t nextSample = 0;
String   label      = "none";

String   cmd        = "";
uint32_t lastCharMs = 0;

// ---------------- I2C helpers ----------------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void readSensor(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz, float &temp) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                     // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  ax = v[0] / ACC_LSB;  ay = v[1] / ACC_LSB;  az = v[2] / ACC_LSB;
  temp = v[3] / 333.87 + 21.0;
  gx = v[4] / GYR_LSB;  gy = v[5] / GYR_LSB;  gz = v[6] / GYR_LSB;
}

// ---------------- Messages ----------------
void printHelp() {
  Serial.println(F("=== IMU raw data logger (no calibration) ==="));
  Serial.println(F("Serial Monitor: 115200 baud, line ending = Newline"));
  Serial.println(F("Commands:"));
  Serial.println(F("  s [label]  start a batch   e.g.  s flat   s upside_down   s roll_p90"));
  Serial.println(F("  x          stop the current batch"));
  Serial.print  (F("  n <count>  samples per batch (now "));
  Serial.print(samplesPerBatch); Serial.println(F(")"));
  Serial.print  (F("  p <ms>     sample period in ms (now "));
  Serial.print(periodMs); Serial.print(F(" ms = "));
  Serial.print(1000.0 / periodMs, 1); Serial.println(F(" Hz)"));
  Serial.println(F("  i          sensor info"));
  Serial.println(F("  h          this help"));
  Serial.println(F("Units: accel g, gyro deg/s, temp degC, t ms (raw sensor axes)"));
  Serial.println(F("Tip: for a 360 deg turn use a longer batch, e.g.  n 500  then  s yaw_360"));
}

void printInfo() {
  uint8_t who = readReg(0x75);
  Serial.print(F("WHO_AM_I = 0x")); Serial.print(who, HEX);
  if      (who == 0x71) Serial.println(F(" (MPU-9250)"));
  else if (who == 0x73) Serial.println(F(" (MPU-9255)"));
  else if (who == 0x70) Serial.println(F(" (MPU-6500, no magnetometer)"));
  else                  Serial.println(F(" (unexpected - check wiring)"));
  Serial.println(F("Ranges: accel +/-2 g, gyro +/-250 deg/s, DLPF ~41-44 Hz"));
  Serial.print(F("Batch: ")); Serial.print(samplesPerBatch);
  Serial.print(F(" samples, period ")); Serial.print(periodMs);
  Serial.print(F(" ms, batches so far: ")); Serial.println(batchNo);
}

// ---------------- Commands ----------------
void startBatch(String lab) {
  lab.trim();
  lab.replace(',', '_');                // keep the CSV-style output parseable
  lab.replace(' ', '_');
  label = (lab.length() > 0) ? lab : "none";
  batchNo++;
  sampleNo = 0;
  Serial.print(F("BEGIN,batch=")); Serial.print(batchNo);
  Serial.print(F(",label="));      Serial.print(label);
  Serial.print(F(",samples="));    Serial.print(samplesPerBatch);
  Serial.print(F(",period_ms="));  Serial.println(periodMs);
  running    = true;
  batchStart = millis();
  nextSample = batchStart;
}

void endBatch(bool aborted) {
  running = false;
  Serial.print(F("END,batch=")); Serial.print(batchNo);
  Serial.print(F(",label="));    Serial.print(label);
  Serial.print(F(",count="));    Serial.print(sampleNo);
  if (aborted) Serial.print(F(",aborted"));
  Serial.println();
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = tolower(s.charAt(0));
  String arg = s.substring(1);
  arg.trim();

  switch (c) {
    case 's':
      if (running) { Serial.println(F("Busy: batch running (x to stop)")); break; }
      startBatch(arg);
      break;
    case 'x':
      if (running) endBatch(true);
      else Serial.println(F("No batch running"));
      break;
    case 'n': {
      long v = arg.toInt();
      if (running)                 Serial.println(F("Busy: stop the batch first"));
      else if (v < 1 || v > 10000) Serial.println(F("n must be 1..10000"));
      else { samplesPerBatch = v; Serial.print(F("Samples per batch = ")); Serial.println(v); }
      break;
    }
    case 'p': {
      long v = arg.toInt();
      if (running)                 Serial.println(F("Busy: stop the batch first"));
      else if (v < 10 || v > 1000) Serial.println(F("p must be 10..1000 ms"));
      else { periodMs = v; Serial.print(F("Sample period = ")); Serial.print(v); Serial.println(F(" ms")); }
      break;
    }
    case 'i': printInfo(); break;
    case 'h': case '?': printHelp(); break;
    default:
      Serial.print(F("Unknown command: ")); Serial.println(s);
      Serial.println(F("Type h for help"));
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  writeReg(0x6B, 0x00);   // wake up
  delay(100);
  writeReg(0x1A, 0x03);   // gyro digital low-pass ~41 Hz
  writeReg(0x1B, 0x00);   // gyro +/-250 dps
  writeReg(0x1C, 0x00);   // accel +/-2 g
  writeReg(0x1D, 0x03);   // accel digital low-pass ~44 Hz
}

void loop() {
  // Show help when the Serial Monitor is opened (Leonardo does not reset on open).
  // Checked only while idle: on the Leonardo, testing Serial takes ~10 ms.
  if (!running) {
    static bool wasConnected = false;
    bool connected = Serial;
    if (connected && !wasConnected) { delay(200); printHelp(); }
    wasConnected = connected;
  }

  // Read commands (works with "Newline", "CR", "Both", or "No line ending")
  while (Serial.available()) {
    char ch = Serial.read();
    lastCharMs = millis();
    if (ch == '\n' || ch == '\r') {
      handleCommand(cmd);
      cmd = "";
    } else if (cmd.length() < 40) {
      cmd += ch;
    }
  }
  if (cmd.length() > 0 && millis() - lastCharMs > 150) {   // no line ending sent
    handleCommand(cmd);
    cmd = "";
  }

  // Sample at a fixed rate while a batch is running
  if (running && (int32_t)(millis() - nextSample) >= 0) {
    nextSample += periodMs;
    float ax, ay, az, gx, gy, gz, temp;
    readSensor(ax, ay, az, gx, gy, gz, temp);
    uint32_t t = millis() - batchStart;
    sampleNo++;

    Serial.print(F("N="));     Serial.print(sampleNo);
    Serial.print(F(",t="));    Serial.print(t);
    Serial.print(F(",ax="));   Serial.print(ax, 5);
    Serial.print(F(",ay="));   Serial.print(ay, 5);
    Serial.print(F(",az="));   Serial.print(az, 5);
    Serial.print(F(",gx="));   Serial.print(gx, 3);
    Serial.print(F(",gy="));   Serial.print(gy, 3);
    Serial.print(F(",gz="));   Serial.print(gz, 3);
    Serial.print(F(",temp=")); Serial.println(temp, 2);

    if (sampleNo >= samplesPerBatch) endBatch(false);
  }
}
