# Navi-Lab5-6: IMU Integration and Traffic Police Hand-Signal Translator

RSE4501 Navigation, Mini Project 2 (Group 5).

An MPU-6500 IMU (GY-9250 module) on an Arduino Leonardo measures orientation (Part 1).
The same IMU, held in a traffic officer's right hand, recognises four hand signals and
shows them on a simulated ERP 2.0 On-Board Unit (OBU) screen in MATLAB (Part 2).

## Hardware

| Item | Notes |
|---|---|
| GY-9250 module (MPU-6500 chip, WHO_AM_I = 0x70) | Accelerometer + gyroscope, no magnetometer |
| Arduino Leonardo | VCC → 5V, GND → GND, SDA → D2, SCL → D3 |

## Folder and file overview

### Arduino sketches (open the `.ino` inside each folder with the Arduino IDE)

| Folder | Purpose |
|---|---|
| `Basic/` | Starting example from the MPU9250 library / handout (initial test) |
| `MPU-9250/` | MPU9250 library test. Fails on this board because the MPU-6500 has no magnetometer |
| `MPU9250_Leonardo_Diagnostic/` | Hardware check without libraries: I2C scan, WHO_AM_I, magnetometer check, live raw data |
| `MPU6500_Leonardo_MATLAB/` | First orientation sketch: complementary filter, sends `111,yaw,pitch,roll,temp,FFF` to MATLAB |
| `IMU_Raw_Logger/` | Raw data logger for calibration. Serial commands (`s <label>`, `n`, `p`) record batches of raw ax, ay, az, gx, gy, gz |
| `IMU_NED_Calibrated/` | Calibrated orientation in the NED frame. Runs calibrated and uncalibrated filters side by side for the verification test |
| `TrafficSignal_IMU/` | **Application.** Quaternion filter, right-hand mounting, detects 4 arm poses (0.5 s hold) and sends the signal to MATLAB |

### MATLAB scripts (keep all `.m` files in one folder)

| File | Purpose |
|---|---|
| `getData.m` | Helper: reads one `111,...,FFF` packet from the serial port. Used by all scripts below |
| `imu_live.m` | Live 3D view of the board plus angle plots; saves a CSV log |
| `imu_part1_experiments.m` | Guided Part 1 experiments (static, tilt, yaw), analysis and PASS/FAIL table |
| `imu_cal_verification.m` | Calibration verification: 6 poses × A/B, gravity-magnitude and tilt error, static run, figures |
| `obu_display.m` | **Application.** Simulated OBU screen showing the detected traffic signal |

### Other

| Folder | Purpose |
|---|---|
| `IMU Schematic/` | KiCad schematic of the Leonardo ↔ GY-9250 wiring |

## Traffic signals recognised (right arm, one IMU)

| Right arm | OBU message |
|---|---|
| Raised up | Vehicles from the officer's RIGHT: STOP |
| Down by side | Vehicles from the officer's RIGHT: GO |
| Horizontal to the right | Vehicles from the officer's RIGHT: PREPARE TO STOP |
| Horizontal across the body | Vehicles in FRONT of the officer: may TURN RIGHT |

## How to run the application

1. Upload `TrafficSignal_IMU/TrafficSignal_IMU.ino` to the Leonardo.
2. Lay the board flat and still, press RESET, and wait 2 s (gyro bias measurement).
3. Close the Arduino Serial Monitor. In `obu_display.m`, set `port` (e.g. `"COM11"`), then run it.
4. Hold the board in the right hand, point the arm straight forward and level, and click **Zero heading**.
5. Perform the signals, holding each for about 1 s.

## Calibration constants (six-position test)

| Axis | Accel bias (g) | Accel scale | Gyro bias (°/s) |
|---|---|---|---|
| X | +0.0017 | 0.9986 | +1.29 |
| Y | −0.0086 | 0.9995 | +1.20 |
| Z | +0.0101 | 1.0068 | −0.67 |
