

https://github.com/user-attachments/assets/e7aca600-2801-4c82-811f-36891c2ff6a2

# STM32 IMU Attitude EKF

Real-time 3D attitude estimation with an **STM32 Nucleo F446RE** and an **MPU6050 (GY-521)**,
using a 7-state quaternion **Extended Kalman Filter**, with a live browser dashboard that shows
the result as a 3D plane, an artificial horizon and a heading tape.

The firmware runs three estimators side by side on the same data, so you can see what the EKF
actually buys you compared with simpler approaches:

| Estimator | What it is | Behaviour |
|---|---|---|
| Gyro only | Quaternion integration of the gyro | Smooth, but drifts without limit |
| Complementary filter | Gyro + accelerometer with a fixed gain | No roll/pitch drift, fixed noise/lag trade-off |
| **EKF** | Quaternion + 3 gyro biases, accelerometer update | Learns the gyro bias, adapts its gain |



## Features

- 100 Hz sensor fusion in Arduino C++ (about 160 us per EKF step on a Nucleo F446RE)
- Guided 6-position accelerometer calibration sketch
- Dashboard (no install, runs in Chrome or Edge via Web Serial):
  - 3D plane with one solid plane (EKF) and two ghost planes (complementary, gyro-only)
  - cockpit-style artificial horizon and heading tape
  - live roll / pitch / yaw plots of all three estimators, with a freeze button and a cursor read-out
  - mouse-controlled camera, sky and horizon, tare (level + zero heading) button

## Hardware

| Part | Notes |
|---|---|
| ST Nucleo-F446RE | Uses the on-board ST-Link USB virtual COM port, no extra cable needed |
| GY-521 (MPU6050) breakout | Has an on-board 3.3 V regulator and I2C pull-ups |

| GY-521 | Nucleo F446RE |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | PB9 (D14) |
| SCL | PB8 (D15) |
| AD0 | GND or unconnected (I2C address 0x68) |

## Repository layout

```
firmware/
  mpu6050_accel_calibration/   6-position accelerometer calibration
  imu_attitude_ekf/            fusion firmware: gyro-only, complementary filter and EKF, streams over serial
dashboard/
  index.html                   browser dashboard (single file)
```

## Quick start

1. **Install the board package.** In the Arduino IDE, add
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
   under *Preferences > Additional boards manager URLs*, then install *STM32 MCU based boards* in the
   Boards Manager. Select *Nucleo-64*, board part number *Nucleo F446RE*, upload method
   *STM32CubeProgrammer (SWD)* (install STM32CubeProgrammer from st.com if the upload complains),
   and set **Tools > U(S)ART support > Enabled (generic 'Serial')**.
2. **Calibrate the accelerometer.** Upload `mpu6050_accel_calibration`, open the serial monitor at
   **460800 baud** and follow the prompts (6 poses, press Enter in each). Copy the printed
   `ACC_OFFSET` and `ACC_SCALE` lines. Hobby MPU6050 boards are typically several percent off in scale
   and offset, which this removes.
3. **Flash the firmware.** Paste those two lines near the top of `imu_attitude_ekf.ino`, then upload it.
   After reset, keep the board still for about 2 seconds: the gyro bias and starting attitude are
   measured then.
4. **Open the dashboard.** Close the Arduino serial monitor (only one program can hold the port), open
   `dashboard/index.html` in **Chrome or Edge**, click *Connect board* and choose the Nucleo's port.
   The page loads three.js from a CDN, so it needs an internet connection.

Dashboard controls: drag to orbit, mouse wheel to zoom, double-click to reset the view, **Space** to
freeze the plots (then hover a plot to read the values at that moment), *Tare* to treat the current pose
as level and the current heading as zero, *Reset filters* to re-zero all three estimators.

## Conventions

Body frame (aerospace FRD): X forward (nose), Y right wing, Z down. Hold the GY-521 **chip side up with
its X arrow pointing at the nose**. The code flips the sensor's Y and Z signs to match.

- roll > 0: right wing down
- pitch > 0: nose up
- yaw > 0: nose turns right (clockwise seen from above)

## Serial protocol

460800 baud, one text line per sample at 100 Hz:

```
Q,t_ms, gw,gx,gy,gz, cw,cx,cy,cz, ew,ex,ey,ez, ekfMicros
```

`g*` is the gyro-only quaternion, `c*` the complementary filter, `e*` the EKF (body to world, unit
quaternions). Lines starting with `#` are comments. Sending any character to the board re-zeroes all
three estimators.

## How it works

**EKF state:** `x = [q0 q1 q2 q3, bx by bz]`, the attitude quaternion and the gyro bias in rad/s.

- **Predict:** integrate the bias-corrected gyro into the quaternion and propagate the covariance with
  `P = F P F^T + Q` (analytic Jacobian `F`).
- **Update:** the accelerometer direction is compared with the gravity direction predicted from the
  quaternion. The innovation is corrected through the gain `K = P H^T (H P H^T + R)^-1`. The
  update is skipped when the accelerometer magnitude is not within 0.15 g of 1 g.

Three safeguards are set in the `EKF` struct. They were added after testing on real hardware, where a
plain EKF learned a bogus z-gyro bias (about -0.16 deg/s, far larger than the real drift) while the board
was moved by hand, and its yaw ended up further from the gyro than it should have:

| Setting | Default | Why |
|---|---|---|
| `lockBiasZ` | `true` | Gravity cannot observe the z-gyro bias, so it is not estimated |
| `tiltOnlyUpdate` | `true` | The accelerometer may correct roll and pitch but never yaw. This is a pragmatic projection, not the textbook EKF update |
| `rAcc` | `9e-5` | Accelerometer noise variance (g^2), inflated 10x from the still-board value because hand motion adds acceleration |

Other tuning values: `sigmaGyro` (gyro noise, rad/s), `qBias` (how fast the x/y biases may wander),
`COMP_KP` (complementary filter gain, time constant is `1/Kp`), `ACC_GATE`.

## Results and limitations

Measured on one Nucleo F446RE with one GY-521 (your board will differ):

- accelerometer magnitude before calibration was about 1.11 g, after the 6-pose calibration about 1.00 g
- with only the startup gyro calibration, gyro-only drift over one minute was roughly 0.3 deg (roll),
  1 deg (pitch) and 0.2 deg (yaw)
- the EKF takes about 159 us per sample, about 1.6% of the 10 ms budget
- after flipping the board through many orientations, the gyro-only, complementary and EKF yaw readings
  agreed to within 0.6 deg

Limitations to be aware of:

- **No magnetometer, so yaw drifts.** Gravity cannot observe rotation about the vertical axis. The EKF's
  yaw is effectively integrated gyro, and the dashboard's yaw is relative, not a true heading.
- Roll and pitch can still be disturbed by strong hand acceleration.
- Gyro scale error (the MPU6050's sensitivity tolerance is a few percent) shows up as yaw error after large
  rotations. It cannot be seen without an external heading reference.
- Validation consists of simulations and simple bench tests on one board, with no external ground-truth
  reference.
- The dashboard needs Chrome or Edge (Web Serial) and assumes a 100 Hz stream for its time axis.

## Troubleshooting

| Problem | Check |
|---|---|
| Nothing appears in the serial monitor | Tools > U(S)ART support must be "Enabled (generic 'Serial')"; baud 460800 |
| `WHO_AM_I` shows 0 or the sketch stops with an error | Wiring and the I2C address (AD0 low = 0x68). Some clone chips report a different ID, which is usually harmless |
| Dashboard cannot connect | Close the Arduino serial monitor; use Chrome or Edge |
| Plane rolls or pitches the wrong way | Board orientation: chip side up, X arrow at the nose |
| "Flat" board does not read 0 degrees | A degree or two is normal (surface, mounting, sensor misalignment). Use *Tare* |

## Possible next steps

- magnetometer for absolute yaw (with gating against magnetic disturbances) and a free z-bias
- temperature-compensated gyro bias (the MPU6050 already reports its temperature)
- error-state (multiplicative) EKF or UKF variants

