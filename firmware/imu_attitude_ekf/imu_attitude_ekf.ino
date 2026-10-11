/*
  imu_attitude_ekf: attitude estimation firmware for the STM32 Nucleo F446RE + MPU6050 (GY-521)

  Runs three attitude estimators on the same sensor data at 100 Hz and streams them
  over serial for the web dashboard (dashboard/index.html):
    1. gyro-only integration      (drifts: the reference for "no correction")
    2. complementary filter       (gyro + accelerometer, fixed gain)
    3. 7-state quaternion EKF     (quaternion + gyro bias, with the safeguards below)

  SETUP
    - Paste your ACC_OFFSET / ACC_SCALE from mpu6050_accel_calibration.ino below.
    - Arduino IDE: Tools > U(S)ART support > "Enabled (generic 'Serial')", serial 460800 baud.
    - Keep the board still for ~2 s after reset: the gyro bias and the starting attitude
      are measured then.

  BODY FRAME (aerospace convention, "FRD"):
      X = forward (nose), Y = right wing, Z = down
  How to hold the GY-521 so it matches:
      chip side UP, board's X arrow pointing to the NOSE.
      (the board's Y arrow then points to the LEFT wing; the code flips
       the Y and Z signs to convert sensor axes -> body axes)

  ANGLES (degrees):
      roll  > 0 : right wing down
      pitch > 0 : nose up
      yaw   > 0 : nose turns right (clockwise seen from above)

  OUTPUT (text lines, 100 Hz, 460800 baud; lines starting with '#' are comments):
      Q,t_ms, gw,gx,gy,gz, cw,cx,cy,cz, ew,ex,ey,ez, ekfMicros
        g* = gyro-only quaternion, c* = complementary filter, e* = EKF   (body -> world)
        ekfMicros = time spent in the EKF per sample [us]

  Send any character over serial to re-zero all three estimators.
*/

#include <Wire.h>

// ====== PASTE YOUR VALUES FROM accel_calibration.ino HERE ======
constexpr float ACC_OFFSET[3] = {0.0f, 0.0f, 0.0f};
constexpr float ACC_SCALE[3]  = {1.0f, 1.0f, 1.0f};
// ================================================================

constexpr uint8_t  MPU_ADDR  = 0x68;
constexpr float    ACCEL_LSB = 16384.0f;   // +-2 g
constexpr float    GYRO_LSB  = 65.5f;      // +-500 deg/s
constexpr float    DEG2RAD   = 0.017453292f;
constexpr float    RAD2DEG   = 57.2957795f;
constexpr uint32_t LOOP_US   = 10000;      // 100 Hz

constexpr float COMP_KP  = 2.0f;    // complementary filter gain, time constant = 1/Kp (0.5 s)
constexpr float ACC_GATE = 0.15f;   // only trust accel if | |a| - 1 g | < 0.15 g

// MPU6050 registers
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

// ---------------------------------------------------------------------
// BEGIN MATH  (pure functions, no hardware access)
// ---------------------------------------------------------------------
struct Quat { float w, x, y, z; };   // rotation body -> world, unit length

void quatNormalize(Quat &q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  q.w /= n; q.x /= n; q.y /= n; q.z /= n;
}

// q_dot = 0.5 * q (x) (0, w)   ->   q += q_dot * dt   ->   renormalise
void quatIntegrate(Quat &q, const float w[3], float dt) {
  float dw = 0.5f * (-q.x * w[0] - q.y * w[1] - q.z * w[2]);
  float dx = 0.5f * ( q.w * w[0] + q.y * w[2] - q.z * w[1]);
  float dy = 0.5f * ( q.w * w[1] - q.x * w[2] + q.z * w[0]);
  float dz = 0.5f * ( q.w * w[2] + q.x * w[1] - q.y * w[0]);
  q.w += dw * dt;
  q.x += dx * dt;
  q.y += dy * dt;
  q.z += dz * dt;
  quatNormalize(q);
}

void quatToEuler(const Quat &q, float &roll, float &pitch, float &yaw) {
  roll = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                1.0f - 2.0f * (q.x * q.x + q.y * q.y));
  float s = 2.0f * (q.w * q.y - q.z * q.x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  pitch = asinf(s);
  yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
               1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

// Quaternion for a given roll and pitch, yaw = 0 (angles in radians)
Quat quatFromRollPitch(float roll, float pitch) {
  float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  Quat q = { cr * cp, sr * cp, cr * sp, -sr * sp };
  return q;
}

// Roll / pitch from the accelerometer alone (radians). Valid only when the
// board is not accelerating, because it assumes the sensor measures gravity.
float accelRoll(const float a[3])  { return atan2f(-a[1], -a[2]); }
float accelPitch(const float a[3]) { return atan2f(a[0], sqrtf(a[1] * a[1] + a[2] * a[2])); }

// Complementary filter: integrate the gyro, but nudge the result toward the
// direction of gravity measured by the accelerometer.
void updateComplementary(Quat &q, const float g[3], const float a[3], float dt) {
  float w[3] = { g[0], g[1], g[2] };

  float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (fabsf(norm - 1.0f) < ACC_GATE) {
    // measured "down" direction in the body frame (unit vector)
    float mx = -a[0] / norm, my = -a[1] / norm, mz = -a[2] / norm;
    // "down" direction in the body frame according to our current estimate
    float vx = 2.0f * (q.x * q.z - q.w * q.y);
    float vy = 2.0f * (q.y * q.z + q.w * q.x);
    float vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    // error = measured x estimated (axis that rotates the estimate toward the measurement)
    float ex = my * vz - mz * vy;
    float ey = mz * vx - mx * vz;
    float ez = mx * vy - my * vx;
    w[0] += COMP_KP * ex;
    w[1] += COMP_KP * ey;
    w[2] += COMP_KP * ez;
  }
  quatIntegrate(q, w, dt);
}
// ---------------------------------------------------------------------
// END MATH
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// BEGIN EKF  (7 states: quaternion q0..q3 and gyro bias bx, by, bz)
// ---------------------------------------------------------------------
// The model:   q_next = q + 0.5 * dt * q (x) (0, w - b)      b_next = b
// The sensor:  accelerometer direction = -(gravity direction in body frame)

// f(x, w): propagate the state one step using the gyro reading w
static void ekfPropagate(const float x[7], const float w[3], float dt, float out[7]) {
  float vx = w[0] - x[4], vy = w[1] - x[5], vz = w[2] - x[6];   // bias-corrected rate
  float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
  float h = 0.5f * dt;
  out[0] = q0 + h * (-q1 * vx - q2 * vy - q3 * vz);
  out[1] = q1 + h * ( q0 * vx + q2 * vz - q3 * vy);
  out[2] = q2 + h * ( q0 * vy - q1 * vz + q3 * vx);
  out[3] = q3 + h * ( q0 * vz + q1 * vy - q2 * vx);
  out[4] = x[4]; out[5] = x[5]; out[6] = x[6];
}

// F = d f / d x   (7x7): how a small error in the state turns into an error one step later
static void ekfJacobianF(const float x[7], const float w[3], float dt, float F[7][7]) {
  for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) F[i][j] = (i == j) ? 1.0f : 0.0f;
  float vx = w[0] - x[4], vy = w[1] - x[5], vz = w[2] - x[6];
  float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
  float h = 0.5f * dt;
  // quaternion block: I + 0.5*dt*A(v)
  F[0][1] = -h * vx;  F[0][2] = -h * vy;  F[0][3] = -h * vz;
  F[1][0] =  h * vx;  F[1][2] =  h * vz;  F[1][3] = -h * vy;
  F[2][0] =  h * vy;  F[2][1] = -h * vz;  F[2][3] =  h * vx;
  F[3][0] =  h * vz;  F[3][1] =  h * vy;  F[3][2] = -h * vx;
  // quaternion-vs-bias block: a bias error acts like a gyro error with the opposite sign
  F[0][4] =  h * q1;  F[0][5] =  h * q2;  F[0][6] =  h * q3;
  F[1][4] = -h * q0;  F[1][5] =  h * q3;  F[1][6] = -h * q2;
  F[2][4] = -h * q3;  F[2][5] = -h * q0;  F[2][6] =  h * q1;
  F[3][4] =  h * q2;  F[3][5] = -h * q1;  F[3][6] = -h * q0;
}

// h(x): the accelerometer direction we EXPECT (unit vector, body frame)
static void ekfMeasure(const float x[7], float out[3]) {
  float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
  out[0] = -2.0f * (q1 * q3 - q0 * q2);
  out[1] = -2.0f * (q2 * q3 + q0 * q1);
  out[2] = -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

// H = d h / d x   (3x7). The biases do not appear in h, so those columns are zero.
static void ekfJacobianH(const float x[7], float H[3][7]) {
  for (int i = 0; i < 3; i++) for (int j = 0; j < 7; j++) H[i][j] = 0.0f;
  float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
  H[0][0] =  2.0f * q2;  H[0][1] = -2.0f * q3;  H[0][2] =  2.0f * q0;  H[0][3] = -2.0f * q1;
  H[1][0] = -2.0f * q1;  H[1][1] = -2.0f * q0;  H[1][2] = -2.0f * q3;  H[1][3] = -2.0f * q2;
  H[2][0] = -2.0f * q0;  H[2][1] =  2.0f * q1;  H[2][2] =  2.0f * q2;  H[2][3] = -2.0f * q3;
}

static bool inv3(const float m[3][3], float o[3][3]) {
  float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
  float c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
  float c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
  float det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
  if (fabsf(det) < 1e-30f) return false;
  float id = 1.0f / det;
  o[0][0] = c00 * id;
  o[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
  o[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
  o[1][0] = c01 * id;
  o[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
  o[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
  o[2][0] = c02 * id;
  o[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
  o[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
  return true;
}

struct EKF {
  float x[7];                  // state: q0 q1 q2 q3 bx by bz   (bias in rad/s)
  float P[7][7];               // state covariance
  float sigmaGyro = 0.001f;    // gyro noise [rad/s]            -> process noise on the quaternion
  float qBias     = 1e-12f;    // bias random walk [(rad/s)^2 per step] -> process noise on the bias
  float rAcc      = 9e-5f;     // accelerometer noise variance [g^2] -> measurement noise R.
                               // A still board measures 9e-6 (sigma 0.003 g), but a board moved
                               // by hand sees much larger errors, so R is inflated 10x.

  // Two safeguards found by testing with hand motion (see the notes in the chat):
  bool lockBiasZ      = true;  // do not estimate the z gyro bias: gravity cannot observe it,
                               // so under hand motion the filter learns a bogus value
  bool tiltOnlyUpdate = true;  // the accelerometer may correct roll and pitch but never yaw

  EKF() {
    for (int i = 0; i < 7; i++) { x[i] = 0; for (int j = 0; j < 7; j++) P[i][j] = 0; }
    x[0] = 1;
  }

  // Start from a given attitude. The learned bias is kept.
  void init(const Quat &q) {
    x[0] = q.w; x[1] = q.x; x[2] = q.y; x[3] = q.z;
    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) P[i][j] = 0;
    for (int i = 0; i < 4; i++) P[i][i] = 1e-4f;
    for (int i = 4; i < 7; i++) P[i][i] = 1e-6f;
    if (lockBiasZ) P[6][6] = 1e-12f;     // "we know bz exactly": the gain for it stays ~0
  }

  Quat quat() const { Quat q = { x[0], x[1], x[2], x[3] }; return q; }

  void normalizeQuat() {
    float n = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
    for (int i = 0; i < 4; i++) x[i] /= n;
  }

  // ---- PREDICT: x = f(x, w),  P = F P F^T + Q -------------------------
  void predict(const float w[3], float dt) {
    float F[7][7], FP[7][7], Pn[7][7], xn[7];
    ekfJacobianF(x, w, dt, F);

    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) {
      float s = 0; for (int k = 0; k < 7; k++) s += F[i][k] * P[k][j];
      FP[i][j] = s;
    }
    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) {
      float s = 0; for (int k = 0; k < 7; k++) s += FP[i][k] * F[j][k];   // FP * F^T
      Pn[i][j] = s;
    }

    ekfPropagate(x, w, dt, xn);
    for (int i = 0; i < 7; i++) x[i] = xn[i];
    normalizeQuat();

    // Q: gyro noise enters the quaternion through G (4x3), and G G^T = 0.25 (I - q q^T)
    float g2 = sigmaGyro * dt;  g2 *= g2;
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++)
      Pn[i][j] += g2 * 0.25f * (((i == j) ? 1.0f : 0.0f) - x[i] * x[j]);
    for (int i = 4; i < 7; i++) {
      if (i == 6 && lockBiasZ) continue;
      Pn[i][i] += qBias;
    }

    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) P[i][j] = Pn[i][j];
  }

  // ---- UPDATE with the accelerometer (any scale, it gets normalised) --
  void update(const float acc[3]) {
    float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n < 1e-6f) return;
    float z[3] = { acc[0] / n, acc[1] / n, acc[2] / n };

    float hx[3], H[3][7];
    ekfMeasure(x, hx);
    ekfJacobianH(x, H);
    float y[3] = { z[0] - hx[0], z[1] - hx[1], z[2] - hx[2] };       // innovation

    float PHt[7][3];                                                 // P H^T
    for (int i = 0; i < 7; i++) for (int j = 0; j < 3; j++) {
      float s = 0; for (int k = 0; k < 4; k++) s += P[i][k] * H[j][k];
      PHt[i][j] = s;
    }
    float S[3][3], Si[3][3];                                         // S = H P H^T + R
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
      float s = 0; for (int k = 0; k < 4; k++) s += H[i][k] * PHt[k][j];
      S[i][j] = s + ((i == j) ? rAcc : 0.0f);
    }
    if (!inv3(S, Si)) return;

    float K[7][3];                                                   // K = P H^T S^-1
    for (int i = 0; i < 7; i++) for (int j = 0; j < 3; j++) {
      float s = 0; for (int k = 0; k < 3; k++) s += PHt[i][k] * Si[k][j];
      K[i][j] = s;
    }

    float qBefore[4] = { x[0], x[1], x[2], x[3] };
    for (int i = 0; i < 7; i++)                                      // x = x + K y
      x[i] += K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];

    if (tiltOnlyUpdate) {
      // Gravity says nothing about yaw, so remove the yaw part of the correction.
      // e = direction in quaternion space that a pure yaw rotation moves q along.
      float e[4] = { -qBefore[3], -qBefore[2], qBefore[1], qBefore[0] };
      float dq[4] = { x[0] - qBefore[0], x[1] - qBefore[1], x[2] - qBefore[2], x[3] - qBefore[3] };
      float dot = dq[0] * e[0] + dq[1] * e[1] + dq[2] * e[2] + dq[3] * e[3];
      for (int i = 0; i < 4; i++) x[i] -= dot * e[i];
    }
    normalizeQuat();

    // P = (I - K H) P (I - K H)^T + K R K^T   (Joseph form: stays symmetric / positive)
    float A[7][7], AP[7][7], Pn[7][7];
    for (int i = 0; i < 7; i++) for (int k = 0; k < 7; k++) {
      float a = (i == k) ? 1.0f : 0.0f;
      if (k < 4) { float s = 0; for (int j = 0; j < 3; j++) s += K[i][j] * H[j][k]; a -= s; }
      A[i][k] = a;
    }
    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) {
      float s = 0; for (int k = 0; k < 7; k++) s += A[i][k] * P[k][j];
      AP[i][j] = s;
    }
    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) {
      float s = 0; for (int k = 0; k < 7; k++) s += AP[i][k] * A[j][k];
      float kk = 0; for (int m = 0; m < 3; m++) kk += K[i][m] * K[j][m];
      Pn[i][j] = s + rAcc * kk;
    }
    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++)
      P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);                        // keep P symmetric

    // A quaternion has 4 numbers but only 3 degrees of freedom, so its covariance is
    // slightly "flat" in one direction and float rounding can push a variance a hair
    // below zero. A tiny floor keeps every variance valid.
    for (int i = 0; i < 7; i++) if (P[i][i] < 1e-12f) P[i][i] = 1e-12f;
  }
};
// ---------------------------------------------------------------------
// END EKF
// ---------------------------------------------------------------------

Quat   qGyro = {1, 0, 0, 0};   // gyro-only estimate
Quat   qComp = {1, 0, 0, 0};   // complementary filter estimate
EKF    ekf;
float  gyroBias[3] = {0, 0, 0};
float  startAcc[3] = {0, 0, 1};
uint32_t nextUs = 0, lastUs = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Returns calibrated accel [g] and gyro [rad/s], already converted to BODY axes.
bool readIMU(float a[3], float g[3]) {
  uint8_t b[14];
  if (!readBurst(REG_ACCEL_XOUT_H, b, 14)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };

  float ar[3], gr[3];   // sensor-frame values
  for (int i = 0; i < 3; i++) {
    ar[i] = (s16(2 * i) / ACCEL_LSB - ACC_OFFSET[i]) * ACC_SCALE[i];
    gr[i] = s16(8 + 2 * i) / GYRO_LSB * DEG2RAD;
  }
  // sensor frame (X fwd, Y left, Z up)  ->  body frame (X fwd, Y right, Z down)
  a[0] =  ar[0];  a[1] = -ar[1];  a[2] = -ar[2];
  g[0] =  gr[0];  g[1] = -gr[1];  g[2] = -gr[2];
  return true;
}

// Average gyro (-> bias) and accel (-> starting attitude) while the board is still.
void calibrateStartup(uint16_t samples = 400) {
  Serial.println("# Keep the board STILL for 2 s (chip up, X arrow to the nose)...");
  double sg[3] = {0, 0, 0}, sa[3] = {0, 0, 0};
  uint16_t n = 0;
  float a[3], g[3];
  while (n < samples) {
    if (readIMU(a, g)) {
      for (int i = 0; i < 3; i++) { sg[i] += g[i]; sa[i] += a[i]; }
      n++;
    }
    delay(5);
  }
  for (int i = 0; i < 3; i++) { gyroBias[i] = sg[i] / n; startAcc[i] = sa[i] / n; }
  Serial.print("# Gyro bias [rad/s]: ");
  Serial.print(gyroBias[0], 6); Serial.print(", ");
  Serial.print(gyroBias[1], 6); Serial.print(", ");
  Serial.println(gyroBias[2], 6);
}

// Start both estimators from the gravity direction (yaw = 0)
void resetAttitude(const float a[3]) {
  Quat q = quatFromRollPitch(accelRoll(a), accelPitch(a));
  qGyro = q;
  qComp = q;
  ekf.init(q);
}

void setup() {
  Serial.begin(460800);
  delay(500);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Wire.setClock(400000);

  writeReg(REG_PWR_MGMT_1, 0x01);
  delay(100);

  uint8_t id = 0;
  readBurst(REG_WHO_AM_I, &id, 1);
  Serial.print("# WHO_AM_I = 0x"); Serial.println(id, HEX);
  if (id == 0) {
    Serial.println("# ERROR: no response. Check wiring / address.");
    while (true) delay(1000);
  }

  writeReg(REG_CONFIG, 0x03);        // low-pass ~42 Hz
  writeReg(REG_SMPLRT_DIV, 9);       // 100 Hz
  writeReg(REG_GYRO_CONFIG, 0x08);   // +-500 deg/s
  writeReg(REG_ACCEL_CONFIG, 0x00);  // +-2 g
  delay(100);

  calibrateStartup();
  resetAttitude(startAcc);

  Serial.println("# Q,t_ms, gyro(w,x,y,z), comp(w,x,y,z), ekf(w,x,y,z), ekfMicros");

  lastUs = micros();
  nextUs = lastUs + LOOP_US;
}

void printQuat(const Quat &q) {
  Serial.print(','); Serial.print(q.w, 4);
  Serial.print(','); Serial.print(q.x, 4);
  Serial.print(','); Serial.print(q.y, 4);
  Serial.print(','); Serial.print(q.z, 4);
}

void loop() {
  // fixed-rate scheduler: keeps dt constant at 10 ms
  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += LOOP_US;

  // measure the real time step instead of assuming exactly 10 ms
  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  if (dt > 0.05f) dt = 0.05f;

  float a[3], g[3];
  if (!readIMU(a, g)) return;          // skip the sample if I2C failed
  for (int i = 0; i < 3; i++) g[i] -= gyroBias[i];

  if (Serial.available()) {            // any character = re-zero
    while (Serial.available()) Serial.read();
    resetAttitude(a);
  }

  // 1) gyro only: pure integration, no correction
  quatIntegrate(qGyro, g, dt);
  // 2) complementary filter: integration + accelerometer correction
  updateComplementary(qComp, g, a, dt);
  // 3) EKF: predict with the gyro every sample, update with the accelerometer when it is trustworthy
  uint32_t t0 = micros();
  ekf.predict(g, dt);
  float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (fabsf(an - 1.0f) < ACC_GATE) ekf.update(a);
  uint32_t ekfUs = micros() - t0;

  Serial.print("Q,"); Serial.print(millis());
  printQuat(qGyro); printQuat(qComp); printQuat(ekf.quat());
  Serial.print(','); Serial.print(ekfUs);
  Serial.println();
}
