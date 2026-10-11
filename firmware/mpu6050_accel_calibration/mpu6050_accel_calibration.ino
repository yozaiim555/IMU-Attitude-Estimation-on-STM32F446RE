/*
  6-position accelerometer calibration for MPU6050 (GY-521)
  Board: Nucleo F446RE (STM32duino core)

  How it works
  ------------
  Gravity is a known reference of exactly 1 g. If you hold each axis pointing
  straight up and then straight down, the sensor should read +1 and -1.
  Whatever it actually reads gives us, per axis:

      offset = (max + min) / 2        (zero-g error)
      scale  = 2 / (max - min)        (sensitivity error)
      corrected = (raw - offset) * scale

  Usage
  -----
  1. Upload, open serial monitor at 460800 baud.
  2. Make sure the line ending dropdown is set to anything (Newline is fine).
  3. For each of the 6 poses: place the board on a flat, hard surface in the
     requested orientation, keep it STILL, then type any character + Enter
     (or just press Enter / click Send).
  4. Copy the printed constants into your main sketch.

  Tip: lean the board against a book or a box corner for the side poses.
  A pose that is a few degrees off is fine; the live readout shows you
  when the requested axis reads close to +-1 g.
*/

#include <Wire.h>

constexpr uint8_t MPU_ADDR  = 0x68;
constexpr float   ACCEL_LSB = 16384.0f;   // +-2 g
constexpr int     N_SAMPLES = 300;        // about 1.5 s per pose

struct Pose {
  int axis;          // 0=X, 1=Y, 2=Z
  int sign;          // +1 = axis points up, -1 = axis points down
  const char *name;
};

const Pose POSES[6] = {
  {2, +1, "Z UP    : board flat on the table, chip side facing up"},
  {2, -1, "Z DOWN  : board flat on the table, upside down"},
  {0, +1, "X UP    : stand the board on its edge so the X arrow points UP"},
  {0, -1, "X DOWN  : stand the board on its edge so the X arrow points DOWN"},
  {1, +1, "Y UP    : stand the board on its edge so the Y arrow points UP"},
  {1, -1, "Y DOWN  : stand the board on its edge so the Y arrow points DOWN"},
};

float mean[6][3];   // averaged accel reading for each pose

bool readAccel(float a[3]) {
  uint8_t b[6];
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++)
    a[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / ACCEL_LSB;
  return true;
}

void flushSerialInput() {
  while (Serial.available()) Serial.read();
}

// Shows live readings until the user presses Enter.
void waitForEnter() {
  flushSerialInput();
  uint32_t last = 0;
  while (!Serial.available()) {
    if (millis() - last > 300) {
      last = millis();
      float a[3];
      if (readAccel(a)) {
        Serial.print("   live  x=");  Serial.print(a[0], 3);
        Serial.print("  y=");         Serial.print(a[1], 3);
        Serial.print("  z=");         Serial.println(a[2], 3);
      }
    }
  }
  flushSerialInput();
}

void capturePose(float out[3]) {
  delay(800);   // let the board settle after you let go of it
  double sum[3] = {0, 0, 0};
  int n = 0;
  while (n < N_SAMPLES) {
    float a[3];
    if (readAccel(a)) {
      for (int i = 0; i < 3; i++) sum[i] += a[i];
      n++;
    }
    delay(5);
  }
  for (int i = 0; i < 3; i++) out[i] = sum[i] / n;
}

// Checks the requested axis really is the one pointing up/down.
bool poseLooksRight(const Pose &p, const float m[3]) {
  if (fabsf(m[p.axis]) < 0.7f) return false;
  if ((m[p.axis] > 0) != (p.sign > 0)) return false;
  for (int i = 0; i < 3; i++)
    if (i != p.axis && fabsf(m[i]) > 0.4f) return false;
  return true;
}

void setup() {
  Serial.begin(460800);
  delay(500);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);   // wake up, gyro X PLL clock
  Wire.write(0x6B); Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);   // low-pass filter ~42 Hz for less noise
  Wire.write(0x1A); Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);   // accel range +-2 g
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission();
  delay(200);

  float test[3];
  if (!readAccel(test)) {
    Serial.println("ERROR: cannot talk to the MPU6050. Check wiring.");
    while (true) delay(1000);
  }

  Serial.println();
  Serial.println("=== 6-position accelerometer calibration ===");
  Serial.println("For each pose: orient the board, keep it still, press Enter.");
  Serial.println();

  for (int p = 0; p < 6; p++) {
    bool ok = false;
    while (!ok) {
      Serial.print("Pose "); Serial.print(p + 1); Serial.println("/6");
      Serial.println(POSES[p].name);
      Serial.println("Hold still, then press Enter...");
      waitForEnter();
      Serial.println("Capturing, do not touch the board...");
      capturePose(mean[p]);

      ok = poseLooksRight(POSES[p], mean[p]);
      if (!ok) {
        Serial.print("Reading was x=");  Serial.print(mean[p][0], 3);
        Serial.print(" y=");             Serial.print(mean[p][1], 3);
        Serial.print(" z=");             Serial.println(mean[p][2], 3);
        Serial.println("That does not match the requested pose. Try again.\n");
      } else {
        Serial.println("OK\n");
      }
    }
  }

  // Compute offset and scale per axis
  float offset[3], scale[3];
  for (int axis = 0; axis < 3; axis++) {
    float hi = 0, lo = 0;
    for (int p = 0; p < 6; p++) {
      if (POSES[p].axis == axis) {
        if (POSES[p].sign > 0) hi = mean[p][axis];
        else                   lo = mean[p][axis];
      }
    }
    offset[axis] = (hi + lo) / 2.0f;
    scale[axis]  = 2.0f / (hi - lo);
  }

  Serial.println("=== RESULT: paste this into your main sketch ===");
  Serial.print("constexpr float ACC_OFFSET[3] = {");
  Serial.print(offset[0], 6); Serial.print("f, ");
  Serial.print(offset[1], 6); Serial.print("f, ");
  Serial.print(offset[2], 6); Serial.println("f};");
  Serial.print("constexpr float ACC_SCALE[3]  = {");
  Serial.print(scale[0], 6);  Serial.print("f, ");
  Serial.print(scale[1], 6);  Serial.print("f, ");
  Serial.print(scale[2], 6);  Serial.println("f};");
  Serial.println();

  // Verification: after correction every pose should have |a| close to 1.000
  Serial.println("=== Check: corrected magnitude per pose (target 1.000) ===");
  for (int p = 0; p < 6; p++) {
    float s = 0;
    for (int i = 0; i < 3; i++) {
      float c = (mean[p][i] - offset[i]) * scale[i];
      s += c * c;
    }
    Serial.print("Pose "); Serial.print(p + 1);
    Serial.print(": "); Serial.println(sqrtf(s), 4);
  }
  Serial.println("\nDone.");
}

void loop() {
}
