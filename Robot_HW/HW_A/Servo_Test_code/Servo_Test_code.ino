// ME4640 HW-A Part 6: Sinusoidal Servo Motion (4 servos)
// Uses LINEAR calibration:  us = m*theta + b
// Logs: t_ms, theta_deg, us1, us2, us3, us4

#include <Servo.h>

// =======================
// SINE SETTINGS
// =======================
const float AMPLITUDE_DEG = 60.0f;   // +/- 60 degrees
const float CENTER_DEG    = 90.0f;   // center angle (deg)
const float FREQ_HZ       = 0.50f;   // <-- set to your assigned frequency
const unsigned long RUN_MS = 4000;   // run for 4 seconds
const unsigned long DT_MS  = 20;     // 20ms sample period (~50 Hz)

// =======================
// SERVO PINS (edit if needed)
// =======================
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 5;
const int SERVO4_PIN = 6;

// =======================
// SERVO SAFE PULSE LIMITS (microseconds)
// EDIT THESE TO YOUR MEASURED MIN/MAX FOR EACH SERVO
// =======================
const int S1_MIN_US = 579,  S1_MAX_US = 2560;
const int S2_MIN_US = 473,  S2_MAX_US = 2473;
const int S3_MIN_US = 553,  S3_MAX_US = 2500;
// IMPORTANT: Servo 4 intercept is negative; you MUST have a real min_us here.
// If you truly measured a different min, replace 500 with it.
const int S4_MIN_US = 500,  S4_MAX_US = 2173;

// =======================
// CALIBRATION EQUATIONS (LINEAR)
// us = m*theta + b
// =======================
const float S1_M = 11.4033f;  const float S1_B = 531.24f;
const float S2_M = 11.1111f;  const float S2_B = 473.00f;
const float S3_M = 10.8484f;  const float S3_B = 551.44f;
const float S4_M = 12.3308f;  const float S4_B = -1.40f;

// =======================
Servo s1, s2, s3, s4;

static inline int thetaToUS_linear(float thetaDeg, float m, float b, int usMin, int usMax) {
  float us = m * thetaDeg + b;
  if (us < usMin) us = (float)usMin;
  if (us > usMax) us = (float)usMax;
  return (int)(us + 0.5f); // round to nearest int
}

void setup() {
  Serial.begin(9600);
  delay(1500);

  // Attach servos with limits
  s1.attach(SERVO1_PIN, S1_MIN_US, S1_MAX_US);
  s2.attach(SERVO2_PIN, S2_MIN_US, S2_MAX_US);
  s3.attach(SERVO3_PIN, S3_MIN_US, S3_MAX_US);
  s4.attach(SERVO4_PIN, S4_MIN_US, S4_MAX_US);

  // Move all to center initially
  int us1c = thetaToUS_linear(CENTER_DEG, S1_M, S1_B, S1_MIN_US, S1_MAX_US);
  int us2c = thetaToUS_linear(CENTER_DEG, S2_M, S2_B, S2_MIN_US, S2_MAX_US);
  int us3c = thetaToUS_linear(CENTER_DEG, S3_M, S3_B, S3_MIN_US, S3_MAX_US);
  int us4c = thetaToUS_linear(CENTER_DEG, S4_M, S4_B, S4_MIN_US, S4_MAX_US);

  s1.writeMicroseconds(us1c);
  s2.writeMicroseconds(us2c);
  s3.writeMicroseconds(us3c);
  s4.writeMicroseconds(us4c);

  // Header for MATLAB copy/paste
  Serial.println("t_ms\ttheta_deg\tus1\tus2\tus3\tus4");
}

void loop() {
  static bool ranOnce = false;
  if (ranOnce) return;
  ranOnce = true;

  unsigned long t0 = millis();
  unsigned long last = t0;

  while (millis() - t0 <= RUN_MS) {
    unsigned long now = millis();
    if (now - last < DT_MS) continue;
    last = now;

    float t = (now - t0) / 1000.0f; // seconds
    float theta = CENTER_DEG + AMPLITUDE_DEG * sinf(2.0f * PI * FREQ_HZ * t);

    // Convert theta -> microseconds (per-servo calibration)
    int us1 = thetaToUS_linear(theta, S1_M, S1_B, S1_MIN_US, S1_MAX_US);
    int us2 = thetaToUS_linear(theta, S2_M, S2_B, S2_MIN_US, S2_MAX_US);
    int us3 = thetaToUS_linear(theta, S3_M, S3_B, S3_MIN_US, S3_MAX_US);
    int us4 = thetaToUS_linear(theta, S4_M, S4_B, S4_MIN_US, S4_MAX_US);

    // Command servos
    s1.writeMicroseconds(us1);
    s2.writeMicroseconds(us2);
    s3.writeMicroseconds(us3);
    s4.writeMicroseconds(us4);

    // Log row: time, commanded theta, commanded us per servo
    Serial.print(now - t0); Serial.print('\t');
    Serial.print(theta, 2); Serial.print('\t');
    Serial.print(us1); Serial.print('\t');
    Serial.print(us2); Serial.print('\t');
    Serial.print(us3); Serial.print('\t');
    Serial.print(us4); Serial.println();
  }

  // Hold at center at end
  int us1c = thetaToUS_linear(CENTER_DEG, S1_M, S1_B, S1_MIN_US, S1_MAX_US);
  int us2c = thetaToUS_linear(CENTER_DEG, S2_M, S2_B, S2_MIN_US, S2_MAX_US);
  int us3c = thetaToUS_linear(CENTER_DEG, S3_M, S3_B, S3_MIN_US, S3_MAX_US);
  int us4c = thetaToUS_linear(CENTER_DEG, S4_M, S4_B, S4_MIN_US, S4_MAX_US);

  s1.writeMicroseconds(us1c);
  s2.writeMicroseconds(us2c);
  s3.writeMicroseconds(us3c);
  s4.writeMicroseconds(us4c);
}
