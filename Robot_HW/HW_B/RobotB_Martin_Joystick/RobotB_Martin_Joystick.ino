// ============================================================
// STUDENT NAME: Paul Martin
// ME4640 Robot Homework B - Parts 1 & 2
// Joystick Control with IIR Filtering, Deadband, and ROM Limits
// ============================================================

#include <Servo.h>

// ===== SERVO PIN ASSIGNMENTS (HRIB Shield) =====
const int PIN_BASE     = 3;   // Joint 1 - Base      (Servo 1)
const int PIN_SHOULDER = 5;   // Joint 2 - Shoulder  (Servo 2)
const int PIN_ELBOW    = 6;   // Joint 3 - Elbow     (Servo 3)
const int PIN_GRIPPER  = 9;   // Joint 4 - Gripper   (Servo 4)

// ===== JOYSTICK ADC PIN ASSIGNMENTS (HRIB Shield) =====
const int JOY_BASE     = A2;  // Base axis
const int JOY_SHOULDER = A3;  // Shoulder axis
const int JOY_ELBOW    = A0;  // Elbow axis
const int JOY_GRIPPER  = A1;  // Gripper axis

// ===== OPTIONAL: HOME-RESET BUTTON =====
// Uncomment and set correct pin if using a push button on your HRIB shield
 const int BTN_HOME = 2;

// ===== SERVO PULSE LIMITS (from HW A calibration) =====
const int MIN_US_1 = 579;    // Base     min pulse (µs)
const int MAX_US_1 = 2560;   // Base     max pulse (µs)
const int MIN_US_2 = 473;    // Shoulder min pulse (µs)
const int MAX_US_2 = 2473;   // Shoulder max pulse (µs)
const int MIN_US_3 = 553;    // Elbow    min pulse (µs)
const int MAX_US_3 = 2500;   // Elbow    max pulse (µs)
const int MIN_US_4 = 700;    // Gripper  min pulse (µs)  <-- adjust if needed
const int MAX_US_4 = 2173;   // Gripper  max pulse (µs)

// ===== CALIBRATION COEFFICIENTS (from HW A, linear fit: angle = SLOPE*us + INTERCEPT) =====
const float SLOPE_1     = 0.0909f;   // deg/µs  Joint 1 Base
const float INTERCEPT_1 = -52.7f;   // deg     Joint 1 Base
const float SLOPE_2     = 0.0909f;   // deg/µs  Joint 2 Shoulder
const float INTERCEPT_2 = -52.7f;   // deg     Joint 2 Shoulder
const float SLOPE_3     = 0.0909f;   // deg/µs  Joint 3 Elbow
const float INTERCEPT_3 = -52.7f;   // deg     Joint 3 Elbow
const float SLOPE_4     = 0.0909f;   // deg/µs  Joint 4 Gripper
const float INTERCEPT_4 = -52.7f;   // deg     Joint 4 Gripper

// ===== HOME POSITION OFFSETS (Part 2) =====
// INSTRUCTIONS: Use joystick to move robot to the home position shown in Figure 1.
// Read the microsecond values from the Serial Monitor, then update these constants.
// These values make theta = 0 deg correspond to the physical home pose.
const int HOME_US_1 = 1646;  // Base     home microseconds  <-- UPDATE AFTER PHYSICAL CALIBRATION
const int HOME_US_2 = 1473;  // Shoulder home microseconds  <-- UPDATE AFTER PHYSICAL CALIBRATION
const int HOME_US_3 = 1061;  // Elbow    home microseconds  <-- UPDATE AFTER PHYSICAL CALIBRATION
const int HOME_US_4 = 1168;  // Gripper  home microseconds  <-- UPDATE AFTER PHYSICAL CALIBRATION

// ===== ROM LIMITS (degrees, referenced from home = 0 deg) =====
const float ROM_MIN_1 = -80.0f;   // Base     min angle
const float ROM_MAX_1 =  80.0f;   // Base     max angle
const float ROM_MIN_2 = -30.0f;   // Shoulder min angle (avoid driving into base)
const float ROM_MAX_2 =  60.0f;   // Shoulder max angle
const float ROM_MIN_3 = -60.0f;   // Elbow    min angle
const float ROM_MAX_3 =  60.0f;   // Elbow    max angle
const float ROM_MIN_4 =   0.0f;   // Gripper  min angle (closed)
const float ROM_MAX_4 =  50.0f;   // Gripper  max angle (open)

// ===== IIR FILTER & JOYSTICK PARAMETERS =====
const float ALPHA     = 0.10f;  // IIR smoothing factor (0.05 to 0.15 recommended)
const int   DEADBAND  = 20;     // Joystick deadband around center (ADC units)
const float K_BASE     = 0.003f; // Joystick sensitivity (deg per ADC unit per loop)
const float K_SHOULDER = 0.003f;
const float K_ELBOW    = 0.003f;
const float K_GRIPPER  = 0.003f;

// ===== LOOP TIMING =====
const int LOOP_MS = 20;   // 20 ms loop = 50 Hz

// ===== SERVO OBJECTS =====
Servo servo1, servo2, servo3, servo4;

// ===== RUNTIME STATE =====
// Joint angles relative to home (deg)
float th1 = 0.0f;
float th2 = 0.0f;
float th3 = 0.0f;
float th4 = 0.0f;

// IIR filter states (filtered joystick deviations)
float j1_filt = 0.0f;
float j2_filt = 0.0f;
float j3_filt = 0.0f;
float j4_filt = 0.0f;

// ============================================================
// HELPER: Convert home-relative angle (deg) → microseconds
// Derived from:  raw_angle = SLOPE*us + INTERCEPT
//          0 deg = SLOPE*home_us + INTERCEPT  (at home)
// Therefore:  us = home_us + theta / SLOPE
// ============================================================
int angleToPulse(float theta, float slope, int home_us, int minUs, int maxUs) {
  int us = home_us + (int)(theta / slope);
  return constrain(us, minUs, maxUs);
}

// ============================================================
// HELPER: Convert microseconds → home-relative angle (deg)
// ============================================================
float pulseToAngle(int us, float slope, int home_us) {
  return slope * (us - home_us);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(230400);

  // Attach servos with hardware pulse limits
  servo1.attach(PIN_BASE,     MIN_US_1, MAX_US_1);
  servo2.attach(PIN_SHOULDER, MIN_US_2, MAX_US_2);
  servo3.attach(PIN_ELBOW,    MIN_US_3, MAX_US_3);
  servo4.attach(PIN_GRIPPER,  MIN_US_4, MAX_US_4);

  // Optional home-reset button
   pinMode(BTN_HOME, INPUT_PULLUP);

  // Move all joints to home position
  servo1.writeMicroseconds(HOME_US_1);
  servo2.writeMicroseconds(HOME_US_2);
  servo3.writeMicroseconds(HOME_US_3);
  servo4.writeMicroseconds(HOME_US_4);
  delay(1000);  // Allow servos to reach home before accepting joystick input

  // Print header
  Serial.println("=================================================================");
  Serial.println("ME4640 HW-B  |  Joystick Control  |  Paul Martin");
  Serial.println("=================================================================");
  Serial.println("th1_us\tth2_us\tth3_us\tth4_us\tth1_deg\tth2_deg\tth3_deg\tth4_deg");
  Serial.println("-----------------------------------------------------------------");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long t_start = millis();

  // --- 1. READ JOYSTICK ADC VALUES ---
  int raw1 = analogRead(JOY_BASE);
  int raw2 = analogRead(JOY_SHOULDER);
  int raw3 = analogRead(JOY_ELBOW);
  int raw4 = analogRead(JOY_GRIPPER);

  // --- 2. APPLY DEADBAND (set to 512 if within deadband of center) ---
  if (abs(raw1 - 512) < DEADBAND) raw1 = 512;
  if (abs(raw2 - 512) < DEADBAND) raw2 = 512;
  if (abs(raw3 - 512) < DEADBAND) raw3 = 512;
  if (abs(raw4 - 512) < DEADBAND) raw4 = 512;

  // Compute signed deviation from center (-512 to +512)
  float dev1 = (float)(raw1 - 512);
  float dev2 = (float)(raw2 - 512);
  float dev3 = (float)(raw3 - 512);
  float dev4 = (float)(raw4 - 512);

  // --- 3. APPLY IIR LOW-PASS FILTER ---
  j1_filt = (1.0f - ALPHA) * j1_filt + ALPHA * dev1;
  j2_filt = (1.0f - ALPHA) * j2_filt + ALPHA * dev2;
  j3_filt = (1.0f - ALPHA) * j3_filt + ALPHA * dev3;
  j4_filt = (1.0f - ALPHA) * j4_filt + ALPHA * dev4;

  // --- 4. UPDATE JOINT ANGLES (velocity / rate control) ---
  th1 += K_BASE     * j1_filt;
  th2 += K_SHOULDER * j2_filt;
  th3 += K_ELBOW    * j3_filt;
  th4 += K_GRIPPER  * j4_filt;

  // --- 5. APPLY ROM LIMITS ---
  th1 = constrain(th1, ROM_MIN_1, ROM_MAX_1);
  th2 = constrain(th2, ROM_MIN_2, ROM_MAX_2);
  th3 = constrain(th3, ROM_MIN_3, ROM_MAX_3);
  th4 = constrain(th4, ROM_MIN_4, ROM_MAX_4);

  // --- OPTIONAL: HOME RESET BUTTON ---
  if (digitalRead(BTN_HOME) == LOW) {
     th1 = 0.0f;  th2 = 0.0f;  th3 = 0.0f;  th4 = 0.0f;
     j1_filt = 0.0f;  j2_filt = 0.0f;  j3_filt = 0.0f;  j4_filt = 0.0f;
  }

  // --- 6. CONVERT ANGLES TO MICROSECONDS ---
  int us1 = angleToPulse(th1, SLOPE_1, HOME_US_1, MIN_US_1, MAX_US_1);
  int us2 = angleToPulse(th2, SLOPE_2, HOME_US_2, MIN_US_2, MAX_US_2);
  int us3 = angleToPulse(th3, SLOPE_3, HOME_US_3, MIN_US_3, MAX_US_3);
  int us4 = angleToPulse(th4, SLOPE_4, HOME_US_4, MIN_US_4, MAX_US_4);

  // --- 7. COMMAND SERVOS ---
  servo1.writeMicroseconds(us1);
  servo2.writeMicroseconds(us2);
  servo3.writeMicroseconds(us3);
  servo4.writeMicroseconds(us4);

  // --- 8. SERIAL OUTPUT (microseconds and degrees) ---
  // Convert back from constrained µs to actual commanded angle for accurate display
  float disp1 = pulseToAngle(us1, SLOPE_1, HOME_US_1);
  float disp2 = pulseToAngle(us2, SLOPE_2, HOME_US_2);
  float disp3 = pulseToAngle(us3, SLOPE_3, HOME_US_3);
  float disp4 = pulseToAngle(us4, SLOPE_4, HOME_US_4);

  Serial.print(us1);   Serial.print("\t");
  Serial.print(us2);   Serial.print("\t");
  Serial.print(us3);   Serial.print("\t");
  Serial.print(us4);   Serial.print("\t");
  Serial.print(disp1, 1); Serial.print("\t");
  Serial.print(disp2, 1); Serial.print("\t");
  Serial.print(disp3, 1); Serial.print("\t");
  Serial.println(disp4, 1);

  // --- MAINTAIN LOOP TIMING ---
  long elapsed = (long)(millis() - t_start);
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}
