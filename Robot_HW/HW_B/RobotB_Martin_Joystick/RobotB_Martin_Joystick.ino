// ============================================================
// STUDENT NAME: Paul Martin
// ME4640 Robot Homework B - Parts 1 & 2
// Joystick Control with IIR Filtering, Deadband, and ROM Limits
// ============================================================

#include <Servo.h>

// ===== SERVO PIN ASSIGNMENTS =====
const int PIN_BASE     = 3;   // Joint 1 - Base
const int PIN_SHOULDER = 5;   // Joint 2 - Shoulder
const int PIN_ELBOW    = 6;   // Joint 3 - Elbow
const int PIN_GRIPPER  = 9;   // Joint 4 - Gripper

// ===== JOYSTICK ADC PIN ASSIGNMENTS =====
const int JOY_BASE     = A2;  // Base axis
const int JOY_SHOULDER = A3;  // Shoulder axis
const int JOY_ELBOW    = A0;  // Elbow axis
const int JOY_GRIPPER  = A1;  // Gripper axis

// ===== HOME-RESET BUTTON =====
const int BTN_HOME = 12;

// ===== SERVO PULSE LIMITS =====
// Hardware limits from HW A calibration
const int MIN_US_1 = 579;
const int MAX_US_1 = 2560;
const int MIN_US_2 = 473;
const int MAX_US_2 = 2473;
const int MIN_US_3 = 553;
const int MAX_US_3 = 2500;
const int MIN_US_4 = 1600;
const int MAX_US_4 = 2400;

// ===== CALIBRATION COEFFICIENTS =====
// Linear fit: angle = SLOPE*us + INTERCEPT
const float SLOPE_1     = 0.097f;
const float INTERCEPT_1 = -48.44f;
const float SLOPE_2     = 0.0909f;
const float INTERCEPT_2 = -52.7f;
const float SLOPE_3     = 0.0909f;
const float INTERCEPT_3 = -52.7f;
const float SLOPE_4     = 0.0909f;
const float INTERCEPT_4 = -52.7f;

// ===== HOME POSITION OFFSETS =====
// Microsecond values that correspond to theta = 0 deg
const int HOME_US_1 = 1523;
const int HOME_US_2 = 1473;
const int HOME_US_3 = 1061;
const int HOME_US_4 = 2000;

// ===== ROM LIMITS =====
// Software angle limits (degrees from home = 0 deg)
const float ROM_MIN_1 = -80.0f;   // Base min
const float ROM_MAX_1 =  80.0f;   // Base max
const float ROM_MIN_2 = -30.0f;   // Shoulder min (avoid collision with base)
const float ROM_MAX_2 =  60.0f;   // Shoulder max
const float ROM_MIN_3 = -60.0f;   // Elbow min
const float ROM_MAX_3 =  60.0f;   // Elbow max
const float ROM_MIN_4 =   0.0f;   // Gripper min (closed)
const float ROM_MAX_4 =  50.0f;   // Gripper max (open)

// ===== IIR FILTER & JOYSTICK PARAMETERS =====
const float ALPHA      =  0.10f;  // IIR smoothing factor (lower = smoother)
const int   DEADBAND   =  20;     // Joystick center deadband (ADC units)
const float K_BASE     =  0.003f; // Joystick sensitivity (deg per ADC unit per loop)
const float K_SHOULDER = -0.003f; // Negative = inverted direction
const float K_ELBOW    = -0.003f; // Negative = inverted direction
const float K_GRIPPER  =  0.003f;

// ===== LOOP TIMING =====
const int LOOP_MS = 20;   // 50 Hz update rate

// ===== SERVO OBJECTS =====
Servo servo1, servo2, servo3, servo4;

// ===== RUNTIME STATE =====
// Current joint angles relative to home (degrees)
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
// HELPER: Convert home-relative angle to servo microseconds
// Derived from: us = home_us + theta / slope
// ============================================================
int angleToPulse(float theta, float slope, int home_us, int minUs, int maxUs) {
  int us = home_us + (int)(theta / slope);
  return constrain(us, minUs, maxUs);
}

// ============================================================
// HELPER: Convert microseconds to home-relative angle
// Inverse of angleToPulse for display purposes
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

  // Configure home reset button with internal pull-up
  pinMode(BTN_HOME, INPUT_PULLUP);

  // Move all joints to home position
  servo1.writeMicroseconds(HOME_US_1);
  servo2.writeMicroseconds(HOME_US_2);
  servo3.writeMicroseconds(HOME_US_3);
  servo4.writeMicroseconds(HOME_US_4);
  delay(1000);  // Allow servos to settle

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

  // Read raw joystick ADC values (0-1023, center ~512)
  int raw1 = analogRead(JOY_BASE);
  int raw2 = analogRead(JOY_SHOULDER);
  int raw3 = analogRead(JOY_ELBOW);
  int raw4 = analogRead(JOY_GRIPPER);

  // Apply deadband: force to center if within deadband range
  if (abs(raw1 - 512) < DEADBAND) raw1 = 512;
  if (abs(raw2 - 512) < DEADBAND) raw2 = 512;
  if (abs(raw3 - 512) < DEADBAND) raw3 = 512;
  if (abs(raw4 - 512) < DEADBAND) raw4 = 512;

  // Compute signed deviation from center (-512 to +512)
  float dev1 = (float)(raw1 - 512);
  float dev2 = (float)(raw2 - 512);
  float dev3 = (float)(raw3 - 512);
  float dev4 = (float)(raw4 - 512);

  // Apply IIR low-pass filter: smooths jerky joystick input
  // filtered_new = (1-α) × filtered_old + α × new_value
  j1_filt = (1.0f - ALPHA) * j1_filt + ALPHA * dev1;
  j2_filt = (1.0f - ALPHA) * j2_filt + ALPHA * dev2;
  j3_filt = (1.0f - ALPHA) * j3_filt + ALPHA * dev3;
  j4_filt = (1.0f - ALPHA) * j4_filt + ALPHA * dev4;

  // Update joint angles using velocity control
  // Joystick position controls rate of change, not absolute position
  th1 += K_BASE     * j1_filt;
  th2 += K_SHOULDER * j2_filt;
  th3 += K_ELBOW    * j3_filt;
  th4 += K_GRIPPER  * j4_filt;

  // Apply ROM limits to prevent self-collision and mechanical damage
  th1 = constrain(th1, ROM_MIN_1, ROM_MAX_1);
  th2 = constrain(th2, ROM_MIN_2, ROM_MAX_2);
  th3 = constrain(th3, ROM_MIN_3, ROM_MAX_3);
  th4 = constrain(th4, ROM_MIN_4, ROM_MAX_4);

  // Check home reset button (active LOW with pull-up)
  if (digitalRead(BTN_HOME) == LOW) {
    // Reset all angles to zero (home position)
    th1 = 0.0f;  th2 = 0.0f;  th3 = 0.0f;  th4 = 0.0f;
    // Clear filter states to prevent overshoot
    j1_filt = 0.0f;  j2_filt = 0.0f;  j3_filt = 0.0f;  j4_filt = 0.0f;
  }

  // Convert angles to microsecond pulses
  int us1 = angleToPulse(th1, SLOPE_1, HOME_US_1, MIN_US_1, MAX_US_1);
  int us2 = angleToPulse(th2, SLOPE_2, HOME_US_2, MIN_US_2, MAX_US_2);
  int us3 = angleToPulse(th3, SLOPE_3, HOME_US_3, MIN_US_3, MAX_US_3);
  int us4 = angleToPulse(th4, SLOPE_4, HOME_US_4, MIN_US_4, MAX_US_4);

  // Send commands to servos
  servo1.writeMicroseconds(us1);
  servo2.writeMicroseconds(us2);
  servo3.writeMicroseconds(us3);
  servo4.writeMicroseconds(us4);

  // Convert constrained microseconds back to angles for accurate display
  float disp1 = pulseToAngle(us1, SLOPE_1, HOME_US_1);
  float disp2 = pulseToAngle(us2, SLOPE_2, HOME_US_2);
  float disp3 = pulseToAngle(us3, SLOPE_3, HOME_US_3);
  float disp4 = pulseToAngle(us4, SLOPE_4, HOME_US_4);

  // Print current state: microseconds and degrees
  Serial.print(us1);   Serial.print("\t");
  Serial.print(us2);   Serial.print("\t");
  Serial.print(us3);   Serial.print("\t");
  Serial.print(us4);   Serial.print("\t");
  Serial.print(disp1, 1); Serial.print("\t");
  Serial.print(disp2, 1); Serial.print("\t");
  Serial.print(disp3, 1); Serial.print("\t");
  Serial.println(disp4, 1);

  // Maintain consistent 20ms loop timing (50 Hz)
  long elapsed = (long)(millis() - t_start);
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}