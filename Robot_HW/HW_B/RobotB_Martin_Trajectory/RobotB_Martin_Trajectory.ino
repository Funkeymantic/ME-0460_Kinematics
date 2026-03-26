// ============================================================
// STUDENT NAME: Paul Martin
// ME4640 Robot Homework B - Part 3
// Programmed Joint Trajectory with Constant Angular Velocity Sweeps
//
// VERIFICATION MODE: Comment out servo.writeMicroseconds() lines
// and run to check Serial output / generate plot before powering servos.
// ============================================================

#include <Servo.h>

// ===== SERVO PIN ASSIGNMENTS (HRIB Shield) =====
const int PIN_BASE     = 3;   // Joint 1 - Base
const int PIN_SHOULDER = 5;   // Joint 2 - Shoulder
const int PIN_ELBOW    = 6;   // Joint 3 - Elbow
const int PIN_GRIPPER  = 9;   // Joint 4 - Gripper

// ===== SERVO PULSE LIMITS (from HW A calibration) =====
const int MIN_US_1 = 579;
const int MAX_US_1 = 2560;
const int MIN_US_2 = 473;
const int MAX_US_2 = 2473;
const int MIN_US_3 = 553;
const int MAX_US_3 = 2500;
const int MIN_US_4 = 700;
const int MAX_US_4 = 2173;

// ===== CALIBRATION COEFFICIENTS (from HW A linear fit: angle = SLOPE*us + INTERCEPT) =====
const float SLOPE_1     = 0.0909f;
const float INTERCEPT_1 = -52.7f;
const float SLOPE_2     = 0.0909f;
const float INTERCEPT_2 = -52.7f;
const float SLOPE_3     = 0.0909f;
const float INTERCEPT_3 = -52.7f;
const float SLOPE_4     = 0.0909f;
const float INTERCEPT_4 = -52.7f;

// ===== HOME POSITION OFFSETS (from Part 2 physical calibration) =====
// UPDATE these values after measuring home position in Part 2!
const int HOME_US_1 = 1570;  // Base     home (µs)  <-- UPDATE
const int HOME_US_2 = 1473;  // Shoulder home (µs)  <-- UPDATE
const int HOME_US_3 = 1533;  // Elbow    home (µs)  <-- UPDATE
const int HOME_US_4 = 1133;  // Gripper  home (µs)  <-- UPDATE

// ===== TRAJECTORY TARGET ANGLES (degrees from home = 0°) =====
// These angles were chosen to produce clear, visible motion.
// Joint 1 - Base sweep targets
const float TH1_A =  45.0f;   // Base sweep:      0° → +45°
const float TH1_HOME = 0.0f;

// Joint 2 - Shoulder targets
const float TH2_A =  40.0f;   // Shoulder sweep:  0° → +40°
const float TH2_HOME = 0.0f;

// Joint 3 - Elbow targets
const float TH3_A = -40.0f;   // Elbow sweep:     0° → -40°
const float TH3_HOME = 0.0f;

// Joint 4 - Gripper targets
const float TH4_OPEN  = 35.0f;  // Gripper open:  0° → +35°
const float TH4_CLOSE = 0.0f;   // Gripper close: back to 0°

// ===== LOOP TIMING =====
const int LOOP_MS = 10;   // 10 ms loop = 100 Hz update rate

// ===== SERVO OBJECTS =====
Servo servo1, servo2, servo3, servo4;

// ===== STATE MACHINE =====
// Trajectory phases:
//  0 → Initial: all joints at home
//  1 → Sweep th1:  0° → +45°  (2 sec)
//  2 → Hold th1 at +45°        (1 sec)
//  3 → Sweep th2:  0° → +40°  (2 sec)
//  4 → Hold th2 at +40°        (1 sec)
//  5 → Sweep th3:  0° → -40°  (2 sec)
//  6 → Hold th3 at -40°        (1 sec)
//  7 → Open  gripper: 0° → +35° (1 sec)
//  8 → Close gripper: +35° → 0° (1 sec)
//  9 → Return th1: +45° → 0°   (2 sec)
// 10 → Done (hold final home position)

int state = 0;
unsigned long phase_start_ms = 0;

// Current commanded angles (deg from home)
float th1 = 0.0f;
float th2 = 0.0f;
float th3 = 0.0f;
float th4 = 0.0f;

// Angle at the beginning of the current sweep phase
float th1_phase_start = 0.0f;
float th2_phase_start = 0.0f;
float th3_phase_start = 0.0f;
float th4_phase_start = 0.0f;

// Elapsed time since program start (for Serial plot)
float t_sec = 0.0f;
unsigned long t_origin_ms = 0;

// ============================================================
// HELPER: angle (deg, home-relative) → microseconds
// us = home_us + theta / slope
// ============================================================
int angleToPulse(float theta, float slope, int home_us, int minUs, int maxUs) {
  int us = home_us + (int)(theta / slope);
  return constrain(us, minUs, maxUs);
}

// ============================================================
// HELPER: Linear interpolation between two angles
// t_frac ∈ [0,1] → constant angular velocity
// ============================================================
float lerp(float a, float b, float t_frac) {
  t_frac = constrain(t_frac, 0.0f, 1.0f);
  return a + (b - a) * t_frac;
}

// ============================================================
// HELPER: Compute and write all four joints, print to Serial
// ============================================================
void commandAndPrint() {
  int us1 = angleToPulse(th1, SLOPE_1, HOME_US_1, MIN_US_1, MAX_US_1);
  int us2 = angleToPulse(th2, SLOPE_2, HOME_US_2, MIN_US_2, MAX_US_2);
  int us3 = angleToPulse(th3, SLOPE_3, HOME_US_3, MIN_US_3, MAX_US_3);
  int us4 = angleToPulse(th4, SLOPE_4, HOME_US_4, MIN_US_4, MAX_US_4);

  // ---- COMMAND SERVOS ----
  // Comment out the four lines below for verification (plot-only) mode:
  servo1.writeMicroseconds(us1);
  servo2.writeMicroseconds(us2);
  servo3.writeMicroseconds(us3);
  servo4.writeMicroseconds(us4);

  // ---- SERIAL OUTPUT (CSV: time, th1, th2, th3, th4) ----
  t_sec = (float)(millis() - t_origin_ms) / 1000.0f;
  Serial.print(t_sec, 3);   Serial.print(",");
  Serial.print(th1, 2);     Serial.print(",");
  Serial.print(th2, 2);     Serial.print(",");
  Serial.print(th3, 2);     Serial.print(",");
  Serial.println(th4, 2);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  // Attach servos with hardware pulse limits
  servo1.attach(PIN_BASE,     MIN_US_1, MAX_US_1);
  servo2.attach(PIN_SHOULDER, MIN_US_2, MAX_US_2);
  servo3.attach(PIN_ELBOW,    MIN_US_3, MAX_US_3);
  servo4.attach(PIN_GRIPPER,  MIN_US_4, MAX_US_4);

  // Move to home
  servo1.writeMicroseconds(HOME_US_1);
  servo2.writeMicroseconds(HOME_US_2);
  servo3.writeMicroseconds(HOME_US_3);
  servo4.writeMicroseconds(HOME_US_4);
  delay(1500);   // Wait for servos to reach home

  // Print CSV header for MATLAB/Python import
  Serial.println("=================================================================");
  Serial.println("ME4640 HW-B  |  Programmed Trajectory  |  Paul Martin");
  Serial.println("=================================================================");
  Serial.println("time_s,th1_deg,th2_deg,th3_deg,th4_deg");

  // Record start times
  t_origin_ms   = millis();
  phase_start_ms = millis();
  state = 1;   // Start immediately at phase 1

  // Capture angle at the beginning of phase 1
  th1_phase_start = th1;
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long loop_start = millis();
  unsigned long phase_elapsed = millis() - phase_start_ms;
  float t_frac = 0.0f;

  switch (state) {

    // -------------------------------------------------------
    // PHASE 1: Sweep th1 from 0° to +45° over 2 seconds
    // -------------------------------------------------------
    case 1: {
      t_frac = (float)phase_elapsed / 2000.0f;
      th1 = lerp(th1_phase_start, TH1_A, t_frac);
      if (phase_elapsed >= 2000) {
        th1 = TH1_A;
        th1_phase_start = th1;
        phase_start_ms = millis();
        state = 2;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 2: Hold th1 at +45° for 1 second
    // -------------------------------------------------------
    case 2: {
      th1 = TH1_A;
      if (phase_elapsed >= 1000) {
        th2_phase_start = th2;
        phase_start_ms = millis();
        state = 3;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 3: Sweep th2 from 0° to +40° over 2 seconds
    // -------------------------------------------------------
    case 3: {
      t_frac = (float)phase_elapsed / 2000.0f;
      th2 = lerp(th2_phase_start, TH2_A, t_frac);
      if (phase_elapsed >= 2000) {
        th2 = TH2_A;
        th2_phase_start = th2;
        phase_start_ms = millis();
        state = 4;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 4: Hold th2 at +40° for 1 second
    // -------------------------------------------------------
    case 4: {
      th2 = TH2_A;
      if (phase_elapsed >= 1000) {
        th3_phase_start = th3;
        phase_start_ms = millis();
        state = 5;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 5: Sweep th3 from 0° to -40° over 2 seconds
    // -------------------------------------------------------
    case 5: {
      t_frac = (float)phase_elapsed / 2000.0f;
      th3 = lerp(th3_phase_start, TH3_A, t_frac);
      if (phase_elapsed >= 2000) {
        th3 = TH3_A;
        th3_phase_start = th3;
        phase_start_ms = millis();
        state = 6;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 6: Hold th3 at -40° for 1 second
    // -------------------------------------------------------
    case 6: {
      th3 = TH3_A;
      if (phase_elapsed >= 1000) {
        th4_phase_start = th4;
        phase_start_ms = millis();
        state = 7;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 7: Open gripper from 0° to +35° over 1 second
    // -------------------------------------------------------
    case 7: {
      t_frac = (float)phase_elapsed / 1000.0f;
      th4 = lerp(th4_phase_start, TH4_OPEN, t_frac);
      if (phase_elapsed >= 1000) {
        th4 = TH4_OPEN;
        th4_phase_start = th4;
        phase_start_ms = millis();
        state = 8;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 8: Close gripper from +35° to 0° over 1 second
    // -------------------------------------------------------
    case 8: {
      t_frac = (float)phase_elapsed / 1000.0f;
      th4 = lerp(th4_phase_start, TH4_CLOSE, t_frac);
      if (phase_elapsed >= 1000) {
        th4 = TH4_CLOSE;
        th1_phase_start = th1;
        phase_start_ms = millis();
        state = 9;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 9: Sweep th1 from +45° back to 0° over 2 seconds
    // -------------------------------------------------------
    case 9: {
      t_frac = (float)phase_elapsed / 2000.0f;
      th1 = lerp(th1_phase_start, TH1_HOME, t_frac);
      if (phase_elapsed >= 2000) {
        th1 = TH1_HOME;
        phase_start_ms = millis();
        state = 10;
      }
      break;
    }

    // -------------------------------------------------------
    // PHASE 10: Done — hold home position
    // -------------------------------------------------------
    case 10: {
      th1 = TH1_HOME;
      th2 = TH2_HOME;
      th3 = TH3_HOME;
      th4 = TH4_CLOSE;
      // Print final position once, then stop
      if (phase_elapsed < LOOP_MS + 5) {
        commandAndPrint();
        Serial.println("# TRAJECTORY COMPLETE");
      }
      // Loop forever holding home — no more Serial output
      while (true) {
        servo1.writeMicroseconds(HOME_US_1);
        servo2.writeMicroseconds(HOME_US_2);
        servo3.writeMicroseconds(HOME_US_3);
        servo4.writeMicroseconds(HOME_US_4);
        delay(500);
      }
    }

  }  // end switch

  // Command servos and print every loop iteration for all active phases
  if (state != 10) {
    commandAndPrint();
  }

  // Maintain loop timing
  long elapsed = (long)(millis() - loop_start);
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}
