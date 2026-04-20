// ============================================================
// STUDENT NAME: Paul Martin (PCM)
// ME4640 Robot Homework C - Part 1g / Part 2


#include <Servo.h>

// ===== SERVO PINS =====
const int PIN_BASE     = 3;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW    = 6;
const int PIN_GRIPPER  = 9;

// ===== BUTTON PINS =====
const int BTN_HOME = 8;
const int BTN_NEXT = 12;

// ===== SERVO PULSE LIMITS =====
const int MIN_US_1 = 579;    const int MAX_US_1 = 2560;
const int MIN_US_2 = 473;    const int MAX_US_2 = 2473;
const int MIN_US_3 = 553;    const int MAX_US_3 = 2500;
const int MIN_US_4 = 1600;   const int MAX_US_4 = 2400;

// ===== CALIBRATION (from HW A) =====
const float SLOPE_1 = 0.097f;   const float INTERCEPT_1 = -48.44f;
const float SLOPE_2 = 0.0909f;  const float INTERCEPT_2 = -52.7f;
const float SLOPE_3 = 0.0909f;  const float INTERCEPT_3 = -52.7f;
const float SLOPE_4 = 0.0909f;  const float INTERCEPT_4 = -52.7f;

// ===== HOME PULSE WIDTHS =====
const int HOME_US_1 = 1523;
const int HOME_US_2 = 1473;
const int HOME_US_3 = 1061;
const int HOME_US_4 = 2000;

// ===== ROM LIMITS (deg from home) =====
const float ROM_MIN_1 = -80.0f;  const float ROM_MAX_1 =  80.0f;
const float ROM_MIN_2 = -30.0f;  const float ROM_MAX_2 =  60.0f;
const float ROM_MIN_3 = -60.0f;  const float ROM_MAX_3 =  60.0f;
const float ROM_MIN_4 =   0.0f;  const float ROM_MAX_4 =  50.0f;

// ===== LINK DIMENSIONS (mm) =====
const float h_B    = 55.0f;
const float L_BC   = 75.0f;
const float L_CD   = 80.0f;
const float L_GRIP = 62.0f;
const float X_BASE = 150.0f;

// ===== MOVEMENT RATE =====
const int   LOOP_MS       = 20;
const float RATE_DEG_S    = 40.0f;
const float RATE_PER_LOOP = RATE_DEG_S * (LOOP_MS / 1000.0f);

// ===== SET POINTS [th1, th2, th3, th4] (degrees) =====
//  B uses th1=+45 (not -45) so x decreases A->B
const float SET_POINTS[][4] = {
  {   0.0f,  0.0f,  0.0f,  0.0f },   // HOME
  {  30.0f, 30.0f,  0.0f,  0.0f },   // A
  {  45.0f, 45.0f, 10.0f,  0.0f },   // B
  { -30.0f, 20.0f, 20.0f,  0.0f },   // C
};
const int NUM_POINTS = 4;
const char* POINT_NAMES[] = { "HOME", "A", "B", "C" };

Servo servo1, servo2, servo3, servo4;

float th1 = 0.0f, th2 = 0.0f, th3 = 0.0f, th4 = 0.0f;

int  target_idx = 0;
int  next_idx   = 1;
bool at_target  = false;

unsigned long last_home_press = 0;
unsigned long last_next_press = 0;
const unsigned long DEBOUNCE_MS = 250;

// ============================================================
int angleToPulse(float theta, float slope, int home_us, int minUs, int maxUs) {
  return constrain(home_us + (int)(theta / slope), minUs, maxUs);
}

float pulseToAngle(int us, float slope, int home_us) {
  return slope * (us - home_us);
}

float moveToward(float current, float target, float max_step) {
  float diff = target - current;
  if (abs(diff) <= max_step) return target;
  return current + (diff > 0 ? max_step : -max_step);
}

// ============================================================
//  FORWARD KINEMATICS
// ============================================================
void computeFK(float t1, float t2, float t3,
               float &x_mm, float &y_mm, float &z_mm) {
  float r1 = t1 * (PI / 180.0f);
  float r2 = t2 * (PI / 180.0f);
  float r3 = t3 * (PI / 180.0f);

  float r  = -L_BC * sin(r2) + L_CD * cos(r2 - r3) + L_GRIP;
  x_mm = X_BASE - r * sin(r1);
  y_mm =          r * cos(r1);
  z_mm = h_B + L_BC * cos(r2) + L_CD * sin(r2 - r3);
}

// ============================================================
void printState(const char* label, float d1, float d2, float d3) {
  float x_mm, y_mm, z_mm;
  computeFK(d1, d2, d3, x_mm, y_mm, z_mm);
  Serial.print(label);
  Serial.print("\tth1: "); Serial.print(d1, 1);
  Serial.print("\tth2: "); Serial.print(d2, 1);
  Serial.print("\tth3: "); Serial.print(d3, 1);
  Serial.print("\tx: ");   Serial.print(x_mm, 2);
  Serial.print("\ty: ");   Serial.print(y_mm, 2);
  Serial.print("\tz: ");   Serial.println(z_mm, 2);
}

// ============================================================
void setup() {
  Serial.begin(230400);

  servo1.attach(PIN_BASE,     MIN_US_1, MAX_US_1);
  servo2.attach(PIN_SHOULDER, MIN_US_2, MAX_US_2);
  servo3.attach(PIN_ELBOW,    MIN_US_3, MAX_US_3);
  servo4.attach(PIN_GRIPPER,  MIN_US_4, MAX_US_4);

  pinMode(BTN_HOME, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);

  servo1.writeMicroseconds(HOME_US_1);
  servo2.writeMicroseconds(HOME_US_2);
  servo3.writeMicroseconds(HOME_US_3);
  servo4.writeMicroseconds(HOME_US_4);
  delay(1000);

  Serial.println("ME4640 HW-C | Paul Martin | BTN_HOME=pin8  BTN_NEXT=pin12");
  Serial.println("point\tth1\tth2\tth3\tx_mm\ty_mm\tz_mm");
  Serial.println("---");

  // Print home FK on startup
  printState("HOME", 0, 0, 0);
}

// ============================================================
void loop() {
  unsigned long t_start = millis();

  bool home_pressed = (digitalRead(BTN_HOME) == LOW) &&
                      (millis() - last_home_press > DEBOUNCE_MS);
  bool next_pressed = (digitalRead(BTN_NEXT) == LOW) &&
                      (millis() - last_next_press > DEBOUNCE_MS);

  // HOME button: snap to home target, reset queue
  if (home_pressed) {
    last_home_press = millis();
    target_idx = 0;
    next_idx   = 1;
    at_target  = false;
    Serial.println("# HOME pressed -- moving to HOME, queue reset to A");
  }

  // NEXT button: advance to next set point
  if (next_pressed) {
    last_next_press = millis();
    if (next_idx < NUM_POINTS) {
      target_idx = next_idx++;
      at_target  = false;
      Serial.print("# NEXT pressed -- moving to point ");
      Serial.println(POINT_NAMES[target_idx]);
    } else {
      Serial.println("# Already at final point C. Press HOME to reset.");
    }
  }

  // Move toward target
  float t1_tgt = SET_POINTS[target_idx][0];
  float t2_tgt = SET_POINTS[target_idx][1];
  float t3_tgt = SET_POINTS[target_idx][2];
  float t4_tgt = SET_POINTS[target_idx][3];

  th1 = moveToward(th1, t1_tgt, RATE_PER_LOOP);
  th2 = moveToward(th2, t2_tgt, RATE_PER_LOOP);
  th3 = moveToward(th3, t3_tgt, RATE_PER_LOOP);
  th4 = moveToward(th4, t4_tgt, RATE_PER_LOOP);

  // Write servos
  int us1 = angleToPulse(th1, SLOPE_1, HOME_US_1, MIN_US_1, MAX_US_1);
  int us2 = angleToPulse(th2, SLOPE_2, HOME_US_2, MIN_US_2, MAX_US_2);
  int us3 = angleToPulse(th3, SLOPE_3, HOME_US_3, MIN_US_3, MAX_US_3);
  int us4 = angleToPulse(th4, SLOPE_4, HOME_US_4, MIN_US_4, MAX_US_4);

  servo1.writeMicroseconds(us1);
  servo2.writeMicroseconds(us2);
  servo3.writeMicroseconds(us3);
  servo4.writeMicroseconds(us4);
  

  // Check arrival -- PRINT ONLY HERE (not every loop)
  bool now_at = (abs(th1-t1_tgt)<0.5f) && (abs(th2-t2_tgt)<0.5f) &&
                (abs(th3-t3_tgt)<0.5f) && (abs(th4-t4_tgt)<0.5f);

  if (now_at && !at_target) {
    at_target = true;
    float d1 = pulseToAngle(us1, SLOPE_1, HOME_US_1);
    float d2 = pulseToAngle(us2, SLOPE_2, HOME_US_2);
    float d3 = pulseToAngle(us3, SLOPE_3, HOME_US_3);
    printState(POINT_NAMES[target_idx], d1, d2, d3);
  }

  long elapsed = (long)(millis() - t_start);
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}
