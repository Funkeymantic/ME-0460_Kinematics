// ============================================================
//  ME 4640 – Robotics Engineering
//  HW D – Part 4: IK Trajectory Execution
//
//  Author: Paul Martin
//  Date  : 04/19/26
//
//  Description:
//    Executes an IK-generated joint trajectory on the ME Arm V3.0.
//    The trajectory is loaded from  trajectory.h, which is produced
//    by running  ME_Arm_IK_Export.m  (or ME_Arm_IK_Export.py) on your
//    Part 2 Cartesian path.
//
//    Copy  trajectory.h  into this sketch folder before compiling.
//    The header stores motor angles [th1, th2, th3] in degrees,
//    home-relative, in AVR program memory (PROGMEM) to conserve SRAM.
//
//  Motor-angle mapping (from Robot_HWC_Soln.m FK derivation):
//    th1 =  -th_a          (base)
//    th2 =  th_b          (shoulder)
//    th3 = -(th_c + th_b) (elbow)
//    th4  –  gripper, not set by IK; controlled separately below.
//
//  Hardware pin mapping (HRIB shield):
//    Joint      Servo   Signal Pin    Index in arrays
//    ---------  ------  ----------    ---------------
//    Base        S1      D3            0
//    Shoulder    S2      D5            1
//    Elbow       S3      D6            2
//    Gripper     S4      D9            3
//
//  WORKFLOW:
//    1. Run ME_Arm_IK_Export.m in MATLAB (or the Python equivalent).
//    2. Copy the generated  trajectory.h  into this folder.
//    3. Set SERVO_ENABLED = false, upload, open Serial Monitor (230400 baud).
//    4. Copy the output into MATLAB/Python, plot theta vs. waypoint,
//       and confirm the path looks correct.
//    5. Set SERVO_ENABLED = true, upload, and run on hardware.
// ============================================================

#include <Servo.h>
#include <avr/pgmspace.h>    // PROGMEM support (AVR boards: Uno, Nano, Mega)
#include "trajectory.h"      // defines N_WPT and TRAJ[N_WPT][3]

// ============================================================
//  VERIFICATION FLAG  –  set false to suppress servo output
// ============================================================
const bool SERVO_ENABLED = true;   // false = verification mode; true = servos active

// ============================================================
//  Calibration Coefficients  –  UPDATE FROM YOUR HW A RESULTS
// ============================================================
const float m[4] = { 10.5,   10.5,   10.5,   10.5  };     // slope (us/deg)
const float b[4] = { 1500.0, 1500.0, 1500.0, 1500.0 };    // intercept (us)

// ============================================================
//  Range of motion limits  –  adjust to protect your robot
// ============================================================
const int US_MIN[4] = { 1000, 1050, 1000, 1000 };
const int US_MAX[4] = { 2400, 2390, 2250, 2110 };
float TH_MIN[4] = { 0, 0, 0, 0 };   // computed in setup()
float TH_MAX[4] = { 0, 0, 0, 0 };   // computed in setup()

// ============================================================
//  Home position offsets  –  UPDATE FROM HW A / HW B
//    th_off[i] = angle (deg) from calibration zero to physical home.
// ============================================================
const float th_off[4] = { 0, 0, 10.02,  54.14 };

// ============================================================
//  Gripper angle during trajectory  (deg, home-relative)
//    IK does not solve for the gripper; set it here manually.
//    Positive values open the gripper; see your HW A calibration.
// ============================================================
const float GRIPPER_ANGLE = 0.0;    // 0 = closed (home); adjust as needed

// ============================================================
//  Trajectory timing
//    DT_MS controls how long the robot dwells at each waypoint.
//    Smaller → faster motion.  Recommended range: 20–100 ms.
//    At 20 ms (50 Hz) a 200-waypoint trajectory takes ~4 seconds.
// ============================================================
const unsigned long DT_MS = 40;     // ms per waypoint  (try 20–100)

// ============================================================
//  Serial print rate
//    Prints every PRINT_EVERY_N waypoints to keep serial output
//    manageable.  Set to 1 to print every waypoint.
// ============================================================
const int PRINT_EVERY_N = 5;

// ============================================================
//  Pin assignments
// ============================================================
const int SERVO_PINS[4] = { 3, 6, 5, 9 };   // D3=Base, D6=Shoulder, D5=Elbow, D9=Gripper

// ============================================================
//  Global State
// ============================================================
Servo servo[4];
float TH_FK[4] = { 0, 0, 0, 0 };   // current joint angles (deg, home-relative)
int   us[4];                        // current servo commands (microseconds)

// ============================================================
//  angleToUs: angle (deg, calibration space) → microseconds
// ============================================================
int angleToUs(int i, float theta) {
  float us_cal = m[i] * theta + b[i];
  return (int)constrain(us_cal, US_MIN[i], US_MAX[i]);
}

// ============================================================
//  usToAngle: microseconds → angle (deg, calibration space)
// ============================================================
float usToAngle(int i, float us_val) {
  return (us_val - b[i]) / m[i];
}

// ============================================================
//  writeAllServos: send us[] to hardware (suppressed in verification mode)
// ============================================================
void writeAllServos() {
  if (SERVO_ENABLED) {
    for (int i = 0; i < 4; i++) servo[i].writeMicroseconds(us[i]);
  }
}

// ============================================================
//  updateAndWrite: TH_FK[] → TH_con → us[], then write servos
//    TH_FK holds home-relative angles.
//    TH_con = TH_FK + th_off shifts to calibration space.
// ============================================================
void updateAndWrite() {
  for (int i = 0; i < 4; i++) {
    float th_con = TH_FK[i] + th_off[i];
    us[i] = angleToUs(i, th_con);
  }
  writeAllServos();
}

// ============================================================
//  printWaypoint: print one row of data to Serial Monitor
//    Format:  wpt_idx  t_ms  th1  th2  th3  th4
//    (copy-paste into MATLAB/Python to verify trajectory shape)
// ============================================================
void printWaypoint(int idx) {
  static int count = 0;
  if (++count < PRINT_EVERY_N) return;
  count = 0;

  Serial.print(idx);          Serial.print("\t");
  Serial.print(millis());     Serial.print("\t");
  Serial.print(TH_FK[0], 3); Serial.print("\t");
  Serial.print(TH_FK[1], 3); Serial.print("\t");
  Serial.print(TH_FK[2], 3); Serial.print("\t");
  Serial.println(TH_FK[3], 3);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(230400);

  // Compute joint angle limits (home-relative) from microsecond limits
  for (int i = 0; i < 4; i++) {
    TH_MIN[i] = usToAngle(i, US_MIN[i]) - th_off[i];
    TH_MAX[i] = usToAngle(i, US_MAX[i]) - th_off[i];
  }

  if (SERVO_ENABLED) {
    for (int i = 0; i < 4; i++) {
      servo[i].attach(SERVO_PINS[i], US_MIN[i], US_MAX[i]);
      us[i] = angleToUs(i, th_off[i]);         // TH_FK=0 → home
      servo[i].writeMicroseconds(us[i]);        // lock to home immediately
    }
    Serial.println(F("# SERVO_ENABLED = true  –  servos are ACTIVE"));
  } else {
    Serial.println(F("# SERVO_ENABLED = false –  VERIFICATION MODE (no servo output)"));
  }

  // Print trajectory metadata
  Serial.print(F("# Waypoints: ")); Serial.println(N_WPT);
  Serial.print(F("# DT_MS    : ")); Serial.println(DT_MS);
  Serial.print(F("# Est. duration (s): "));
  Serial.println((float)N_WPT * DT_MS / 1000.0, 1);

  // Column headers for Serial Monitor / MATLAB / Python
  Serial.println(F("wpt\tt_ms\tth1\tth2\tth3\tth4"));

  delay(500);
}

// ============================================================
//  rampToTarget: linearly interpolate from current TH_FK[] to
//  a target pose over RAMP_STEPS steps, then hold for HOLD_MS.
//  This prevents abrupt jumps when entering/leaving the trajectory.
// ============================================================
const int   RAMP_STEPS = 30;    // number of interpolation steps
const int   RAMP_DT_MS = 20;    // ms per ramp step  (~600 ms total)

void rampToTarget(float t0, float t1, float t2, float t3) {
  float start[4] = { TH_FK[0], TH_FK[1], TH_FK[2], TH_FK[3] };
  float goal[4]  = { t0, t1, t2, t3 };

  for (int s = 1; s <= RAMP_STEPS; s++) {
    float alpha = (float)s / RAMP_STEPS;
    for (int i = 0; i < 4; i++) {
      TH_FK[i] = start[i] + alpha * (goal[i] - start[i]);
    }
    updateAndWrite();
    delay(RAMP_DT_MS);
  }
}

// ============================================================
//  MAIN TRAJECTORY LOOP  (runs once; reset Arduino to repeat)
// ============================================================
void loop() {

  // ---- Move to home before starting ----
  for (int i = 0; i < 4; i++) TH_FK[i] = 0.0;
  updateAndWrite();
  Serial.println(F("# At home. Ramping to trajectory start in 1 s..."));
  delay(1000);

  // ---- Read first waypoint and ramp smoothly to it ----
  //  Without this, the arm jumps from home (0,0,0) directly to the
  //  first IK waypoint in a single DT_MS step.
  float first_th1 = pgm_read_float_near(&TRAJ[0][0]);
  float first_th2 = pgm_read_float_near(&TRAJ[0][1]);
  float first_th3 = pgm_read_float_near(&TRAJ[0][2]);
  rampToTarget(
    constrain(first_th1, TH_MIN[0], TH_MAX[0]),
    constrain(first_th2, TH_MIN[1], TH_MAX[1]),
    constrain(first_th3, TH_MIN[2], TH_MAX[2]),
    constrain(GRIPPER_ANGLE, TH_MIN[3], TH_MAX[3])
  );
  Serial.println(F("# Ramp complete. Starting IK trajectory..."));

  // ---- Step through each IK waypoint ----
  //
  //  Each row of TRAJ holds { th1, th2, th3 } in home-relative degrees.
  //  PROGMEM stores the array in flash; pgm_read_float_near() reads it back.
  //  The gripper (index 3) is not solved by IK – it stays at GRIPPER_ANGLE.
  //
  for (int n = 0; n < N_WPT; n++) {

    // Read motor angles from PROGMEM
    float th1 = pgm_read_float_near(&TRAJ[n][0]);   // base
    float th2 = pgm_read_float_near(&TRAJ[n][1]);   // shoulder
    float th3 = pgm_read_float_near(&TRAJ[n][2]);   // elbow

    // Apply joint limits and load into TH_FK
    TH_FK[0] = constrain(th1, TH_MIN[0], TH_MAX[0]);
    TH_FK[1] = constrain(th2, TH_MIN[1], TH_MAX[1]);
    TH_FK[2] = constrain(th3, TH_MIN[2], TH_MAX[2]);
    TH_FK[3] = constrain(GRIPPER_ANGLE, TH_MIN[3], TH_MAX[3]);

    updateAndWrite();
    printWaypoint(n);
    delay(DT_MS);
  }

  // ---- Ramp smoothly back to home ----
  //  Without this, the arm snaps from the last IK pose to (0,0,0) instantly.
  Serial.println(F("# Trajectory complete. Ramping back to home."));
  rampToTarget(0.0, 0.0, 0.0, 0.0);
  delay(500);

  Serial.println(F("# Done. Reset Arduino to run again."));

  // Halt at home
  while (true) {
    writeAllServos();
    delay(1000);
  }
}
