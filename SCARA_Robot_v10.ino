#include <AccelStepper.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "webpage.h"

// =====================================================
// Pin Definitions
// =====================================================

#define A1_STEP   17
#define A1_DIR    16
#define A1_EN     5
#define A1_LIMIT  14

#define A2_STEP   19
#define A2_DIR    18
#define A2_EN     21
#define A2_LIMIT  27

#define Z_STEP    2
#define Z_DIR     15
#define Z_EN      4
#define Z_LIMIT   26

// =====================================================
// Robot Geometry
// =====================================================

constexpr float L1 = 136.5f;   // mm: Joint 1 center to Joint 2 center
constexpr float L2 = 95.5f;    // mm: Joint 2 center to tool point

// =====================================================
// Joint Limits Used by UI, FK, IK
// =====================================================

constexpr float A1_MIN_DEG = -180.0f;
constexpr float A1_MAX_DEG =  180.0f;

constexpr float A2_MIN_DEG = -160.0f;
constexpr float A2_MAX_DEG =  160.0f;

constexpr float Z_MIN_MM = -135.0f;
constexpr float Z_MAX_MM =  135.0f;

// =====================================================
// Calibration Model
// =====================================================
// This follows the proven style from the reference SCARA code:
// when the switch is touched, set the motor position to the known
// calibrated joint position, then move to user zero.
//
// Do NOT endpoint-calibrate with many "Set A1 +180" buttons anymore.
// Tune these values from the web Setup tab instead.

float A1_STEPS_PER_DEG = 142.22f;   // 200 * 16 * 16 / 360
float A2_STEPS_PER_DEG = 40.0f;     // 200 * 16 * 4.5 / 360
float Z_STEPS_PER_MM   = 400.0f;    // 200 * 16 / 8 mm

// The physical angle when each axis touches its home switch.
// Your current machine: A1 switch side = negative side.
// A2 switch side = negative side.
// Z switch side = top.
// The switch is NOT a usable endpoint. It must be a little outside the user range.
// That means user command A1=-180 and A2=-160 stop before touching the limit switch.
float A1_HOME_DEG = -195.0f;
float A2_HOME_DEG = -185.0f;
float Z_HOME_MM   = -137.5f;

// Step direction from home toward increasing user coordinate.
// A1: from -180 to +180 uses positive machine steps.
// A2: from -160 to +160 uses negative machine steps on your machine.
// Z:  from top to down uses positive machine steps.
constexpr int A1_STEP_SIGN =  1;
constexpr int A2_STEP_SIGN = -1;
constexpr int Z_STEP_SIGN  =  1;

// Fine trims. Use these only after steps/degree is correct.
float A1_TRIM_DEG = 0.0f;
float A2_TRIM_DEG = 0.0f;
float Z_TRIM_MM   = 0.0f;

// =====================================================
// WiFi AP
// =====================================================

const char* AP_SSID = "SCARA-Robot";
const char* AP_PASS = "robot1234";

WebServer server(80);
Preferences prefs;

// =====================================================
// Stepper Objects
// =====================================================

AccelStepper arm1(AccelStepper::DRIVER, A1_STEP, A1_DIR);
AccelStepper arm2(AccelStepper::DRIVER, A2_STEP, A2_DIR);
AccelStepper zAxis(AccelStepper::DRIVER, Z_STEP, Z_DIR);

// =====================================================
// Motion Settings
// =====================================================

float ARM_MAX_SPEED = 5600.0f;
float ARM_ACCEL     = 2200.0f;

float Z_MAX_SPEED   = 9000.0f;
float Z_ACCEL       = 3500.0f;

const int LIMIT_PRESSED = LOW;

const int A1_HOME_DIR = -1;   // tested: A1 homes toward switch with negative target
const int A2_HOME_DIR =  1;   // tested: A2 homes toward switch with positive target
const int Z_HOME_DIR  = -1;   // tested: Z homes upward/top with negative target

const long A1_HOME_MAX_STEPS = 90000;
const long A2_HOME_MAX_STEPS = 40000;
const long Z_HOME_MAX_STEPS  = 110000;

const long A1_BACKOFF_STEPS = 500;
const long A2_BACKOFF_STEPS = 500;
const long Z_BACKOFF_STEPS  = 1000;

// Raw-step safety clearance from the home switch.
// A1 switch is at raw step 0 and safe work is positive.
// A2 switch is at raw step 0 and safe work is negative.
// Z top switch is at raw step 0 and safe work is positive/down.
const long A1_SWITCH_CLEARANCE_STEPS = 2000;
const long A2_SWITCH_CLEARANCE_STEPS = 1000;
const long Z_SWITCH_CLEARANCE_STEPS  = 1000;

const unsigned long HOME_TIMEOUT_MS = 60000;
const unsigned long IDLE_DISABLE_DELAY_MS = 3000;

const bool KEEP_ARMS_ENABLED_WHEN_IDLE = true;
const bool KEEP_Z_ENABLED_WHEN_IDLE    = false;

// =====================================================
// State
// =====================================================

bool isHomed = false;
bool isHoming = false;
bool motorsEnabled = false;
bool stopRequested = false;

unsigned long lastMoveTime = 0;

// =====================================================
// Data Structures
// =====================================================

struct Position {
  float x;
  float y;
  float z;
};

struct IKCandidate {
  bool valid;
  float a1;
  float a2;
  float score;
};

struct IKResult {
  bool ok;
  float a1;
  float a2;
  String error;
};

// =====================================================
// Motor Enable / Disable
// =====================================================

void enableMotors() {
  digitalWrite(A1_EN, LOW);
  digitalWrite(A2_EN, LOW);
  digitalWrite(Z_EN, LOW);
  motorsEnabled = true;
  lastMoveTime = millis();
}

void forceMotorsOff() {
  digitalWrite(A1_EN, HIGH);
  digitalWrite(A2_EN, HIGH);
  digitalWrite(Z_EN, HIGH);
  motorsEnabled = false;
}

void applyIdleMotorPolicy() {
  if (!isHomed) {
    forceMotorsOff();
    return;
  }

  digitalWrite(A1_EN, KEEP_ARMS_ENABLED_WHEN_IDLE ? LOW : HIGH);
  digitalWrite(A2_EN, KEEP_ARMS_ENABLED_WHEN_IDLE ? LOW : HIGH);
  digitalWrite(Z_EN,  KEEP_Z_ENABLED_WHEN_IDLE    ? LOW : HIGH);

  motorsEnabled = KEEP_ARMS_ENABLED_WHEN_IDLE || KEEP_Z_ENABLED_WHEN_IDLE;
}

void haltAllMotion() {
  arm1.moveTo(arm1.currentPosition());
  arm2.moveTo(arm2.currentPosition());
  zAxis.moveTo(zAxis.currentPosition());
}

// =====================================================
// Calibration Storage
// =====================================================

void loadCalibration() {
  prefs.begin("scara_v10", false);

  A1_STEPS_PER_DEG = prefs.getFloat("a1spd", 142.22f);
  A2_STEPS_PER_DEG = prefs.getFloat("a2spd", 40.0f);
  Z_STEPS_PER_MM   = prefs.getFloat("zspmm", 400.0f);

  A1_HOME_DEG = prefs.getFloat("a1home", -195.0f);
  A2_HOME_DEG = prefs.getFloat("a2home", -185.0f);
  Z_HOME_MM   = prefs.getFloat("zhome", -137.5f);

  A1_TRIM_DEG = prefs.getFloat("a1trim", 0.0f);
  A2_TRIM_DEG = prefs.getFloat("a2trim", 0.0f);
  Z_TRIM_MM   = prefs.getFloat("ztrim", 0.0f);

  ARM_MAX_SPEED = prefs.getFloat("armspeed", 5600.0f);
  ARM_ACCEL     = prefs.getFloat("armaccel", 2200.0f);
}

void applySpeedSettings() {
  arm1.setMaxSpeed(ARM_MAX_SPEED);
  arm1.setAcceleration(ARM_ACCEL);

  arm2.setMaxSpeed(ARM_MAX_SPEED);
  arm2.setAcceleration(ARM_ACCEL);

  zAxis.setMaxSpeed(Z_MAX_SPEED);
  zAxis.setAcceleration(Z_ACCEL);
}

void saveCalibration() {
  prefs.putFloat("a1spd", A1_STEPS_PER_DEG);
  prefs.putFloat("a2spd", A2_STEPS_PER_DEG);
  prefs.putFloat("zspmm", Z_STEPS_PER_MM);

  prefs.putFloat("a1home", A1_HOME_DEG);
  prefs.putFloat("a2home", A2_HOME_DEG);
  prefs.putFloat("zhome", Z_HOME_MM);

  prefs.putFloat("a1trim", A1_TRIM_DEG);
  prefs.putFloat("a2trim", A2_TRIM_DEG);
  prefs.putFloat("ztrim", Z_TRIM_MM);

  prefs.putFloat("armspeed", ARM_MAX_SPEED);
  prefs.putFloat("armaccel", ARM_ACCEL);

  Serial.println("Calibration saved.");
}

void clearCalibration() {
  A1_STEPS_PER_DEG = 142.22f;
  A2_STEPS_PER_DEG = 40.0f;
  Z_STEPS_PER_MM   = 400.0f;

  A1_HOME_DEG = -195.0f;
  A2_HOME_DEG = -185.0f;
  Z_HOME_MM   = -137.5f;

  A1_TRIM_DEG = 0.0f;
  A2_TRIM_DEG = 0.0f;
  Z_TRIM_MM   = 0.0f;

  ARM_MAX_SPEED = 5600.0f;
  ARM_ACCEL     = 2200.0f;

  applySpeedSettings();
  saveCalibration();

  isHomed = false;
  Serial.println("Calibration cleared. Re-home required.");
}

String calibrationJson() {
  String j = "{";
  j += "\"a1spd\":" + String(A1_STEPS_PER_DEG, 6);
  j += ",\"a2spd\":" + String(A2_STEPS_PER_DEG, 6);
  j += ",\"zspmm\":" + String(Z_STEPS_PER_MM, 6);
  j += ",\"a1home\":" + String(A1_HOME_DEG, 6);
  j += ",\"a2home\":" + String(A2_HOME_DEG, 6);
  j += ",\"zhome\":" + String(Z_HOME_MM, 6);
  j += ",\"a1trim\":" + String(A1_TRIM_DEG, 6);
  j += ",\"a2trim\":" + String(A2_TRIM_DEG, 6);
  j += ",\"ztrim\":" + String(Z_TRIM_MM, 6);
  j += ",\"armspeed\":" + String(ARM_MAX_SPEED, 1);
  j += ",\"armaccel\":" + String(ARM_ACCEL, 1);
  j += "}";
  return j;
}

void printCalibration() {
  Serial.println("Calibration:");
  Serial.print("A1_STEPS_PER_DEG = "); Serial.println(A1_STEPS_PER_DEG, 6);
  Serial.print("A2_STEPS_PER_DEG = "); Serial.println(A2_STEPS_PER_DEG, 6);
  Serial.print("Z_STEPS_PER_MM   = "); Serial.println(Z_STEPS_PER_MM, 6);
  Serial.print("A1_HOME_DEG      = "); Serial.println(A1_HOME_DEG, 6);
  Serial.print("A2_HOME_DEG      = "); Serial.println(A2_HOME_DEG, 6);
  Serial.print("Z_HOME_MM        = "); Serial.println(Z_HOME_MM, 6);
  Serial.print("A1_TRIM_DEG      = "); Serial.println(A1_TRIM_DEG, 6);
  Serial.print("A2_TRIM_DEG      = "); Serial.println(A2_TRIM_DEG, 6);
  Serial.print("Z_TRIM_MM        = "); Serial.println(Z_TRIM_MM, 6);
  Serial.print("ARM_MAX_SPEED    = "); Serial.println(ARM_MAX_SPEED, 1);
  Serial.print("ARM_ACCEL        = "); Serial.println(ARM_ACCEL, 1);
}

// =====================================================
// Step / Angle Conversion
// =====================================================

long a1ToSteps(float userDeg) {
  userDeg = constrain(userDeg + A1_TRIM_DEG, A1_MIN_DEG, A1_MAX_DEG);
  return lround(A1_STEP_SIGN * (userDeg - A1_HOME_DEG) * A1_STEPS_PER_DEG);
}

long a2ToSteps(float userDeg) {
  userDeg = constrain(userDeg + A2_TRIM_DEG, A2_MIN_DEG, A2_MAX_DEG);
  return lround(A2_STEP_SIGN * (userDeg - A2_HOME_DEG) * A2_STEPS_PER_DEG);
}

long zToSteps(float userMM) {
  userMM = constrain(userMM + Z_TRIM_MM, Z_MIN_MM, Z_MAX_MM);
  return lround(Z_STEP_SIGN * (userMM - Z_HOME_MM) * Z_STEPS_PER_MM);
}

long clampA1SafeSteps(long targetSteps) {
  // A1 switch is at raw step 0. Safe working area is positive.
  if (targetSteps < A1_SWITCH_CLEARANCE_STEPS) {
    return A1_SWITCH_CLEARANCE_STEPS;
  }
  return targetSteps;
}

long clampA2SafeSteps(long targetSteps) {
  // A2 switch is at raw step 0. Safe working area is negative.
  if (targetSteps > -A2_SWITCH_CLEARANCE_STEPS) {
    return -A2_SWITCH_CLEARANCE_STEPS;
  }
  return targetSteps;
}

long clampZSafeSteps(long targetSteps) {
  // Z top switch is at raw step 0. Downward working area is positive.
  if (targetSteps < Z_SWITCH_CLEARANCE_STEPS) {
    return Z_SWITCH_CLEARANCE_STEPS;
  }
  return targetSteps;
}

float stepsToA1(long steps) {
  return ((float)steps / (A1_STEP_SIGN * A1_STEPS_PER_DEG)) + A1_HOME_DEG - A1_TRIM_DEG;
}

float stepsToA2(long steps) {
  return ((float)steps / (A2_STEP_SIGN * A2_STEPS_PER_DEG)) + A2_HOME_DEG - A2_TRIM_DEG;
}

float stepsToZ(long steps) {
  return ((float)steps / (Z_STEP_SIGN * Z_STEPS_PER_MM)) + Z_HOME_MM - Z_TRIM_MM;
}

float getA1() {
  return stepsToA1(arm1.currentPosition());
}

float getA2() {
  return stepsToA2(arm2.currentPosition());
}

float getZ() {
  return stepsToZ(zAxis.currentPosition());
}

// =====================================================
// Kinematics
// =====================================================

Position forwardKinematics(float a1Deg, float a2Deg, float zMM) {
  float a1 = radians(a1Deg);
  float a2 = radians(a2Deg);

  Position p;
  p.x = L1 * cos(a1) + L2 * cos(a1 + a2);
  p.y = L1 * sin(a1) + L2 * sin(a1 + a2);
  p.z = zMM;
  return p;
}

float normalizeAngle(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

float angleDistance(float a, float b) {
  float d = fabs(normalizeAngle(a - b));
  return d;
}

IKCandidate solveIKBranch(float x, float y, bool positiveA2, float refA1, float refA2) {
  IKCandidate c;
  c.valid = false;
  c.a1 = 0.0f;
  c.a2 = 0.0f;
  c.score = 999999.0f;

  float d2 = x * x + y * y;
  float r = sqrt(d2);

  if (r > (L1 + L2) || r < fabs(L1 - L2)) return c;

  float cosA2 = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (cosA2 < -1.0f || cosA2 > 1.0f) return c;

  cosA2 = constrain(cosA2, -1.0f, 1.0f);

  float sinA2 = sqrt(1.0f - cosA2 * cosA2);
  if (!positiveA2) sinA2 = -sinA2;

  float a2 = atan2(sinA2, cosA2);
  float a1 = atan2(y, x) - atan2(L2 * sinA2, L1 + L2 * cosA2);

  float a1Deg = normalizeAngle(degrees(a1));
  float a2Deg = degrees(a2);

  // Accept boundary values with small tolerance.
  const float JOINT_LIMIT_EPS = 0.05f;
  if (a1Deg < A1_MIN_DEG - JOINT_LIMIT_EPS || a1Deg > A1_MAX_DEG + JOINT_LIMIT_EPS) return c;
  if (a2Deg < A2_MIN_DEG - JOINT_LIMIT_EPS || a2Deg > A2_MAX_DEG + JOINT_LIMIT_EPS) return c;

  a1Deg = constrain(a1Deg, A1_MIN_DEG, A1_MAX_DEG);
  a2Deg = constrain(a2Deg, A2_MIN_DEG, A2_MAX_DEG);

  Position check = forwardKinematics(a1Deg, a2Deg, 0.0f);
  float err = sqrt((check.x - x) * (check.x - x) + (check.y - y) * (check.y - y));
  if (err > 2.0f) return c;

  c.valid = true;
  c.a1 = a1Deg;
  c.a2 = a2Deg;
  c.score = angleDistance(a1Deg, refA1) + fabs(a2Deg - refA2);
  return c;
}

IKResult inverseKinematics(float x, float y, const String &mode) {
  IKResult r;
  r.ok = false;
  r.a1 = 0.0f;
  r.a2 = 0.0f;
  r.error = "IK failed";

  float refA1 = getA1();
  float refA2 = getA2();

  IKCandidate pos = solveIKBranch(x, y, true,  refA1, refA2);
  IKCandidate neg = solveIKBranch(x, y, false, refA1, refA2);

  if (mode == "positive") {
    if (!pos.valid) { r.error = "No positive A2 solution"; return r; }
    r.ok = true; r.a1 = pos.a1; r.a2 = pos.a2; r.error = "OK"; return r;
  }

  if (mode == "negative") {
    if (!neg.valid) { r.error = "No negative A2 solution"; return r; }
    r.ok = true; r.a1 = neg.a1; r.a2 = neg.a2; r.error = "OK"; return r;
  }

  if (pos.valid && neg.valid) {
    IKCandidate best = (pos.score <= neg.score) ? pos : neg;
    r.ok = true; r.a1 = best.a1; r.a2 = best.a2; r.error = "OK"; return r;
  }

  if (pos.valid) { r.ok = true; r.a1 = pos.a1; r.a2 = pos.a2; r.error = "OK"; return r; }
  if (neg.valid) { r.ok = true; r.a1 = neg.a1; r.a2 = neg.a2; r.error = "OK"; return r; }

  r.error = "Out of reach or outside joint limits";
  return r;
}

// =====================================================
// Limit Switch
// =====================================================

bool switchPressed(int pin) {
  for (int i = 0; i < 10; i++) {
    if (digitalRead(pin) == HIGH) return false;
    delayMicroseconds(200);
  }
  return true;
}

// =====================================================
// Movement Target Setting
// =====================================================

bool setJointTarget(float a1, float a2, float z) {
  if (!isHomed || isHoming) return false;

  a1 = constrain(a1, A1_MIN_DEG, A1_MAX_DEG);
  a2 = constrain(a2, A2_MIN_DEG, A2_MAX_DEG);
  z  = constrain(z,  Z_MIN_MM,  Z_MAX_MM);

  enableMotors();

  arm1.moveTo(clampA1SafeSteps(a1ToSteps(a1)));
  arm2.moveTo(clampA2SafeSteps(a2ToSteps(a2)));
  zAxis.moveTo(clampZSafeSteps(zToSteps(z)));

  return true;
}

bool setA1Target(float a1) {
  if (!isHomed || isHoming) return false;
  enableMotors();
  arm1.moveTo(clampA1SafeSteps(a1ToSteps(a1)));
  return true;
}

bool setA2Target(float a2) {
  if (!isHomed || isHoming) return false;
  enableMotors();
  arm2.moveTo(clampA2SafeSteps(a2ToSteps(a2)));
  return true;
}

bool setZTarget(float z) {
  if (!isHomed || isHoming) return false;
  enableMotors();
  zAxis.moveTo(clampZSafeSteps(zToSteps(z)));
  return true;
}

// =====================================================
// Safety
// =====================================================

void checkLimitSafety() {
  if (!isHomed || isHoming) return;

  // Preemptive software soft-limit before the mechanical switches.
  if (arm1.distanceToGo() < 0 && arm1.currentPosition() <= A1_SWITCH_CLEARANCE_STEPS) {
    Serial.println("SOFT LIMIT: Arm 1 stopped before switch.");
    arm1.moveTo(arm1.currentPosition());
  }

  if (arm2.distanceToGo() > 0 && arm2.currentPosition() >= -A2_SWITCH_CLEARANCE_STEPS) {
    Serial.println("SOFT LIMIT: Arm 2 stopped before switch.");
    arm2.moveTo(arm2.currentPosition());
  }

  if (zAxis.distanceToGo() < 0 && zAxis.currentPosition() <= Z_SWITCH_CLEARANCE_STEPS) {
    Serial.println("SOFT LIMIT: Z stopped before switch.");
    zAxis.moveTo(zAxis.currentPosition());
  }

  if (arm1.distanceToGo() < 0 && switchPressed(A1_LIMIT)) {
    Serial.println("EMERGENCY: Arm 1 limit hit during movement.");
    stopRequested = true;
  }

  if (arm2.distanceToGo() > 0 && switchPressed(A2_LIMIT)) {
    Serial.println("EMERGENCY: Arm 2 limit hit during movement.");
    stopRequested = true;
  }

  if (zAxis.distanceToGo() < 0 && switchPressed(Z_LIMIT)) {
    Serial.println("EMERGENCY: Z limit hit during movement.");
    stopRequested = true;
  }

  if (stopRequested) {
    haltAllMotion();
    forceMotorsOff();
    isHomed = false;
    stopRequested = false;
  }
}

bool runMotorToTargetBlocking(AccelStepper &motor, const char* name, unsigned long timeoutMS) {
  unsigned long start = millis();

  while (motor.distanceToGo() != 0) {
    motor.run();

    if (millis() - start > timeoutMS) {
      Serial.print(name);
      Serial.println(" move timeout.");
      haltAllMotion();
      return false;
    }
  }

  return true;
}

// =====================================================
// Homing
// =====================================================

bool homeMotorToKnownJoint(
  AccelStepper &motor,
  int limitPin,
  const char* name,
  int homeDir,
  long maxTravelSteps,
  long backoffSteps,
  long switchStepPosition
) {
  Serial.print("Homing ");
  Serial.println(name);

  enableMotors();

  if (switchPressed(limitPin)) {
    Serial.println("Switch already pressed. Backing off first.");
    motor.move(-homeDir * backoffSteps);
    if (!runMotorToTargetBlocking(motor, name, HOME_TIMEOUT_MS)) return false;
    delay(150);
  }

  long startPos = motor.currentPosition();
  motor.move(homeDir * maxTravelSteps);
  unsigned long startTime = millis();

  while (!switchPressed(limitPin)) {
    motor.run();

    if (labs(motor.currentPosition() - startPos) > maxTravelSteps) {
      Serial.print(name);
      Serial.println(" homing failed: max travel reached.");
      haltAllMotion();
      return false;
    }

    if (millis() - startTime > HOME_TIMEOUT_MS) {
      Serial.print(name);
      Serial.println(" homing failed: timeout.");
      haltAllMotion();
      return false;
    }
  }

  Serial.print(name);
  Serial.println(" switch touched.");

  // Reference-code style:
  // Tell AccelStepper that this physical switch is a known joint coordinate.
  motor.setCurrentPosition(switchStepPosition);

  // Move away from the switch. Do not reset position afterward.
  motor.move(-homeDir * backoffSteps);
  if (!runMotorToTargetBlocking(motor, name, HOME_TIMEOUT_MS)) return false;

  if (switchPressed(limitPin)) {
    Serial.print(name);
    Serial.println(" warning: switch still pressed after backoff.");
  }

  Serial.print(name);
  Serial.println(" homed.");
  return true;
}

bool homeAll() {
  Serial.println("Starting homing sequence.");

  isHoming = true;
  isHomed = false;
  stopRequested = false;
  enableMotors();

  if (!homeMotorToKnownJoint(
        zAxis,
        Z_LIMIT,
        "Z Axis",
        Z_HOME_DIR,
        Z_HOME_MAX_STEPS,
        Z_BACKOFF_STEPS,
        0
      )) {
    isHoming = false;
    forceMotorsOff();
    return false;
  }

  zAxis.moveTo(zToSteps(0.0f));
  if (!runMotorToTargetBlocking(zAxis, "Z Axis", HOME_TIMEOUT_MS)) {
    isHoming = false;
    forceMotorsOff();
    return false;
  }

  delay(250);

  if (!homeMotorToKnownJoint(
        arm1,
        A1_LIMIT,
        "Arm 1",
        A1_HOME_DIR,
        A1_HOME_MAX_STEPS,
        A1_BACKOFF_STEPS,
        0
      )) {
    isHoming = false;
    forceMotorsOff();
    return false;
  }

  arm1.moveTo(a1ToSteps(0.0f));
  if (!runMotorToTargetBlocking(arm1, "Arm 1", HOME_TIMEOUT_MS)) {
    isHoming = false;
    forceMotorsOff();
    return false;
  }

  delay(250);

  if (!homeMotorToKnownJoint(
        arm2,
        A2_LIMIT,
        "Arm 2",
        A2_HOME_DIR,
        A2_HOME_MAX_STEPS,
        A2_BACKOFF_STEPS,
        0
      )) {
    isHoming = false;
    forceMotorsOff();
    return false;
  }

  arm2.moveTo(a2ToSteps(0.0f));
  if (!runMotorToTargetBlocking(arm2, "Arm 2", HOME_TIMEOUT_MS)) {
    isHoming = false;
    forceMotorsOff();
    return false;
  }

  isHoming = false;
  isHomed = true;
  applyIdleMotorPolicy();

  Serial.println("Homing complete. Robot at user zero.");
  printPosition("HOME");
  return true;
}

// =====================================================
// Web Helpers
// =====================================================

void noCache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

void sendJson(int code, const String &json) {
  noCache();
  server.send(code, "application/json", json);
}

void sendText(int code, const String &text) {
  noCache();
  server.send(code, "text/plain", text);
}

// =====================================================
// Web Server Handlers
// =====================================================

void handleRoot() {
  noCache();
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleMove() {
  if (!server.hasArg("a1") || !server.hasArg("a2") || !server.hasArg("z")) {
    sendText(400, "Missing params");
    return;
  }

  if (!isHomed) {
    sendText(409, "Home first");
    return;
  }

  float a1 = server.arg("a1").toFloat();
  float a2 = server.arg("a2").toFloat();
  float z  = server.arg("z").toFloat();

  if (!setJointTarget(a1, a2, z)) {
    sendText(409, "Move rejected");
    return;
  }

  Position p = forwardKinematics(a1, a2, z);

  String json = "{";
  json += "\"ok\":true";
  json += ",\"x\":" + String(p.x, 1);
  json += ",\"y\":" + String(p.y, 1);
  json += ",\"z\":" + String(p.z, 1);
  json += "}";

  sendJson(200, json);
}

void handleMoveRT() {
  if (!isHomed) {
    sendText(409, "Home first");
    return;
  }

  if (server.hasArg("a1")) setA1Target(server.arg("a1").toFloat());
  if (server.hasArg("a2")) setA2Target(server.arg("a2").toFloat());
  if (server.hasArg("z"))  setZTarget(server.arg("z").toFloat());

  sendText(200, "OK");
}

void handleHome() {
  bool ok = homeAll();
  if (ok) sendText(200, "OK");
  else sendText(500, "Homing failed");
}

void handleIK() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("z")) {
    sendJson(400, "{\"ok\":false,\"err\":\"Missing params\"}");
    return;
  }

  if (!isHomed) {
    sendJson(409, "{\"ok\":false,\"err\":\"Home first\"}");
    return;
  }

  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  float z = server.arg("z").toFloat();

  if (z < Z_MIN_MM || z > Z_MAX_MM) {
    sendJson(200, "{\"ok\":false,\"err\":\"Z outside limit\"}");
    return;
  }

  String mode = "closest";
  if (server.hasArg("mode")) {
    mode = server.arg("mode");
    mode.toLowerCase();
  }

  IKResult ik = inverseKinematics(x, y, mode);

  if (!ik.ok) {
    sendJson(200, "{\"ok\":false,\"err\":\"" + ik.error + "\"}");
    return;
  }

  if (!setJointTarget(ik.a1, ik.a2, z)) {
    sendJson(409, "{\"ok\":false,\"err\":\"Move rejected\"}");
    return;
  }

  Position p = forwardKinematics(ik.a1, ik.a2, z);

  String json = "{";
  json += "\"ok\":true";
  json += ",\"a1\":" + String(ik.a1, 2);
  json += ",\"a2\":" + String(ik.a2, 2);
  json += ",\"x\":"  + String(p.x, 1);
  json += ",\"y\":"  + String(p.y, 1);
  json += ",\"z\":"  + String(p.z, 1);
  json += "}";

  sendJson(200, json);
}

void handlePosition() {
  float a1 = getA1();
  float a2 = getA2();
  float z  = getZ();

  Position p = forwardKinematics(a1, a2, z);

  bool moving = (
    arm1.distanceToGo() != 0 ||
    arm2.distanceToGo() != 0 ||
    zAxis.distanceToGo() != 0
  );

  String json = "{";
  json += "\"a1\":" + String(a1, 2);
  json += ",\"a2\":" + String(a2, 2);
  json += ",\"z\":"  + String(z, 2);
  json += ",\"x\":"  + String(p.x, 2);
  json += ",\"y\":"  + String(p.y, 2);
  json += ",\"homed\":";
  json += (isHomed ? "true" : "false");
  json += ",\"moving\":";
  json += (moving ? "true" : "false");
  json += "}";

  sendJson(200, json);
}

void handleStop() {
  haltAllMotion();
  forceMotorsOff();
  isHomed = false;
  isHoming = false;
  sendText(200, "STOPPED. Re-home required.");
}

void handleOff() {
  haltAllMotion();
  forceMotorsOff();
  isHomed = false;
  sendText(200, "Motors OFF. Re-home required.");
}

void handleCalJson() {
  sendJson(200, calibrationJson());
}

void handleSaveCal() {
  saveCalibration();
  sendJson(200, calibrationJson());
}

void handleClearCal() {
  clearCalibration();
  sendJson(200, calibrationJson());
}

void handleSetParam() {
  if (!server.hasArg("name") || !server.hasArg("value")) {
    sendText(400, "Missing name/value");
    return;
  }

  String name = server.arg("name");
  name.toLowerCase();
  float value = server.arg("value").toFloat();

  if (name == "a1spd") A1_STEPS_PER_DEG = value;
  else if (name == "a2spd") A2_STEPS_PER_DEG = value;
  else if (name == "zspmm") Z_STEPS_PER_MM = value;
  else if (name == "a1home") A1_HOME_DEG = value;
  else if (name == "a2home") A2_HOME_DEG = value;
  else if (name == "zhome") Z_HOME_MM = value;
  else if (name == "a1trim") A1_TRIM_DEG = value;
  else if (name == "a2trim") A2_TRIM_DEG = value;
  else if (name == "ztrim") Z_TRIM_MM = value;
  else if (name == "armspeed") ARM_MAX_SPEED = value;
  else if (name == "armaccel") ARM_ACCEL = value;
  else {
    sendText(400, "Bad parameter");
    return;
  }

  applySpeedSettings();

  // Changing calibration while homed invalidates current physical coordinate.
  if (name != "armspeed" && name != "armaccel") {
    isHomed = false;
  }

  sendJson(200, calibrationJson());
}

void handleNudgeParam() {
  if (!server.hasArg("name") || !server.hasArg("mul")) {
    sendText(400, "Missing name/mul");
    return;
  }

  String name = server.arg("name");
  name.toLowerCase();
  float mul = server.arg("mul").toFloat();

  if (mul <= 0.0f) {
    sendText(400, "Bad multiplier");
    return;
  }

  if (name == "a1spd") A1_STEPS_PER_DEG *= mul;
  else if (name == "a2spd") A2_STEPS_PER_DEG *= mul;
  else if (name == "zspmm") Z_STEPS_PER_MM *= mul;
  else {
    sendText(400, "Bad parameter");
    return;
  }

  isHomed = false;
  sendJson(200, calibrationJson());
}

void handleJogRelative() {
  if (!isHomed) {
    sendText(409, "Home first");
    return;
  }

  if (!server.hasArg("axis") || !server.hasArg("value")) {
    sendText(400, "Missing axis/value");
    return;
  }

  String axis = server.arg("axis");
  axis.toLowerCase();
  float v = server.arg("value").toFloat();

  bool ok = false;
  if (axis == "a1") ok = setA1Target(getA1() + v);
  else if (axis == "a2") ok = setA2Target(getA2() + v);
  else if (axis == "z") ok = setZTarget(getZ() + v);

  if (ok) sendText(200, "Jog accepted");
  else sendText(409, "Jog rejected");
}

// =====================================================
// Serial Helpers
// =====================================================

void printPosition(const char* label) {
  float a1 = getA1();
  float a2 = getA2();
  float z  = getZ();

  Position p = forwardKinematics(a1, a2, z);

  Serial.print(label);
  Serial.print(" A1:"); Serial.print(a1, 2);
  Serial.print(" A2:"); Serial.print(a2, 2);
  Serial.print(" Z:");  Serial.print(z, 2);
  Serial.print(" | X:"); Serial.print(p.x, 2);
  Serial.print(" Y:");  Serial.print(p.y, 2);
  Serial.println(" mm");
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("HOME");
  Serial.println("MOVE a1 a2 z");
  Serial.println("ARM1 deg");
  Serial.println("ARM2 deg");
  Serial.println("Z mm");
  Serial.println("IK x y z");
  Serial.println("WHERE");
  Serial.println("STOP");
  Serial.println("OFF");
  Serial.println("CAL");
  Serial.println("SAVECAL");
  Serial.println("CLEARCAL");
  Serial.println("A1SPD value");
  Serial.println("A2SPD value");
  Serial.println("A1HOME value");
  Serial.println("A2HOME value");
}

void parseCommand(String cmd) {
  cmd.trim();

  String upper = cmd;
  upper.toUpperCase();

  if (upper == "HELP") { printHelp(); return; }
  if (upper == "HOME") { homeAll(); return; }
  if (upper == "WHERE") { printPosition("Pos"); return; }
  if (upper == "CAL") { printCalibration(); return; }
  if (upper == "SAVECAL") { saveCalibration(); return; }
  if (upper == "CLEARCAL") { clearCalibration(); return; }

  if (upper == "STOP") {
    haltAllMotion(); forceMotorsOff(); isHomed = false; isHoming = false;
    Serial.println("STOPPED. Re-home required."); return;
  }

  if (upper == "OFF") {
    haltAllMotion(); forceMotorsOff(); isHomed = false;
    Serial.println("Motors OFF. Re-home required."); return;
  }

  if (upper.startsWith("A1SPD ")) { A1_STEPS_PER_DEG = upper.substring(6).toFloat(); isHomed = false; Serial.println("A1 steps/deg set. Re-home."); return; }
  if (upper.startsWith("A2SPD ")) { A2_STEPS_PER_DEG = upper.substring(6).toFloat(); isHomed = false; Serial.println("A2 steps/deg set. Re-home."); return; }
  if (upper.startsWith("A1HOME ")) { A1_HOME_DEG = upper.substring(7).toFloat(); isHomed = false; Serial.println("A1 home angle set. Re-home."); return; }
  if (upper.startsWith("A2HOME ")) { A2_HOME_DEG = upper.substring(7).toFloat(); isHomed = false; Serial.println("A2 home angle set. Re-home."); return; }

  if (upper.startsWith("MOVE")) {
    float a1, a2, z;
    if (sscanf(upper.c_str(), "MOVE %f %f %f", &a1, &a2, &z) == 3) {
      Serial.println(setJointTarget(a1, a2, z) ? "Move accepted." : "Move rejected.");
    } else Serial.println("Use: MOVE a1 a2 z");
    return;
  }

  if (upper.startsWith("ARM1")) {
    float v;
    if (sscanf(upper.c_str(), "ARM1 %f", &v) == 1) Serial.println(setA1Target(v) ? "Arm1 accepted." : "Arm1 rejected.");
    return;
  }

  if (upper.startsWith("ARM2")) {
    float v;
    if (sscanf(upper.c_str(), "ARM2 %f", &v) == 1) Serial.println(setA2Target(v) ? "Arm2 accepted." : "Arm2 rejected.");
    return;
  }

  if (upper.startsWith("Z ")) {
    float v;
    if (sscanf(upper.c_str(), "Z %f", &v) == 1) Serial.println(setZTarget(v) ? "Z accepted." : "Z rejected.");
    return;
  }

  if (upper.startsWith("IK")) {
    float x, y, z;
    if (sscanf(upper.c_str(), "IK %f %f %f", &x, &y, &z) == 3) {
      IKResult ik = inverseKinematics(x, y, "closest");
      if (!ik.ok) {
        Serial.print("IK failed: ");
        Serial.println(ik.error);
        return;
      }
      Serial.println(setJointTarget(ik.a1, ik.a2, z) ? "IK accepted." : "IK rejected.");
    } else Serial.println("Use: IK x y z");
    return;
  }

  Serial.print("? Unknown command: ");
  Serial.println(cmd);
}

// =====================================================
// Motion Task
// =====================================================

void runMotionTask() {
  bool moving = (
    arm1.distanceToGo() != 0 ||
    arm2.distanceToGo() != 0 ||
    zAxis.distanceToGo() != 0
  );

  if (moving) {
    enableMotors();
    checkLimitSafety();

    arm1.run();
    arm2.run();
    zAxis.run();

    lastMoveTime = millis();
  } else {
    if (millis() - lastMoveTime > IDLE_DISABLE_DELAY_MS) {
      applyIdleMotorPolicy();
    }
  }
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  loadCalibration();

  pinMode(A1_EN, OUTPUT);
  pinMode(A2_EN, OUTPUT);
  pinMode(Z_EN, OUTPUT);

  forceMotorsOff();

  pinMode(A1_LIMIT, INPUT_PULLUP);
  pinMode(A2_LIMIT, INPUT_PULLUP);
  pinMode(Z_LIMIT, INPUT_PULLUP);

  // Keep the direction setup from your working code.
  arm1.setPinsInverted(true, false, false);
  arm2.setPinsInverted(false, false, false);
  zAxis.setPinsInverted(false, false, false);

  applySpeedSettings();

  delay(1000);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);

  Serial.print("AP: ");
  Serial.println(AP_SSID);
  Serial.print("http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/move_rt", handleMoveRT);
  server.on("/home", handleHome);
  server.on("/ik", handleIK);
  server.on("/position", handlePosition);
  server.on("/stop", handleStop);
  server.on("/off", handleOff);
  server.on("/caljson", handleCalJson);
  server.on("/savecal", handleSaveCal);
  server.on("/clearcal", handleClearCal);
  server.on("/setparam", handleSetParam);
  server.on("/nudgeparam", handleNudgeParam);
  server.on("/jogrel", handleJogRelative);

  server.begin();

  Serial.println("SCARA Robot v9 ready.");
  Serial.println("Reference-style homing + protected negative soft limits.");
  printHelp();
  printCalibration();
}

// =====================================================
// Loop
// =====================================================

void loop() {
  server.handleClient();
  runMotionTask();

  if (Serial.available()) {
    parseCommand(Serial.readStringUntil('\n'));
  }
}
