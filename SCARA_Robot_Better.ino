#include <AccelStepper.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include "webpage.h"

// ════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════

// ── Arm 1 (Shoulder) ──
#define A1_STEP   17
#define A1_DIR    16
#define A1_EN     5
#define A1_LIMIT  14

// ── Arm 2 (Elbow) ──
#define A2_STEP   19
#define A2_DIR    18
#define A2_EN     21
#define A2_LIMIT  27

// ── Z Axis ──
#define Z_STEP    2
#define Z_DIR     15
#define Z_EN      4
#define Z_LIMIT   12

// ════════════════════════════════════════
//  ROBOT CALIBRATION
// ════════════════════════════════════════

// Steps calibration
#define A1_STEPS_PER_DEG  142.22f
#define A2_STEPS_PER_DEG  40.89f
#define Z_STEPS_PER_MM    400.0f

// User coordinate offsets
// Arm1: -180 at limit switch, 0 at center
// Arm2: +160 at limit switch, 0 at center
// Z:    limit switch at top, 0 at working middle
constexpr float A1_ZERO_OFFSET = 180.0f;
constexpr float A2_ZERO_OFFSET = 160.0f;
constexpr float Z_ZERO_MM      = 137.5f;

// Link lengths in mm
constexpr float L1 = 136.5f;
constexpr float L2 = 95.5f;

// Safe software limits.
// These preserve your old working range.
// If Arm 1 physically hits the robot body before +180, reduce A1_MAX_DEG, for example to 140.
constexpr float A1_MIN_DEG = -180.0f;
constexpr float A1_MAX_DEG =  180.0f;

constexpr float A2_MIN_DEG = -160.0f;
constexpr float A2_MAX_DEG =  160.0f;

constexpr float Z_MIN_MM = -135.0f;
constexpr float Z_MAX_MM =  135.0f;

// Homing safety
constexpr unsigned long HOME_TIMEOUT_MS = 15000;
constexpr long HOME_BACKOFF_STEPS = 300;

// Idle motor shutdown
constexpr unsigned long MOTOR_IDLE_DISABLE_MS = 3000;

// WiFi Access Point
const char* AP_SSID = "SCARA-Robot";
const char* AP_PASS = "robot1234";

// ════════════════════════════════════════
//  GLOBAL STATE
// ════════════════════════════════════════

WebServer server(80);

AccelStepper arm1(AccelStepper::DRIVER, A1_STEP, A1_DIR);
AccelStepper arm2(AccelStepper::DRIVER, A2_STEP, A2_DIR);
AccelStepper zAxis(AccelStepper::DRIVER, Z_STEP, Z_DIR);

struct Position {
  float x;
  float y;
  float z;
};

float currentA1_deg = 0.0f;
float currentA2_deg = 0.0f;
float currentZ_mm   = 0.0f;

float targetA1_deg = 0.0f;
float targetA2_deg = 0.0f;
float targetZ_mm   = 0.0f;

bool isHomed = false;
bool motorsEnabled = true;
bool emergencyStopped = false;

unsigned long lastMoveTime = 0;

// ════════════════════════════════════════
//  BASIC HELPERS
// ════════════════════════════════════════

bool inRange(float v, float mn, float mx) {
  return (v >= mn && v <= mx);
}

float normalizeAngle(float deg) {
  while (deg >  180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

String jsonBool(bool v) {
  return v ? "true" : "false";
}

void sendJsonError(const String& err) {
  server.send(200, "application/json", "{\"ok\":false,\"err\":\"" + err + "\"}");
}

// ════════════════════════════════════════
//  STEP / USER UNIT CONVERSIONS
// ════════════════════════════════════════

long arm1DegToSteps(float deg) {
  return lround((deg + A1_ZERO_OFFSET) * A1_STEPS_PER_DEG);
}

long arm2DegToSteps(float deg) {
  return lround((deg - A2_ZERO_OFFSET) * A2_STEPS_PER_DEG);
}

long zMmToSteps(float mm) {
  return lround((mm + Z_ZERO_MM) * Z_STEPS_PER_MM);
}

float arm1StepsToDeg(long steps) {
  return (steps / A1_STEPS_PER_DEG) - A1_ZERO_OFFSET;
}

float arm2StepsToDeg(long steps) {
  return (steps / A2_STEPS_PER_DEG) + A2_ZERO_OFFSET;
}

float zStepsToMm(long steps) {
  return (steps / Z_STEPS_PER_MM) - Z_ZERO_MM;
}

void updateCurrentFromMotors() {
  currentA1_deg = arm1StepsToDeg(arm1.currentPosition());
  currentA2_deg = arm2StepsToDeg(arm2.currentPosition());
  currentZ_mm   = zStepsToMm(zAxis.currentPosition());
}

// ════════════════════════════════════════
//  MOTOR ENABLE / DISABLE / STOP
// ════════════════════════════════════════

void enableMotors() {
  digitalWrite(A1_EN, LOW);
  digitalWrite(A2_EN, LOW);
  digitalWrite(Z_EN,  LOW);
  motorsEnabled = true;
  lastMoveTime = millis();
}

void disableMotors() {
  digitalWrite(A1_EN, HIGH);
  digitalWrite(A2_EN, HIGH);
  digitalWrite(Z_EN,  HIGH);
  motorsEnabled = false;
}

void stopMotionNow() {
  arm1.stop();
  arm2.stop();
  zAxis.stop();

  arm1.moveTo(arm1.currentPosition());
  arm2.moveTo(arm2.currentPosition());
  zAxis.moveTo(zAxis.currentPosition());

  updateCurrentFromMotors();

  targetA1_deg = currentA1_deg;
  targetA2_deg = currentA2_deg;
  targetZ_mm   = currentZ_mm;

  disableMotors();
  emergencyStopped = true;
}

// ════════════════════════════════════════
//  KINEMATICS
// ════════════════════════════════════════

Position forwardKinematics(float a1_deg, float a2_deg, float z_mm) {
  float a1 = radians(a1_deg);
  float a2 = radians(a2_deg);

  Position p;
  p.x = L1 * cos(a1) + L2 * cos(a1 + a2);
  p.y = L1 * sin(a1) + L2 * sin(a1 + a2);
  p.z = z_mm;
  return p;
}

bool inverseKinematics(float x, float y, float &a1_deg, float &a2_deg, bool elbowUp = true) {
  float d2 = x * x + y * y;
  float cosA2Raw = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  // Target is outside physical reach.
  if (cosA2Raw < -1.01f || cosA2Raw > 1.01f) {
    return false;
  }

  float cosA2 = fmax(-1.0f, fmin(1.0f, cosA2Raw));
  float sinA2 = elbowUp ? sqrt(1.0f - cosA2 * cosA2)
                        : -sqrt(1.0f - cosA2 * cosA2);

  float a2 = atan2(sinA2, cosA2);
  float a1 = atan2(y, x) - atan2(L2 * sinA2, L1 + L2 * cosA2);

  float a1d = normalizeAngle(degrees(a1));
  float a2d = degrees(a2);

  // Near the ±180 boundary, FK gives almost the same point.
  // Pick the value that is valid and gives the smallest FK error.
  if (fabs(fabs(a1d) - 180.0f) < 1.0f) {
    float candidates[2] = { -180.0f, 180.0f };
    float bestErr = 999999.0f;
    float bestA1 = a1d;

    for (int i = 0; i < 2; i++) {
      float c = candidates[i];

      if (!inRange(c, A1_MIN_DEG, A1_MAX_DEG)) {
        continue;
      }

      float r1 = radians(c);
      float r2 = radians(a2d);

      float vx = L1 * cos(r1) + L2 * cos(r1 + r2);
      float vy = L1 * sin(r1) + L2 * sin(r1 + r2);
      float err = sqrt((vx - x) * (vx - x) + (vy - y) * (vy - y));

      if (err < bestErr) {
        bestErr = err;
        bestA1 = c;
      }
    }

    a1d = bestA1;
  }

  // Do not silently clamp IK. If the solution is outside safe joint limits, reject it.
  if (!inRange(a1d, A1_MIN_DEG, A1_MAX_DEG)) return false;
  if (!inRange(a2d, A2_MIN_DEG, A2_MAX_DEG)) return false;

  a1_deg = a1d;
  a2_deg = a2d;
  return true;
}

// ════════════════════════════════════════
//  MOTION TARGETS
// ════════════════════════════════════════

bool validateTarget(float a1, float a2, float z, String &err) {
  if (!inRange(a1, A1_MIN_DEG, A1_MAX_DEG)) {
    err = "Arm 1 outside safe limit";
    return false;
  }

  if (!inRange(a2, A2_MIN_DEG, A2_MAX_DEG)) {
    err = "Arm 2 outside safe limit";
    return false;
  }

  if (!inRange(z, Z_MIN_MM, Z_MAX_MM)) {
    err = "Z outside safe limit";
    return false;
  }

  return true;
}

bool setMoveTarget(float a1, float a2, float z, String &err) {
  if (!isHomed) {
    err = "Home first";
    return false;
  }

  if (!validateTarget(a1, a2, z, err)) {
    return false;
  }

  enableMotors();
  emergencyStopped = false;

  targetA1_deg = a1;
  targetA2_deg = a2;
  targetZ_mm   = z;

  arm1.moveTo(arm1DegToSteps(a1));
  arm2.moveTo(arm2DegToSteps(a2));
  zAxis.moveTo(zMmToSteps(z));

  lastMoveTime = millis();
  return true;
}

bool robotIsMoving() {
  return (arm1.distanceToGo() != 0 ||
          arm2.distanceToGo() != 0 ||
          zAxis.distanceToGo() != 0);
}

// ════════════════════════════════════════
//  HOMING
// ════════════════════════════════════════

bool switchPressed(int pin) {
  // INPUT_PULLUP: pressed = LOW.
  // Require stable LOW for simple debounce.
  for (int i = 0; i < 10; i++) {
    if (digitalRead(pin) == HIGH) {
      return false;
    }
    delayMicroseconds(200);
  }
  return true;
}

bool runUntilSwitch(AccelStepper &motor, int limitPin, unsigned long timeoutMs) {
  unsigned long start = millis();

  while (!switchPressed(limitPin)) {
    motor.run();

    if (millis() - start > timeoutMs) {
      motor.stop();
      return false;
    }
  }

  return true;
}

// Arm1 and Z: home by moving negative until limit hit.
bool homeMotorNeg(AccelStepper &motor, int limitPin, const char* name) {
  Serial.print("Homing ");
  Serial.println(name);

  motor.moveTo(-999999);

  if (!runUntilSwitch(motor, limitPin, HOME_TIMEOUT_MS)) {
    Serial.print("HOME FAILED: ");
    Serial.println(name);
    return false;
  }

  motor.stop();
  motor.setCurrentPosition(0);

  motor.moveTo(HOME_BACKOFF_STEPS);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }

  Serial.print(name);
  Serial.println(" homed!");
  return true;
}

// Arm2: home by moving positive until limit hit.
bool homeMotorPos(AccelStepper &motor, int limitPin, const char* name) {
  Serial.print("Homing ");
  Serial.println(name);

  motor.moveTo(999999);

  if (!runUntilSwitch(motor, limitPin, HOME_TIMEOUT_MS)) {
    Serial.print("HOME FAILED: ");
    Serial.println(name);
    return false;
  }

  motor.stop();
  motor.setCurrentPosition(0);

  motor.moveTo(-HOME_BACKOFF_STEPS);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }

  Serial.print(name);
  Serial.println(" homed!");
  return true;
}

bool homeAll() {
  enableMotors();
  emergencyStopped = false;
  isHomed = false;

  // 1. Home Z first for safety.
  if (!homeMotorNeg(zAxis, Z_LIMIT, "Z Axis")) {
    disableMotors();
    return false;
  }

  delay(300);

  Serial.println("Z -> user zero");
  zAxis.moveTo(zMmToSteps(0.0f));
  while (zAxis.distanceToGo() != 0) {
    zAxis.run();
  }

  delay(300);

  // 2. Home Arm 1.
  if (!homeMotorNeg(arm1, A1_LIMIT, "Arm 1")) {
    disableMotors();
    return false;
  }

  delay(300);

  Serial.println("Arm1 -> user zero");
  arm1.moveTo(arm1DegToSteps(0.0f));
  while (arm1.distanceToGo() != 0) {
    arm1.run();
  }

  delay(300);

  // 3. Home Arm 2.
  if (!homeMotorPos(arm2, A2_LIMIT, "Arm 2")) {
    disableMotors();
    return false;
  }

  delay(300);

  Serial.println("Arm2 -> user zero");
  arm2.moveTo(arm2DegToSteps(0.0f));
  while (arm2.distanceToGo() != 0) {
    arm2.run();
  }

  currentA1_deg = 0.0f;
  currentA2_deg = 0.0f;
  currentZ_mm   = 0.0f;

  targetA1_deg = 0.0f;
  targetA2_deg = 0.0f;
  targetZ_mm   = 0.0f;

  isHomed = true;
  lastMoveTime = millis();

  Serial.println("All homed! Robot at 0 / 0 / 0");
  return true;
}

// ════════════════════════════════════════
//  WEB SERVER HANDLERS
// ════════════════════════════════════════

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleMove() {
  if (!server.hasArg("a1") || !server.hasArg("a2") || !server.hasArg("z")) {
    sendJsonError("Missing params");
    return;
  }

  float a1 = server.arg("a1").toFloat();
  float a2 = server.arg("a2").toFloat();
  float z  = server.arg("z").toFloat();

  String err;
  if (!setMoveTarget(a1, a2, z, err)) {
    sendJsonError(err);
    return;
  }

  Position p = forwardKinematics(a1, a2, z);

  String j = "{";
  j += "\"ok\":true,";
  j += "\"moving\":true,";
  j += "\"targetA1\":" + String(a1, 2) + ",";
  j += "\"targetA2\":" + String(a2, 2) + ",";
  j += "\"targetZ\":"  + String(z, 2)  + ",";
  j += "\"targetX\":"  + String(p.x, 1) + ",";
  j += "\"targetY\":"  + String(p.y, 1);
  j += "}";

  server.send(200, "application/json", j);
}

void handleMoveRT() {
  if (!server.hasArg("a1") || !server.hasArg("a2") || !server.hasArg("z")) {
    sendJsonError("Missing params");
    return;
  }

  float a1 = server.arg("a1").toFloat();
  float a2 = server.arg("a2").toFloat();
  float z  = server.arg("z").toFloat();

  String err;
  if (!setMoveTarget(a1, a2, z, err)) {
    sendJsonError(err);
    return;
  }

  server.send(200, "application/json", "{\"ok\":true,\"moving\":true}");
}

void handleHome() {
  bool ok = homeAll();

  if (ok) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(200, "application/json", "{\"ok\":false,\"err\":\"Homing failed. Check limit switch wiring or direction.\"}");
  }
}

void handleStop() {
  stopMotionNow();
  server.send(200, "application/json", "{\"ok\":true,\"stopped\":true}");
}

void handleIK() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("z")) {
    sendJsonError("Missing params");
    return;
  }

  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  float z = server.arg("z").toFloat();

  bool elbowUp = !(server.hasArg("elbow") && server.arg("elbow") == "0");

  float a1;
  float a2;

  if (!inverseKinematics(x, y, a1, a2, elbowUp)) {
    sendJsonError("Out of reach or outside joint limits");
    return;
  }

  String err;
  if (!setMoveTarget(a1, a2, z, err)) {
    sendJsonError(err);
    return;
  }

  Position p = forwardKinematics(a1, a2, z);

  String j = "{";
  j += "\"ok\":true,";
  j += "\"moving\":true,";
  j += "\"a1\":" + String(a1, 2) + ",";
  j += "\"a2\":" + String(a2, 2) + ",";
  j += "\"z\":"  + String(z, 2)  + ",";
  j += "\"x\":"  + String(p.x, 1) + ",";
  j += "\"y\":"  + String(p.y, 1);
  j += "}";

  server.send(200, "application/json", j);
}

void handlePosition() {
  updateCurrentFromMotors();
  Position p = forwardKinematics(currentA1_deg, currentA2_deg, currentZ_mm);

  bool moving = robotIsMoving();

  String j = "{";
  j += "\"a1\":" + String(currentA1_deg, 2) + ",";
  j += "\"a2\":" + String(currentA2_deg, 2) + ",";
  j += "\"z\":"  + String(currentZ_mm, 2) + ",";
  j += "\"x\":"  + String(p.x, 1) + ",";
  j += "\"y\":"  + String(p.y, 1) + ",";
  j += "\"targetA1\":" + String(targetA1_deg, 2) + ",";
  j += "\"targetA2\":" + String(targetA2_deg, 2) + ",";
  j += "\"targetZ\":"  + String(targetZ_mm, 2) + ",";
  j += "\"homed\":" + jsonBool(isHomed) + ",";
  j += "\"moving\":" + jsonBool(moving) + ",";
  j += "\"stopped\":" + jsonBool(emergencyStopped);
  j += "}";

  server.send(200, "application/json", j);
}

// ════════════════════════════════════════
//  SERIAL COMMANDS
// ════════════════════════════════════════

void printPosition(const char* label, Position pos) {
  Serial.print(label);
  Serial.print(" X:");
  Serial.print(pos.x, 2);
  Serial.print(" Y:");
  Serial.print(pos.y, 2);
  Serial.print(" Z:");
  Serial.print(pos.z, 2);
  Serial.println("mm");
}

void parseCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd.startsWith("MOVE")) {
    float a1, a2, z;

    if (sscanf(cmd.c_str(), "MOVE %f %f %f", &a1, &a2, &z) == 3) {
      String err;
      if (!setMoveTarget(a1, a2, z, err)) {
        Serial.print("ERR: ");
        Serial.println(err);
      }
    } else {
      Serial.println("Usage: MOVE a1 a2 z");
    }
    return;
  }

  if (cmd.startsWith("ARM1")) {
    float v;

    if (sscanf(cmd.c_str(), "ARM1 %f", &v) == 1) {
      String err;
      if (!setMoveTarget(v, targetA2_deg, targetZ_mm, err)) {
        Serial.print("ERR: ");
        Serial.println(err);
      }
    }
    return;
  }

  if (cmd.startsWith("ARM2")) {
    float v;

    if (sscanf(cmd.c_str(), "ARM2 %f", &v) == 1) {
      String err;
      if (!setMoveTarget(targetA1_deg, v, targetZ_mm, err)) {
        Serial.print("ERR: ");
        Serial.println(err);
      }
    }
    return;
  }

  if (cmd.startsWith("Z ")) {
    float v;

    if (sscanf(cmd.c_str(), "Z %f", &v) == 1) {
      String err;
      if (!setMoveTarget(targetA1_deg, targetA2_deg, v, err)) {
        Serial.print("ERR: ");
        Serial.println(err);
      }
    }
    return;
  }

  if (cmd.startsWith("IK")) {
    float x, y, z;

    if (sscanf(cmd.c_str(), "IK %f %f %f", &x, &y, &z) == 3) {
      float a1, a2;

      if (!inverseKinematics(x, y, a1, a2, true)) {
        Serial.println("ERR: IK target out of reach or outside joint limits");
        return;
      }

      String err;
      if (!setMoveTarget(a1, a2, z, err)) {
        Serial.print("ERR: ");
        Serial.println(err);
      }
    } else {
      Serial.println("Usage: IK x y z");
    }
    return;
  }

  if (cmd.startsWith("WHERE")) {
    updateCurrentFromMotors();
    printPosition("Pos", forwardKinematics(currentA1_deg, currentA2_deg, currentZ_mm));
    return;
  }

  if (cmd.startsWith("HOME")) {
    homeAll();
    return;
  }

  if (cmd.startsWith("STOP")) {
    stopMotionNow();
    Serial.println("STOPPED");
    return;
  }

  Serial.print("? ");
  Serial.println(cmd);
}

// ════════════════════════════════════════
//  SETUP & LOOP
// ════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  pinMode(A1_EN, OUTPUT);
  pinMode(A2_EN, OUTPUT);
  pinMode(Z_EN,  OUTPUT);

  digitalWrite(A1_EN, LOW);
  digitalWrite(A2_EN, LOW);
  digitalWrite(Z_EN,  LOW);

  pinMode(A1_LIMIT, INPUT_PULLUP);
  pinMode(A2_LIMIT, INPUT_PULLUP);
  pinMode(Z_LIMIT,  INPUT_PULLUP);

  // Arm1: inverted. Positive user motion moves away from the -180 limit.
  arm1.setPinsInverted(true, false, false);

  // Arm2: not inverted. Positive motor direction moves toward the +160 limit.
  arm2.setPinsInverted(false, false, false);

  zAxis.setPinsInverted(false, false, false);

  // Safer speeds than the old sketch.
  // Increase only after the robot moves reliably without missed steps.
  arm1.setMaxSpeed(2200);
  arm1.setAcceleration(700);

  arm2.setMaxSpeed(1800);
  arm2.setAcceleration(600);

  zAxis.setMaxSpeed(3000);
  zAxis.setAcceleration(900);

  delay(1000);

  WiFi.mode(WIFI_AP);
  delay(100);

  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);

  Serial.print("AP: ");
  Serial.println(AP_SSID);
  Serial.print("http://");
  Serial.println(WiFi.softAPIP());

  server.on("/",         handleRoot);
  server.on("/move",     handleMove);
  server.on("/move_rt",  handleMoveRT);
  server.on("/home",     handleHome);
  server.on("/stop",     handleStop);
  server.on("/ik",       handleIK);
  server.on("/position", handlePosition);

  server.begin();

  Serial.println("Ready. Connect to WiFi AP and press HOME.");
}

void loop() {
  server.handleClient();

  if (motorsEnabled && !emergencyStopped) {
    bool moving = robotIsMoving();

    if (moving) {
      arm1.run();
      arm2.run();
      zAxis.run();

      updateCurrentFromMotors();
      lastMoveTime = millis();
    } else {
      updateCurrentFromMotors();

      if (millis() - lastMoveTime > MOTOR_IDLE_DISABLE_MS) {
        disableMotors();
      }
    }
  }

  if (Serial.available()) {
    parseCommand(Serial.readStringUntil('\n'));
  }
}
