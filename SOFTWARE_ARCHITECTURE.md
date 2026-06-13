# Software Architecture 

The firmware runs entirely on an ESP32. There is no host PC involvement during normal operation. The browser connects over WiFi, sends HTTP requests to the ESP32's built-in web server, and polls a JSON status endpoint to update the UI. All motion, kinematics, safety, and storage run on the microcontroller.

---

## Source File Map

Each `.ino` tab has a single focused responsibility. The main tab (`SCARA_ROBOT_ARM_CODE.ino`) holds all shared state variables, pin definitions, constants, and the `setup()` / `loop()` functions. All other tabs contain the implementations.

| File | Responsibility |
|------|----------------|
| `SCARA_ROBOT_ARM_CODE.ino` | Shared state, pin config, constants, `setup()`, `loop()` |
| `01_DriverControl.ino` | DRV8825 enable/disable, emergency stop, setup-required mode |
| `02_CalibrationAndStorage.ino` | NVS calibration read/write, motion profiles, TCP length, setup jogging |
| `03_JointCoordinates.ino` | Step-to-angle and angle-to-step conversions for all three axes |
| `04_ForwardKinematics.ino` | FK solver — joint angles → XY end-effector position |
| `05_InverseKinematics.ino` | IK solver — XY target → joint angles, elbow-up/down selection |
| `06_MotionAndSafety.ino` | Move queuing, collision path checking, auto-safe routing, limit monitoring |
| `07_HomingSequence.ino` | Full collision-safe homing sequence for Z, A2, and A1 |
| `08_WebApi.ino` | HTTP handler functions — every web button and slider maps to a handler here |
| `09_SerialDebug.ino` | Serial monitor status printing and command parsing |
| `10_GripperControl.ino` | Servo PWM generation, gripper calibration, open/hold/close commands |
| `11_CollisionSafety.ino` | Z-layered collision envelope storage, interpolation, and path/pose checking |
| `12_PackageRecipes.ino` | Recipe storage (LittleFS), step execution state machine, playback control |
| `13_ArmJobController.ino` | Autonomous job state machine layered above the recipe system |
| `webpage.h` | Full web interface HTML/CSS/JS served directly from flash |

---

## Coordinate System and Kinematics

**Forward Kinematics** (`04_ForwardKinematics.ino`)

Given joint angles A1 and A2 in degrees and the configured TCP length L2:

```
x = L1 * cos(A1) + L2 * cos(A1 + A2)
y = L1 * sin(A1) + L2 * sin(A1 + A2)
```

L1 is the upper arm length (fixed constant in the main config). L2 is the TCP length stored in NVS and configurable from the web interface.

**Inverse Kinematics** (`05_InverseKinematics.ino`)

Given a target XY position, the IK solver computes both elbow-up (A2 < 0) and elbow-down (A2 > 0) solutions using the standard two-link planar IK equations. Each solution is validated against the calibrated joint limits. If both solutions are valid, the one closest to the current joint posture is selected to minimize motion. The IK result is verified by running FK on the solution and checking the position error is within tolerance.

**Step ↔ Angle Conversion** (`03_JointCoordinates.ino`)

All motion targets are stored and commanded in raw step counts. Angles in degrees are only a display and input format. The conversions account for the different homing directions of each axis — A2 in particular moves in the negative raw direction for positive angles.

---

## Motion Pipeline

A move request from the web interface goes through these stages:

1. **State validation** — Is the robot homed? Is a fault active? Is a recipe or job running?
2. **Joint limit check** — Are the requested angles within the calibrated safe range?
3. **Collision pose check** — Does the target pose violate any active Z-layer collision envelope?
4. **Collision path check** — Does the direct path between current and target pose sweep through a forbidden region?
5. **Route selection** — If the direct path is safe, move directly. If not, route through the saved A2 corridor (a three-phase detour that brings A2 to a safe transit angle before rotating A1, then reaches the final target).
6. **Motor command** — `arm1.moveTo()`, `arm2.moveTo()`, `zAxis.moveTo()` are called with converted step targets. AccelStepper handles acceleration and velocity.

The `loop()` function calls `runMotionServiceSlice()` every iteration. This function runs the stepper step pulses in time-sliced blocks — longer slices for real-time profile moves, shorter for point-to-point — so the ESP32 can also service the web server between motion steps.

---

## Collision Safety System

(`11_CollisionSafety.ino`)

Collision protection is implemented as a set of Z-layered envelope layers. Each layer defines:

- An A1 side (positive or negative A1 region)
- An A2 boundary direction (A2 must stay below or above the limit)
- A Z range where the layer is active
- A list of (A1, A2 limit) boundary points that are interpolated at runtime

When a move is requested, the firmware checks both the target pose and the entire direct path (sweeping A1 and Z ranges) against all active layers. If any layer is violated, the firmware either rejects the move or automatically routes through the saved A2 corridor.

The collision envelope is stored in NVS under `scara_env3`. Envelope data is saved in raw step coordinates so it remains physically aligned if display calibration is later adjusted.

---

## Recipe System

(`12_PackageRecipes.ino`)

Recipes are sequences of named steps stored in LittleFS as `/scara_recipes.dat`. Each step contains a command type, target joint positions (in raw steps), a gripper angle, and a wait duration.

**Step types:**

| Command | Behavior |
|---------|----------|
| `MOVE_SAFE_XYZ` | Full collision-safe move through `queueJointMove()` |
| `MOVE_A1` | Moves only A1, other axes hold |
| `MOVE_A2` | Moves only A2 |
| `MOVE_Z` | Moves only Z |
| `MOVE_A1_A2` | Moves A1 and A2 together, Z fixed |
| `MOVE_XYZ_DIRECT` | Direct joint move, no auto-reroute |
| `GRIPPER` | Commands the servo only |
| `WAIT` | Pause for a set duration |

The recipe playback state machine runs in `servicePackageRecipe()`, called from `loop()`. After each movement step completes, the stored gripper angle is commanded, then the step's wait timer runs before advancing to the next step.

Recipes are saved atomically: the firmware writes to a temporary file, verifies the checksum, then renames it over the previous file. If anything fails, the old file is preserved.

---

## Job Controller

(`13_ArmJobController.ino`)

The job controller is a state machine layered above the recipe system. It models a complete package-handling workflow:

```
NOT_HOMED → READY → JOB_LOADED → WAITING_FOR_AMR → RUNNING → DONE / FAULT
```

A job associates a recipe with a job ID and package class. Once loaded, the arm waits for AMR (autonomous mobile robot) arrival confirmation before starting the recipe. The web interface provides buttons to simulate each stage for manual testing. Future integration will replace these buttons with MQTT or sensor signals.

---

## Storage

Three NVS namespaces and one LittleFS file are used:

| Store | Namespace / Path | Contents |
|-------|-----------------|----------|
| Calibration | `scara_raw_v2` | Axis zeros and raw limits |
| Motion profiles | `scara_motion` | Speed percentages per profile |
| Tool settings | `scara_tool` | TCP length |
| Gripper | `scara_grip` | Open, hold, and max-close angles |
| Collision | `scara_env3` | Envelope layers, corridor, park pose |
| Recipes | `/scara_recipes.dat` | Full recipe library (LittleFS) |

---

## Web API Summary

All endpoints are HTTP GET with URL query parameters. The ESP32 responds with JSON.

| Endpoint | Description |
|----------|-------------|
| `/position` | Full robot state — position, limits, speeds, recipe/job status |
| `/home` | Trigger homing sequence |
| `/move` | Joint-space move (FK profile) |
| `/moveRT` | Joint-space move (real-time profile) |
| `/ik` | Cartesian IK move |
| `/stop` | Emergency stop |
| `/off` | Disable motors |
| `/config` | Full configuration dump including calibration and recipes |
| `/calibration/jog` | Setup jog for a single axis |
| `/calibration/setZero` | Save geometric zero for an axis |
| `/calibration/setLimit` | Save a raw endpoint limit |
| `/calibration/export` | Download calibration JSON backup |
| `/calibration/import` | Restore calibration from JSON |
| `/gripper/*` | Gripper open, hold, move, calibration |
| `/recipe/*` | Recipe create, edit, play, save, export, import |
| `/job/*` | Job load, arm, confirm AMR, abort, clear |
| `/collision/*` | Envelope layer and point management |
