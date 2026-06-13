# Calibration Guide — SCARA v10.22

Calibration teaches the firmware where the arm physically is in space. All physical limits are stored as raw stepper step counts from the home switch — not as degree values. Degree limits are derived from the stored zeros and raw endpoints each time calibration is loaded. This means recalibrating a zero does not corrupt your saved endpoints, and endpoints saved under one geometry remain physically correct if you move the zero.

---

## Before You Start

The robot must be homed before any calibration can be saved. Run HOME from the web interface before opening the Setup panel. Each HOME attempt requires confirming the Safe Startup Pose first — place the arm in the saved park position and press **Confirm Safe Startup Pose** on the web page.

---

## Coordinate System

| Axis | Home Switch | Direction |
|------|-------------|-----------|
| A1 | Negative end (switch = raw 0) | Away from switch = increasing raw steps |
| A2 | Negative end (switch = raw 0) | Away from switch = decreasing raw steps |
| Z | Top switch (switch = raw 0) | Downward = increasing raw steps |

A1 positive angles and A2 positive angles both move away from their respective switches. Z positive millimeters move downward from the top position.

---

## Calibration Parameters

### Geometric Zero (per axis)

The geometric zero is the raw step position at which the axis reads 0°/0 mm. After homing, use the Setup jog controls to move each axis to its physical zero position and save it.

**A1 zero** — The position where A1 reads 0°. Typically this is the arm pointing forward along the robot's baseline.

**A2 zero** — The position where A2 reads 0°. This is the position where the forearm is straight relative to the upper arm.

**Z zero** — The height you define as Z = 0 mm in your workspace. Usually this is the working surface or a convenient reference height.

To save a zero: jog the axis to the desired position in Setup, then click **Set Zero** for that axis. The firmware saves the raw step count at that position.

### Joint Endpoints

Endpoints define the safe operating range for each axis. They must be set with the arm homed and the axis referenced.

**A1 negative limit** — The minimum A1 angle. Must maintain a minimum raw clearance from the home switch. Set this by jogging A1 to its safe negative extreme and pressing **Set Negative Limit**.

**A1 positive limit** — The maximum A1 angle. Must be beyond the negative limit. Set by jogging to the safe positive extreme and pressing **Set Positive Limit**.

**A2 negative and positive limits** — Same process. A2 moves in the negative raw direction for positive angles, so the firmware validates limits in raw coordinates internally. The web interface shows angles in degrees.

**Z upper limit** — The highest working position (closest to the home switch). Must maintain minimum clearance from the top switch.

**Z lower limit** — The lowest working position. Must be below the upper limit. Set this to just above where the gripper would contact the table or fixture.

### A2 Home-Clearance Park Pose

This is the A2 angle the arm parks at during A1 homing. The gripper must be clear of the robot base and column in this position while A1 sweeps to its home switch.

Set it by jogging A2 to a safe folded position (typically a more negative angle than zero, with the forearm rotated inward) and pressing **Set A2 HOME Clearance Park**. The value must be on the positive-clearance side of A2 zero and within the calibrated A2 range.

### Tool Center Point (TCP)

The TCP length is the distance from the A2 pivot to the tool tip. It is used in both FK and IK calculations.

Set it in millimeters using the **TCP Length** field. The allowed range is 20 to 250 mm. Default is 120 mm. This value is saved separately from axis calibration and persists across reboots.

### Gripper Calibration

Three servo positions define the gripper range:

- **Open** — The fully open position (servo angle). Default 0°.
- **Hold** — The box-hold position used during recipe execution. Default 70°.
- **Max Close** — The furthest the gripper is allowed to close. Hard limit is 120°.

The constraint is Open ≤ Hold ≤ Max Close. Calibrate by commanding the gripper to each position using the slider, then pressing **Save as Open / Hold / Max Close** in the Gripper panel.

---

## Homing Sequence

The HOME sequence is collision-safe and runs in a fixed order:

1. Z homes first — drives up to the top switch, then backs off a fixed raw release distance.
2. A2 homes — drives to its switch, releases, then moves to the calibrated geometric zero and parks at the A2 clearance position.
3. A1 homes — only after A2 is parked clear of the base column.
4. A1 returns to calibrated geometric zero. A2 comes out of park and returns to geometric zero.
5. Both rotary axes move to the HOME finish pose (A1 = 130°, A2 = −136° by default).

If calibration is missing or invalid at any stage, the firmware enters Setup Required mode rather than triggering an emergency stop. In Setup Required, you can jog referenced axes and save corrected values, then HOME again.

---

## Motion Speed Profiles

Three speed profiles control how fast the arm moves in different modes:

- **Manual / FK** — Used for manual jogging and FK moves from the web interface. Default 60%.
- **Real-Time** — Used for streaming real-time position commands. Default 35%.
- **IK** — Used for Cartesian IK moves. Default 60%.

Adjust them with the sliders in the Motion panel and save with **Save Speed Profiles**. Homing always runs at locked full speed regardless of these settings.

---

## Saving and Backing Up Calibration

All calibration is saved automatically when you press **Set Zero**, **Set Limit**, or **Save TCP**. It is stored in ESP32 NVS (non-volatile storage) under the `scara_raw_v2` namespace and survives power cycles.

Use **Export Calibration** in the web interface to download a JSON backup of all calibration values including gripper settings, motion profiles, collision envelope, and the startup park pose. Use **Import Calibration** to restore from a backup. After importing, HOME is required before normal operation.

---

## Safe Startup Pose and Power-Off Park

The Safe Startup Pose is a saved arm position you physically place the arm in before each HOME. It gives the firmware a known starting geometry so the collision-safe homing path is predictable.

To set it: HOME the arm, move it to a safe folded position with the gripper open, and press **Save Startup / Power-Off Park Pose**. Before powering off, use **Park for Power-Off** to move the arm to this position automatically, then cut power once motion stops.

Before each HOME, physically verify the arm matches the saved pose and press **Confirm Safe Startup Pose** in the web interface.

---

## Serial Monitor Commands

The serial monitor (115200 baud) accepts basic commands for headless operation:

```
CONFIRM_STARTUP   — Confirms the startup pose for one HOME attempt
HOME              — Runs the full home sequence
MOVE a1 a2 z      — Queues a joint-space move (degrees, degrees, mm)
WHERE             — Prints current position, state, and fault message
STOP              — Emergency stop
OFF               — Identical to STOP
```
