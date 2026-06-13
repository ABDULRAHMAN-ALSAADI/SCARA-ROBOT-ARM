# Calibration Guide

## Configure Robot Dimensions

Update:

```cpp
L1
L2
```

These values must match your physical robot.

---

## Configure Motor Parameters

Update:

```cpp
STEPS_PER_REV
MICROSTEPPING
```

---

## Configure Joint Limits

```cpp
JOINT1_MIN
JOINT1_MAX

JOINT2_MIN
JOINT2_MAX
```

---

## Configure Homing

```cpp
HOME_SPEED
HOME_DIRECTION
```

---

## Calibration Procedure

1. Power robot.
2. Execute homing.
3. Verify Joint 1 direction.
4. Verify Joint 2 direction.
5. Move 10 mm.
6. Measure actual movement.
7. Correct parameters if necessary.
8. Repeat until accurate.
