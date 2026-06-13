# Kinematics

## Overview

The SCARA robot uses a 2-DOF planar arm configuration.

The kinematics module is responsible for converting:

* Joint Angles → Cartesian Position (Forward Kinematics)
* Cartesian Position → Joint Angles (Inverse Kinematics)

---

## Robot Geometry

![Robot Geometry](images/scara-geometry.png)

Definitions:

```text
L1 = Length of Link 1
L2 = Length of Link 2

θ1 = Joint 1 Angle
θ2 = Joint 2 Angle

X = End Effector X Position
Y = End Effector Y Position
```

---

## Forward Kinematics

Forward Kinematics calculates the end-effector position from joint angles.

Equations:

X Position:

```math
X = L1*cos(θ1) + L2*cos(θ1 + θ2)
```

Y Position:

```math
Y = L1*sin(θ1) + L2*sin(θ1 + θ2)
```

---

### Code Location

```text
V10_22/
└── Kinematics/
    └── ForwardKinematics.cpp
```

Example:

```cpp
float x =
    L1 * cos(theta1) +
    L2 * cos(theta1 + theta2);

float y =
    L1 * sin(theta1) +
    L2 * sin(theta1 + theta2);
```

---

## Inverse Kinematics

Inverse Kinematics calculates the required joint angles to reach a target coordinate.

Inputs:

```text
X
Y
```

Outputs:

```text
θ1
θ2
```

---

### Step 1

Calculate distance from origin.

```math
r = sqrt(X² + Y²)
```

---

### Step 2

Calculate θ2.

```math
cos(θ2) =
(X² + Y² - L1² - L2²)
/ (2*L1*L2)
```

```math
θ2 = acos(cos(θ2))
```

---

### Step 3

Calculate θ1.

```math
θ1 =
atan2(Y,X)
-
atan2(
L2*sin(θ2),
L1+L2*cos(θ2)
)
```

---

### Code Location

```text
V10_22/
└── Kinematics/
    └── InverseKinematics.cpp
```

Example:

```cpp
float c2 =
(x*x + y*y - L1*L1 - L2*L2)
/
(2.0f * L1 * L2);

float theta2 = acos(c2);

float theta1 =
atan2(y, x)
-
atan2(
L2 * sin(theta2),
L1 + L2 * cos(theta2)
);
```

---

## Reachable Workspace

The maximum reach is:

```math
Rmax = L1 + L2
```

The minimum reach is:

```math
Rmin = |L1 - L2|
```

Any target outside this range cannot be reached.

---

## Configuration

Robot dimensions are defined in:

```text
Config/RobotConfig.h
```

Example:

```cpp
#define LINK1_LENGTH 200.0f
#define LINK2_LENGTH 180.0f
```

Update these values if you redesign the robot.

---

## Common Issues

### Robot misses target

Possible causes:

* Incorrect link lengths
* Wrong step calibration
* Mechanical backlash

---

### IK returns invalid values

Possible causes:

* Target outside workspace
* Numerical errors
* Incorrect geometry configuration

---

## References

Introduction to Robotics:
Mechanics and Control

John J. Craig

Robot Modeling and Control

Spong, Hutchinson, Vidyasagar
