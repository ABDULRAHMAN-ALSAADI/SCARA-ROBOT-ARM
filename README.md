# SCARA Robot Arm Controller

![SCARA Robot](docs/images/scara-overview.jpg)

A modular SCARA robot arm control system built on ESP32 featuring Forward Kinematics, Inverse Kinematics, Motion Control, Recipe Execution, Persistent Storage, and a Web-Based User Interface.

The latest stable version of the project is located in:

```text
V10_22/
```

This repository is intended for:

- Robotics students
- Embedded systems developers
- Makers and hobbyists
- Researchers
- Anyone building a custom SCARA robot

---

# Features

## Motion Control

- Coordinated multi-axis movement
- Acceleration control
- Homing system
- Stepper driver integration

## Kinematics

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Cartesian positioning
- Joint-space calculations

## Recipe System

- Save robot operations
- Execute predefined sequences
- Repeatable automation tasks

## Data Storage

- LittleFS integration
- Persistent settings
- Recipe storage

## Communication

- WebSocket communication
- Command processing
- Real-time robot status updates

---

# Software Architecture

![Architecture](docs/images/software-architecture.png)

```text
Web Interface
      │
      ▼
ESP32 Main Controller

 ├── Motion Control
 ├── Kinematics
 ├── Recipe Manager
 ├── State Manager
 ├── Data Storage (LittleFS)
 └── Communication Layer

      │
      ▼

Stepper Drivers + Sensors
```

---

# Web Interface

![Web Interface](docs/images/web-interface.png)

The web interface allows:

- Manual control
- Cartesian positioning
- Joint positioning
- Recipe creation
- Recipe execution
- System monitoring

---

# Hardware

## Controller

- ESP32

## Actuators

- Stepper Motors
- Stepper Drivers

## Sensors

- Limit Switches
- Homing Sensors

---

# Getting Started

Read the following documentation in order:

1. BUILD_GUIDE.md
2. CALIBRATION_GUIDE.md
3. SOFTWARE_ARCHITECTURE.md
4. KINEMATICS.md

---

# Configuration

Before using the software on your own robot, update:

- Link lengths
- Motor directions
- Steps per revolution
- Joint limits
- Homing parameters
- WiFi credentials

Detailed instructions are available in:

```text
docs/CALIBRATION_GUIDE.md
```

---

# Project Structure

```text
V10_22/

├── MotionControl
├── Kinematics
├── RecipeManager
├── StateManager
├── Communication
├── Storage
├── WebInterface
├── Config
└── Main
```

---

# Building Your Own Robot

If you print or redesign the robot:

1. Assemble mechanics
2. Install motors
3. Install sensors
4. Configure dimensions
5. Calibrate robot
6. Verify FK
7. Verify IK
8. Test motion system

---

# Documentation

| File | Description |
|--------|--------|
| BUILD_GUIDE.md | Hardware setup |
| CALIBRATION_GUIDE.md | Robot calibration |
| SOFTWARE_ARCHITECTURE.md | Code architecture |
| KINEMATICS.md | FK and IK explanation |
| TROUBLESHOOTING.md | Common issues |
| CONTRIBUTING.md | Contribution guide |

---

# Future Improvements

- ROS2 integration
- Vision system support
- Advanced trajectory planning
- Closed-loop control
- Collision avoidance

---

# License

MIT License
