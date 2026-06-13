<img width="1122" height="1402" alt="ChatGPT Image Jun 13, 2026, 09_01_35 PM" src="https://github.com/user-attachments/assets/6d13665c-fd13-4664-ac48-65e8bd2c5aa5" /># SCARA Robot Arm Controller
<img width="6600" height="5100" alt="B2 Z-AXIS ARM BASE ASSEMBLY" src="https://github.com/user-attachments/assets/6d757b25-525f-4041-a2ba-22e4e828ae29" />


A modular SCARA robot arm control system built on ESP32 featuring Forward Kinematics, Inverse Kinematics, Motion Control, Recipe Execution, Persistent Storage, and a Web-Based User Interface.

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

<img width="1122" height="1402" alt="ARc" src="https://github.com/user-attachments/assets/da55f20c-abe2-46da-9c35-706a7649c201" />


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
<div class="image-grid">
 <img width="1554" height="1580" alt="192 168 137 30_" src="https://github.com/user-attachments/assets/0a381091-2a29-4056-8bba-7c5e4e1af6be" />
 <img width="1554" height="1580" alt="192 168 137 30_ (1)" src="https://github.com/user-attachments/assets/38e8ac0a-8b31-4991-8c3c-decfc5fd5bed" />
 <img width="1554" height="1716" alt="192 168 137 30_ (2)" src="https://github.com/user-attachments/assets/a9e763d2-556a-4588-97bf-74bf61977059" />
 <img width="1554" height="2979" alt="192 168 137 30_ (3)" src="https://github.com/user-attachments/assets/479ba439-d6ee-4a3c-9fae-f91f80972248" />

</div>



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
