# Software Architecture

## Overview

The project follows a modular architecture.

<img width="1122" height="1402" alt="ARc" src="https://github.com/user-attachments/assets/da55f20c-abe2-46da-9c35-706a7649c201" />


---

## Motion Control

Responsible for:

- Step generation
- Speed control
- Acceleration control

---

## Kinematics

Responsible for:

- Forward Kinematics
- Inverse Kinematics
- Coordinate transformations

---

## Recipe Manager

Responsible for:

- Recipe creation
- Recipe storage
- Recipe execution

---

## State Manager

Responsible for:

- Robot modes
- Error states
- System status

---

## Data Storage

Responsible for:

- LittleFS operations
- Configuration persistence
- Recipe persistence

---

## Communication Layer

Responsible for:

- WebSocket communication
- Command routing
- Status reporting
