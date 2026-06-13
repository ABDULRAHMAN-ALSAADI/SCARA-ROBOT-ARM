# Software Architecture

## Overview

The project follows a modular architecture.

![Architecture](images/software-architecture.png)

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
