# Self-Balancing-Electric-Vehicle-
Self-Balancing Electric Vehicle
# Self-Balancing-Electric-Vehicle-
Self-Balancing Electric Vehicle
# Self-Balancing Robot

This repository contains the control software for a **two-wheeled self-balancing robot** based on an inverted pendulum model.  
The system integrates inertial sensing, real-time embedded control, and motor actuation to maintain upright stability.

---

## Project Overview

The robot uses an **IMU sensor (BNO055)** to measure orientation and angular velocity.  
Sensor data is processed in real time to estimate the robot’s tilt angle, which is then used by the control algorithm to generate motor commands that stabilize the platform.

The code is organized into multiple modules responsible for:
- Sensor data processing and filtering  
- Balance control and yaw control  
- Motor pulse counting and timing interrupts  
- High-frequency control loop execution  

---

## Code Structure

Key files and their roles include:

- `setup.ino`  
  Initializes sensors, motors, interrupts, and system parameters.

- `loop.ino / Mainloop.ino`  
  Main runtime loop coordinating balance control and motion behavior.

- `Balance.cpp / Balance.h`  
  Core balance control logic for the inverted pendulum system.

- `KalmanFilter.cpp / KalmanFilter.h`  
  Sensor fusion and filtering for tilt angle estimation.

- `yaw_control.ino`  
  Yaw and turning control logic.

- `interrupt_5ms.ino / interrupts.cpp / interrupts.h`  
  Timer-based interrupt routines for real-time control execution.

- `countpulse.ino`  
  Encoder pulse counting for motor speed and position feedback.

- `move_distance.ino`  
  Motion and distance-related control logic.

---

## Authorship and Attribution

**Original Author:**  
Dr. R. Bauer  
Dalhousie University  
February 24, 2024  

This project is **based on instructional and reference code** provided by the course instructor and adapted from the Adeept Self-Balancing Robot example framework.

**Modifications and Integration:**  
Modified, extended, and integrated by **Aeron HAN** as part of an academic engineering project.  
Work includes system integration, parameter tuning, control logic refinement, and experimental testing to achieve stable self-balancing behavior.

---

## Operation Summary

1. Hold the robot upright.
2. Power on the robot using the rear switch.
3. Wait for the buzzer signal.
4. Calibrate the BNO055 sensor by rotating the robot in the air until LEDs turn off.
5. Place the robot on level ground within 5 seconds to initiate balancing.

---

## Notes

This repository is intended for **educational and portfolio demonstration purposes**, focusing on system integration, embedded control understanding, and electromechanical design rather than from-scratch controller derivation.
