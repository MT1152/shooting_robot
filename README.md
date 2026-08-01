# Humanoid Robot Ball Detection and Kicking System

## Overview

This project implements a vision-guided humanoid robot capable of detecting a white ball, estimating its distance using computer vision, walking toward it, and performing a kick using inverse kinematics.

The system consists of two main components:

- **Python (OpenCV)** – Detects the white ball, estimates distance, and communicates with the robot through serial communication.
- **Arduino** – Controls six servo motors, executes the walking gait, and performs an inverse-kinematics-based kicking motion.

---

## System Architecture

```
USB Camera
      │
      ▼
Python + OpenCV
      │
 Detect White Ball
 Estimate Distance
      │
 Serial Commands
      │
      ▼
Arduino Mega/Uno
      │
Walking Controller
Inverse Kinematics
Servo Control
      │
      ▼
Humanoid Robot
```

---

## Features

- Real-time white ball detection using HSV color segmentation
- Distance estimation using the pinhole camera model
- Noise removal with morphological filtering
- Automatic walking initiation after stable detection
- Real-time serial communication between Python and Arduino
- Six-servo humanoid leg control
- Inverse and forward kinematics implementation
- Smooth servo motion using cosine interpolation
- Walking state machine
- Automatic kicking once the robot reaches the target distance

---
# Hardware Requirements

- Arduino Uno 
- 6 Servo Motors
- USB Camera
- Humanoid robot chassi
- USB cable
- Computer running Python

---


# Camera Calibration

The distance is estimated using the pinhole camera model

\[
Distance = \frac{Real\ Diameter \times Focal\ Length}{Object\ Width}
\]

Current parameters:

| Parameter | Value |
|-----------|------:|
| Ball Diameter | 6.7 cm |
| Focal Length | 621.4 pixels |
| Safe Kick Distance | 17 cm |

These values should be recalibrated for different cameras.

---

# Computer Vision Pipeline

1. Capture camera frame
2. Resize image
3. Convert BGR → HSV
4. Threshold white color
5. Morphological opening
6. Morphological closing
7. Detect contours
8. Select largest valid contour
9. Estimate distance
10. Send command to Arduino

---

# Robot Behavior

1. Detect white ball
2. Verify detection over multiple frames
3. Begin walking
4. Continuously receive distance updates
5. Stop at safe kicking distance
6. Execute kick
7. Return to neutral posture

---

# Walking Cycle

The robot performs an 8-phase walking gait:

1. Shift weight right
2. Move left leg forward
3. Center weight
4. Return left leg
5. Shift weight left
6. Move right leg forward
7. Center weight
8. Return right leg

---

# Inverse Kinematics

Leg dimensions:

| Link | Length |
|------|--------|
| Hip → Knee | 6.0 cm |
| Knee → Ankle | 5.5 cm |

Maximum reach:

```
11.5 cm
```

The inverse kinematics solver computes the required hip and knee angles using the law of cosines and validates the solution with forward kinematics before executing movement.

---

# Servo Configuration

| Joint | Pin |
|--------|----:|
| Left Hip | 10 |
| Left Knee | 9 |
| Left Ankle | 6 |
| Right Hip | 3 |
| Right Knee | 11 |
| Right Ankle | 5 |

Neutral position:

```
90°
```

---

