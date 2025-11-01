#  Self-Balancing Bicycle Platform  
**Hardware and Control System Design Based on STM32 and FreeRTOS**  

---

##  Overview

This project presents a **two-wheeled self-balancing bicycle platform** that integrates **embedded hardware design**, **real-time control algorithms**, and **upper-computer visualization software**.  
The system utilizes **an STM32 microcontroller**, **MPU6050 IMU sensor**, and **dual motor control** with a **momentum wheel and steering servo**, achieving balance through **cascaded PID control** and **FreeRTOS task scheduling**.

The platform is designed for research and educational purposes, demonstrating practical implementation of control theory, sensor fusion, and embedded systems in robotics.


![IMG_3757](https://github.com/user-attachments/assets/6efc8853-2551-4b69-b04f-ce2d2f71bf7f)

---

##  System Features

-  **Cascaded PID Control**  
  Combines angular velocity and angle feedback for stable balance control.

-  **Quaternion-Based Attitude Estimation**  
  Uses complementary filtering and MPU6050 data fusion to determine pitch and roll.

-  **FreeRTOS Task Management**  
  Multitasking structure for sensor acquisition, control loop, and communication threads.

-  **Bluetooth Communication**  
  Supports remote parameter tuning, data transmission, and control commands.

-  **Upper-Computer Visualization**  
  Python-based GUI for real-time data plotting, PID tuning, and system monitoring.

-  **Compact Hardware Platform**  
  Custom PCB with integrated STM32F103C8T6, voltage regulator circuit, bluetooth module,and OLED display.

---

##  System Architecture


<img width="1427" height="1292" alt="值 (2)" src="https://github.com/user-attachments/assets/df421de1-d981-494d-b9de-6238656d0981" />

---

## Hardware Design

The **Self-Balancing Bicycle Platform** was developed as a compact embedded system integrating sensing, computation, and control.  
It is based on an **STM32F103C8T6** microcontroller and incorporates multiple modules to achieve real-time dynamic balance and steering stabilization.

| Component | Description |
|------------|-------------|
| **Microcontroller** | STM32F103C8T6 (Cortex-M3, 72 MHz, 64 KB Flash) |
| **IMU Sensor** | MPU6050, combining a 3-axis accelerometer and a 3-axis gyroscope |
| **Actuators** | EA90 three-phase brushless motors for drive and balance control , and a servo motor for steering |
| **Display** | 0.96" OLED (I²C interface) showing pitch angle, PWM output, and status |
| **Communication** | JDY-31 Bluetooth module for remote data monitoring and parameter adjustment |
| **Power Supply** | 7.4V lithium-ion battery with AMS1117 voltage regulation |
| **PCB Design** | Custom two-layer PCB designed in **Altium Designer** |


---

### Hardware Implementation Highlights

- The STM32 microcontroller handles all real-time tasks under the FreeRTOS environment.  
- The MPU6050 provides raw motion data for attitude estimation.  
- A momentum wheel generates compensating torque to maintain upright balance.  
- The servo motor adjusts steering for horizontal stabilization.  
- The PCB layout minimizes interference between power and signal lines using a ground plane and separated analog/digital regions.  
- All modules were validated through bench testing before system integration.

<img width="637" height="358" alt="值" src="https://github.com/user-attachments/assets/91628191-30bc-4c03-ae37-cad69c565eb1" />

![IMG_3851](https://github.com/user-attachments/assets/c15c9c56-d97a-486d-b01c-7bb8fcf48989)

*Custom two-layer PCB for the control system.*

---

##  Control Algorithm

The control system applies a **cascaded PID controller** that stabilizes both angular position and angular velocity of the bicycle.  
A quaternion-based complementary filter processes the IMU data to obtain accurate pitch and roll angles.

### Algorithm Overview

- The **outer PID loop** regulates the tilt angle (pitch) to maintain balance.  
  It outputs a desired angular velocity for the inner loop.  
- The **inner PID loop** stabilizes angular velocity by adjusting motor torque through PWM control.  
- Both loops run in real time under **FreeRTOS** tasks with a 2 ms sampling period.  
- Attitude estimation is derived from **gyro and accelerometer fusion**, where 98% weight is assigned to the gyroscope to minimize noise and drift.

<img width="1261" height="391" alt="值 (1)" src="https://github.com/user-attachments/assets/e7a5c6a9-6e7d-43ec-be9c-fa7d5c348a7d" />


### PID Tuning and Parameters

Parameters were experimentally tuned to balance stability and responsiveness.  
Step response analysis was used to refine each loop’s gain values.

| Loop | Kp | Ki | Kd | Sampling Time |
|------|----|----|----|----------------|
| **Outer (Angle)** | 90 | 0.5 | 3 | 2 ms |
| **Inner (Velocity)** | 25 | 1.2 | 0.1 | 2 ms |

The final configuration achieved fast convergence with minimal overshoot.  
Under disturbance tests, the bicycle restored its balance within 0.3 seconds after small perturbations.

---

##  Experimental Results

### Performance Evaluation
- The platform maintained stable balance with **±3° maximum pitch deviation**.  
- The response time to external disturbances was **less than 300 ms**.  
- Angular drift was effectively suppressed through quaternion fusion.  
- The system demonstrated consistent stability during extended operation (>30 minutes).  

### Observations
- The complementary filter provided smoother attitude estimation compared to raw gyroscope data.  
- The cascaded PID structure improved dynamic response over single-loop control.  
- Thermal effects on sensor bias were mitigated through initial calibration at power-up.
  

![IMG_3786](https://github.com/user-attachments/assets/deb9adaa-fa2c-4595-8d4d-5024355f2477)

*Prototype of the self-balancing bicycle during testing.*

![IMG_3762](https://github.com/user-attachments/assets/616ddfa1-9ad9-4902-a682-c4ce3ebdfd8d)
*Self-balancing bicycle control system platform.*

![IMG_3787](https://github.com/user-attachments/assets/fd1a26ea-b11b-4642-b701-806b2b643135)
*Momentum wheel module.*

![IMG_3763](https://github.com/user-attachments/assets/e479775a-121c-4882-9d40-3d5559eea375)
*Rear wheel drive module.*

---
##  Author

**Jayson Chen**  
Master’s in Audio and Music Technology  
University of York  

Contact: *[jaysonchen816@gmail.com]*  
