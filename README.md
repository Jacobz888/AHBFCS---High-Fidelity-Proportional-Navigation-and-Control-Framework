#  AHBFCS: Autonomous High-Bandwidth Flight Control System

##  Overview: The Pursuit of Optimized Intercept Trajectories

The Autonomous High-Bandwidth Flight Control System (AHBFCS) is a theoretical,  embedded software framework designed for the rigorous study and implementation of advanced control principles, specifically focusing on the realization of a robust, real-time control loop executing the **Proportional Navigation (PN)** guidance law, like the AIM-9 Sidewinder or similar military project.

This project serves as an intellectual laboratory for researchers and students interested in:
* **Sensor Fusion:** Combining high-rate IMU data with passive optical/IR tracking signals.
* **Real-Time Kinematics:** Implementing fixed-rate, low-latency control loops on microcontrollers.
* **Classical Control Theory:** Deep application of the Kalman filter and PID control structures.

> **DISCLAIMER:** This repository contains only conceptual pseudo-code and architecture blueprints for academic discussion. It is **not** intended for, and must not be used in, the construction, deployment, or operation of any physical device capable of causing harm or injury. The project maintains a strict policy against use in any weaponized context.

---

## ⚙️ Architectural Blueprint: The 5-Stage G&C Pipeline

The G&C system is architected as a sequential, deterministic pipeline running on a fixed-rate interrupt timer (targeting $200\text{ Hz}$ to $400\text{ Hz}$). This ensures control stability and responsiveness. 


| Stage | Function | Inputs | Output | Key Algorithm |
| :--- | :--- | :--- | :--- | :--- |
| **I: Data Acquisition** | Reads raw sensor voltage/counts. | IMU (Accel/Gyro/Mag), Seeker Quadrant Signals, Barometer. | Raw Digital Readings | Sampling Theory, Synchronization |
| **II: State Estimation** | Fuses noisy data into an optimal state vector. | Raw Readings, Previous State Vector. | Filtered State Vector (Position, Velocity, Attitude, Rates). | **Extended Kalman Filter (EKF)** |
| **III: Guidance** | Calculates the required lateral acceleration to null the LOS rate. | Filtered State Vector, Line-of-Sight Rate ($\dot{\lambda}$). | Acceleration Command ($A_c$: Pitch/Yaw). | **Proportional Navigation (PN) Law** |
| **IV: Control** | Translates the acceleration command into physical torque/deflection. | Acceleration Command, Current Angular Rates. | Torque/Deflection Command ($\delta$). | **Nested PID Control Loops** |
| **V: Actuation** | Converts digital commands into analog/PWM signals for motors. | Deflection Command ($\delta$). | Servo PWM Duty Cycles / DAC Voltage. | Signal Saturation, Servo Mapping |

---

## 🔬 Core Algorithms: Mathematics of Interception

The intellectual value of this project lies in the rigorous implementation of the following algorithms:

### 4.1. Proportional Navigation (PN)

PN is a lead-angle pursuit strategy. The commanded lateral acceleration ($A_c$) is perpendicular to the line-of-sight (LOS) and is proportional to the rate of rotation of the LOS ($\dot{\lambda}$).

The fundamental law is:
$$
A_c = N \cdot V \cdot \dot{\lambda}
$$

* $N$: **Effective Navigation Constant** (Tuned for intercept optimization, typically $N \in [3.0, 5.0]$).
* $V$: Velocity magnitude of the controlled vehicle.
* $\dot{\lambda}$: The **LOS angular rate**, calculated via numerical differentiation of the seeker's tracking error: $\frac{\Delta(\text{Error})}{\Delta t}$. This differentiation step is a critical source of noise and requires careful filtering.
<img width="206" height="197" alt="image" src="https://github.com/user-attachments/assets/4956a841-0d76-40de-8964-6272507296d2" />


### 4.2. Extended Kalman Filter (EKF) Implementation

Given the non-linear dynamics of a high-speed vehicle, an **Extended Kalman Filter (EKF)** is the required approach for high-fidelity state estimation. The EKF must be implemented efficiently on the constrained MCU environment. 

* **State Vector ($\mathbf{x}$):** Includes 9-15 elements, minimally covering Attitude (Quaternions), Angular Rates, Velocity, and Position.
* **Process Model ($\mathbf{f}$):** Uses the nonlinear dynamics model of the vehicle (including drag and thrust approximations) to predict the next state.
* **Measurement Model ($\mathbf{h}$):** Maps the predicted state to the expected sensor readings (IMU, Seeker).
<img width="237" height="212" alt="image" src="https://github.com/user-attachments/assets/4782870e-5233-495b-b328-178d9d5978aa" />


### 4.3. High-Rate PID Control

The Control Law translates the acceleration command ($A_c$) into the physical deflection of the control fins ($\delta$) using nested PID loops. 

* **Inner Loop (SAS):** Controls Pitch/Yaw/Roll **rates** ($p, q, r$). This provides immediate stability augmentation.
* **Outer Loop (G&C):** Controls Pitch/Yaw **acceleration** ($a_y, a_z$) to track the $A_c$ command from the PN law.


<img width="654" height="339" alt="image" src="https://github.com/user-attachments/assets/e30df48a-ee44-4c54-8263-f68b05176e9d" />



The PID structure:
$$
\text{Output} = K_P \cdot e(t) + K_I \int e(t) dt + K_D \frac{d}{dt} e(t)
$$

Where $e(t)$ is the error between the setpoint (command) and the measured value (e.g., $A_c - a_{measured}$).

---

## 🛠️ Dependencies and Setup (Conceptual Environment)

This project targets a high-performance, embedded platform capable of robust real-time operations.

### 5.1. Hardware Abstraction Layer (HAL)

* **Target MCU:** STM32H7 series or similar high-performance ARM Cortex-M7.
* **Toolchain:** GCC ARM Embedded, CMake for build system generation.
* **RTOS:** FreeRTOS or a bare-metal implementation to ensure maximum determinism and control over timing.


### 5.2. Core Library Requirements

* **CMSIS-DSP:** For high-speed matrix algebra necessary for the Kalman Filter (e.g., $9 \times 9$ covariance matrices).
* **Drivers:** Optimized I2C/SPI drivers for high-speed data acquisition from IMUs and ADCs.


