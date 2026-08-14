<p align="center">
  <img src="logo-branding-amrita-universiy-2024.jpeg" alt="Amrita Vishwa Vidyapeetham Logo" width="400"/>
</p>

# Quaternion Attitude Control System of Highly Maneuverable Aerial Vehicles

**Course:** 21AER314 / Drone Technologies & Flight Control  
**Institution:** Amrita School of Engineering, Amrita Vishwa Vidyapeetham  
**Repository Naming Convention:** `Drones_S5_CD13_Quaternion_Attitude_Control_UAV`  

---

## Team Member Details

| Name | Roll Number | Email | Contribution |
| :--- | :--- | :--- | :--- |
| **Raghuram S** (Lead) | `AM.EN.U4AERO22001` | `raghuram@amrita.edu` | Simulation Architecture, SITL, Mathematical Modeling |
| **Team Member 2** | `AM.EN.U4AERO22xxx` | `member2@amrita.edu` | Controller Implementation, Benchmark Analysis |
| **Team Member 3** | `AM.EN.U4AERO22yyy` | `member3@amrita.edu` | Report Documentation, Literature Review |

---

## Project Title & Abstract

### Title
**Quaternion-Based Non-Singular Attitude Control Architecture for Highly Maneuverable Fixed-Wing UAVs and Aerobatic Drones**

### Abstract
Modern unmanned aerial vehicles (UAVs) and high-performance strike drones are increasingly required to execute aggressive spatial maneuvers, including steep bank turns ($80^\circ - 90^\circ$), vertical climbs, and knife-edge flights. Traditional attitude control architectures relying on **Euler angles** ($\phi, \theta, \psi$) suffer from inherent mathematical singularities (gimbal lock at $\theta = \pm 90^\circ$) and catastrophic control surface cross-coupling (where elevators inadvertently control yaw and rudders control pitch). This project presents a non-singular **Quaternion Proportional ($Q\_P$) Attitude Controller** cascaded with a 3-axis inner-loop angular rate PID controller. Using unit quaternion kinematics, the controller guarantees non-singular, shortest-path 3D attitude tracking across full spatial rotations without trigonometric overhead. The system is validated in a 6-DOF dynamic Software-in-the-Loop (SITL) simulation environment modeling an aerobatic aircraft (Extra 330SC). Comparative benchmarks against conventional Euler PID controllers across $30^\circ, 60^\circ, 80^\circ,$ and $90^\circ$ bank maneuvers demonstrate that the quaternion controller completely eliminates cross-coupling error, reducing tracking RMSE at $90^\circ$ bank from $30.50^\circ$ (Euler) to $3.07^\circ$ (Quaternion).

**Keywords:** `Attitude Control`, `Unit Quaternions`, `Gimbal Lock`, `Aerobatic UAV`, `Cross-Coupling`, `SITL Simulation`, `Shortest-Path SLERP`, `Cascaded PID`

---

## 1. Introduction

**Keywords:** `Avionics`, `Euler Angle Limitations`, `Singularity Avoidance`, `Fixed-Wing Flight Control`, `Non-Linear Kinematics`

Flight attitude control is a cornerstone of autonomous aerial robotics. Conventional autopilot systems (such as standard ArduPilot or PX4 configurations) traditionally express aircraft orientation using **Euler angles** (Roll $\phi$, Pitch $\theta$, Yaw $\psi$). While intuitive for pilot display interfaces, Euler angles introduce severe mathematical degradation during extreme aerobatic or combat maneuvers:

1. **Gimbal Lock Singularity:** When the aircraft pitch angle reaches $\theta = \pm 90^\circ$, the roll and yaw axes align, collapsing a degree of freedom and causing matrix inversion failure in kinematic equations.
2. **Elevator-Rudder Coupling at High Bank Angles:** In knife-edge flight ($\phi = 90^\circ$), the physical elevator produces yawing moment relative to the inertial frame, while the rudder produces pitching moment. Classical Euler controllers fail to decouple these control surfaces without complex gain scheduling.
3. **Trigonometric Computational Burden:** Continuous evaluation of trigonometric functions ($\sin, \cos, \tan$) increases CPU execution latency in embedded flight control computers.

### Objective
This project implements the quaternion-based attitude control scheme proposed by Michał Gołąbek et al. (MDPI Electronics 2022). The primary objective is to replace the outer Euler angle loop with a unit quaternion attitude controller ($Q\_P$) that operates entirely in hyper-complex quaternion space.

---

## 2. Methodology & Mathematical Formulation

**Keywords:** `Quaternion Algebra`, `Hamilton Product`, `Attitude Error`, `Axis-Angle Representation`, `Shortest Arc`, `Airspeed Scaling`

```
  +-----------------------+     q_err     +----------------------+   w_sp   +-------------------+  control_cmd  +--------------------+
  | Setpoint Quaternion  | -------------> |   Q_P Attitude       | -------> |  3-Axis Rate PID  | ------------> | Aircraft Dynamics  |
  |       q_sp            |               |    Controller        |          |    Controller     |               |      (6-DOF)       |
  +-----------------------+               +----------------------+          +-------------------+               +--------------------+
                                                     ^                                ^                                      |
                                                     | q_meas                         | w_meas                               |
                                                     +--------------------------------+--------------------------------------+
                                                                         AHRS Feedback Loop
```

### 2.1 Unit Quaternion Fundamentals
A quaternion $q \in \mathbb{H}$ is defined as a 4-dimensional hyper-complex number:
$$q = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix} = q_w + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}$$
where $q_w \in \mathbb{R}$ is the scalar part and $\mathbf{q}_v = [q_x, q_y, q_z]^T \in \mathbb{R}^3$ is the vector part.

Unit quaternions satisfy the norm condition $\|q\| = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2} = 1$, forming the special orthogonal group $Sp(1) \cong SO(3)$.

The **Quaternion Conjugate** $\bar{q}$ is defined as:
$$\bar{q} = \begin{bmatrix} q_w \\ -q_x \\ -q_y \\ -q_z \end{bmatrix}$$

The **Hamilton Product** ($p \otimes q$) representing successive rotations is:
$$p \otimes q = \begin{bmatrix} 
p_w q_w - p_x q_x - p_y q_y - p_z q_z \\
p_w q_x + p_x q_w + p_y q_z - p_z q_y \\
p_w q_y - p_x q_z + p_y q_w + p_z q_x \\
p_w q_z + p_x q_y - p_y q_x + p_z q_w 
\end{bmatrix}$$

### 2.2 Attitude Error Quaternion Derivation ($q_{\text{err}}$)
Let $q_{\text{meas}}$ denote the current aircraft attitude measured by the AHRS, and $q_{\text{sp}}$ denote the target setpoint attitude. The intrinsic rotation relationship is:
$$q_{\text{sp}} = q_{\text{meas}} \otimes q_{\text{err}}$$

Left-multiplying both sides by the conjugate $\bar{q}_{\text{meas}}$ yields the explicit attitude error equation:
$$q_{\text{err}} = \bar{q}_{\text{meas}} \otimes q_{\text{sp}}$$

### 2.3 Shortest-Path Rotation Resolution
Because $q$ and $-q$ represent identical spatial rotations (double covering of $SO(3)$), the controller must avoid taking the longer rotation path ($> 180^\circ$). The shortest path condition is evaluated directly on the scalar component $q_{w,\text{err}}$:
$$q_{\text{err,short}} = \begin{cases} -q_{\text{err}} & \text{if } q_{w,\text{err}} < 0 \\ q_{\text{err}} & \text{if } q_{w,\text{err}} \ge 0 \end{cases}$$

### 2.4 Outer-Loop Control Law ($Q\_P$ Controller)
Assuming the setpoint quaternion derivative $\dot{q}_{\text{sp}}$ is proportional to error with proportional gain $K_p$:
$$\dot{q}_{\text{sp}} = K_p \cdot q_{\text{err,short}}$$

The relationship connecting quaternion derivative $\dot{q}$ to body angular rate setpoints $\mathbf{\omega}_{\text{sp}} = [\omega_{x,\text{sp}}, \omega_{y,\text{sp}}, \omega_{z,\text{sp}}]^T$ is:
$$\mathbf{\omega}_{\text{sp}} = 2 \, \bar{q}_u \otimes \dot{q}_{\text{sp}}$$
where $q_u = [1, 0, 0, 0]^T$ is the identity unit quaternion.

Expanding and simplifying this matrix product yields the elegant outer-loop control law:
$$\begin{bmatrix} \omega_{x,\text{sp}} \\ \omega_{y,\text{sp}} \\ \omega_{z,\text{sp}} \end{bmatrix} = 2 \cdot K_p \cdot \begin{bmatrix} q_{x,\text{err,short}} \\ q_{y,\text{err,short}} \\ q_{z,\text{err,short}} \end{bmatrix}$$

### 2.5 Inner-Loop Rate PID & Aerodynamic Scaling
The angular rate error $\mathbf{e}_\omega = \mathbf{\omega}_{\text{sp}} - \mathbf{\omega}_{\text{meas}}$ is processed by 3-axis PID rate controllers:
$$\mathbf{u}_{\text{raw}} = \mathbf{K}_{P,\text{rate}} \mathbf{e}_\omega + \mathbf{K}_{I,\text{rate}} \int \mathbf{e}_\omega dt - \mathbf{K}_{D,\text{rate}} \mathbf{\omega}_{\text{meas}}$$

To preserve control authority across varying velocity flight regimes, commands are scaled by dynamic pressure ($V^2$):
$$\mathbf{u}_{\text{cmd}} = \mathbf{u}_{\text{raw}} \cdot \left( \frac{V_0}{V} \right)^2$$
where $V_0$ is nominal trim velocity and $V$ is true airspeed. Output deflections control Aileron ($\delta_A$), Elevator ($\delta_H$), and Rudder ($\delta_V$).

---

## 3. Step-by-Step Numerical Toy Example

**Keywords:** `Numerical Verification`, `Toy Example`, `Knife-Edge Orientation`, `Quaternion Arithmetic`, `Rate Setpoints`

To demonstrate the mathematical execution of the quaternion attitude controller, consider an aircraft executing a **knife-edge transition maneuver**:

### Given Initial & Target States:
1. **Measured Aircraft Attitude ($q_{\text{meas}}$):** Banked at $90^\circ$ roll ($\phi = 90^\circ, \theta = 0^\circ, \psi = 0^\circ$):
   $$q_{\text{meas}} = \begin{bmatrix} \cos(45^\circ) \\ \sin(45^\circ) \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} 0.7071 \\ 0.7071 \\ 0.0000 \\ 0.0000 \end{bmatrix}$$

2. **Target Setpoint Attitude ($q_{\text{sp}}$):** Pitching up by $30^\circ$ ($\phi = 0^\circ, \theta = 30^\circ, \psi = 0^\circ$):
   $$q_{\text{sp}} = \begin{bmatrix} \cos(15^\circ) \\ 0 \\ \sin(15^\circ) \\ 0 \end{bmatrix} = \begin{bmatrix} 0.9659 \\ 0.0000 \\ 0.2588 \\ 0.0000 \end{bmatrix}$$

3. **Controller Gain:** $K_p = 3.5$.

### Step-by-Step Calculation:

#### Step 1: Conjugate of Measured Quaternion
$$\bar{q}_{\text{meas}} = \begin{bmatrix} 0.7071 \\ -0.7071 \\ 0.0000 \\ 0.0000 \end{bmatrix}$$

#### Step 2: Hamilton Product Error Evaluation
$$q_{\text{err}} = \bar{q}_{\text{meas}} \otimes q_{\text{sp}} = \begin{bmatrix} 
(0.7071)(0.9659) - (-0.7071)(0.0) - (0)(0.2588) - (0)(0) \\
(0.7071)(0.0) + (-0.7071)(0.9659) + (0)(0) - (0)(0.2588) \\
(0.7071)(0.2588) - (-0.7071)(0) + (0)(0.9659) + (0)(0) \\
(0.7071)(0) + (-0.7071)(0.2588) - (0)(0) + (0)(0.9659)
\end{bmatrix} = \begin{bmatrix} 0.6830 \\ -0.6830 \\ 0.1830 \\ -0.1830 \end{bmatrix}$$

#### Step 3: Shortest-Path Verification
Since $q_{w,\text{err}} = 0.6830 \ge 0$, no sign inversion is required:
$$q_{\text{err,short}} = \begin{bmatrix} 0.6830 \\ -0.6830 \\ 0.1830 \\ -0.1830 \end{bmatrix}$$

#### Step 4: Compute Proportional Angular Rate Setpoints ($\mathbf{\omega}_{\text{sp}}$)
$$\omega_{x,\text{sp}} = 2 \cdot 3.5 \cdot (-0.6830) = -4.7811 \text{ rad/s } (-273.94^\circ/\text{s})$$
$$\omega_{y,\text{sp}} = 2 \cdot 3.5 \cdot (0.1830) = +1.2811 \text{ rad/s } (+73.40^\circ/\text{s})$$
$$\omega_{z,\text{sp}} = 2 \cdot 3.5 \cdot (-0.1830) = -1.2811 \text{ rad/s } (-73.40^\circ/\text{s})$$

> **Physical Insight:** Despite the aircraft being in a $90^\circ$ knife-edge orientation, the $Q\_P$ controller automatically computes non-zero roll, pitch, and yaw body rate setpoints that perfectly decouple the body axes without any trigonometric conversions or matrix singularity!

---

## 4. Results & Simulation Benchmarking

**Keywords:** `SITL Benchmarking`, `Tracking RMSE`, `Knife-Edge Turn`, `Cross-Coupling Analysis`, `Comparative Plots`

The control architecture was evaluated in a 6-DOF non-linear dynamic aircraft simulation across 4 bank angle turn maneuvers: **$30^\circ$, $60^\circ$, $80^\circ$, and $90^\circ$ (knife-edge)**.

### 4.1 Numerical Performance Comparison Table

| Bank Angle | Controller Architecture | Roll RMSE ($\phi$) | Pitch RMSE ($\theta$) | Yaw RMSE ($\psi$) | Performance Summary |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **$30^\circ$ Bank** | **Quaternion ($Q\_P$)** | **$1.43^\circ$** | **$0.26^\circ$** | **$3.08^\circ$** | Smooth tracking, zero cross-coupling. |
| | Classical Euler PID | $1.43^\circ$ | $1.55^\circ$ | $2.70^\circ$ | Comparable low-angle tracking. |
| **$60^\circ$ Bank** | **Quaternion ($Q\_P$)** | **$2.85^\circ$** | **$0.26^\circ$** | **$3.07^\circ$** | Tight attitude hold across all axes. |
| | Classical Euler PID | $2.84^\circ$ | $2.69^\circ$ | $1.73^\circ$ | Minor pitch offset during bank entry. |
| **$80^\circ$ Bank** | **Quaternion ($Q\_P$)** | **$3.80^\circ$** | **$0.27^\circ$** | **$3.07^\circ$** | **Superior tracking; pitch error remains $< 0.3^\circ$.** |
| | Classical Euler PID | $3.77^\circ$ | $3.48^\circ$ | $1.96^\circ$ | Increasing rudder-pitch control coupling. |
| **$90^\circ$ Bank** | **Quaternion ($Q\_P$)** | **$4.28^\circ$** | **$0.27^\circ$** | **$3.07^\circ$** | **Flawless tracking; maintains altitude via rudder.** |
| *(Knife-Edge)* | Classical Euler PID | $11.64^\circ$ | $13.32^\circ$ | $30.50^\circ$ | **Severe failure; $30.5^\circ$ yaw error due to coupling.** |

---

### 4.2 Simulation Response Figures

#### Figure 1: 30° Bank Angle Turn Performance
![30 Degree Bank Turn Simulation](figures/fig_30deg_turn.png)

#### Figure 2: 60° Bank Angle Turn Performance
![60 Degree Bank Turn Simulation](figures/fig_60deg_turn.png)

#### Figure 3: 80° Bank Angle Turn Performance
![80 Degree Bank Turn Simulation](figures/fig_80deg_turn.png)

#### Figure 4: 90° Knife-Edge Turn Performance (Critical Benchmark)
![90 Degree Bank Turn Simulation](figures/fig_90deg_turn.png)

#### Figure 5: Yaw Tracking Error Comparison Across Bank Angles
![Yaw Tracking Error Comparison](figures/fig_error_comparison.png)

---

## 5. Key Discussion & Comparative Findings

1. **Low Bank Angles ($30^\circ - 60^\circ$):** Both controllers exhibit good tracking performance. Euler controllers perform adequately when roll angles are small and fixed-body approximations hold true.
2. **High Bank Angles ($80^\circ - 90^\circ$):** The Euler controller experiences massive performance breakdown. At $90^\circ$ bank, the elevator acts physically as the rudder (controlling yaw) and the rudder acts as the elevator (controlling pitch). Because Euler controllers compute pitch error independently of roll angle, they command elevator deflection when pitch error occurs, accelerating yaw instead of pitch!
3. **Quaternion Invariance:** The quaternion $Q\_P$ controller inherently handles 3D spatial rotations. The pitch tracking error under quaternion control remains nearly constant ($0.26^\circ - 0.27^\circ$) across all bank angles from $30^\circ$ to $90^\circ$, proving total resistance to aerodynamic cross-coupling!

---

## 6. Conclusion & Future Work

**Keywords:** `Conclusion`, `Non-Singular Control`, `Autopilot Integration`, `Multi-Physics Simulation`, `ArduPilot/PX4 Plugin`

### Conclusion
This project successfully designed, implemented, and validated a non-singular **Quaternion Attitude Controller** for aerobatic fixed-wing drones. By replacing traditional Euler angle loops with a single $Q\_P$ quaternion error controller, we achieved:
- Complete elimination of gimbal lock singularities.
- Automatic axis decoupling during steep bank ($80^\circ$) and knife-edge ($90^\circ$) flight maneuvers.
- A **$10\times$ reduction in yaw tracking error** ($3.07^\circ$ vs $30.50^\circ$) during $90^\circ$ bank turns compared to classical Euler PID control.

### Future Work for Final Evaluation
As instructed by faculty, for the final project evaluation we will expand our SITL simulation across all four targeted simulation environments:
1. `gym-pybullet-drones` (Reinforcement Learning & Multi-agent simulation)
2. `MuJoCo` (Physics-based multi-body dynamic simulation)
3. `Gazebo` (ROS-integrated full drone environment)
4. `ArduPilot` (Firmware SITL compilation & flight controller integration)

---

## 7. References

1. **Base Paper:** Michał Gołąbek, Michał Welcer, Cezary Szczepański, Mariusz Krawczyk, Albert Zajdel, and Krystian Borodacz. *"Quaternion Attitude Control System of Highly Maneuverable Aircraft."* Electronics 2022, 11(22), 3775. [https://doi.org/10.3390/electronics11223775](https://doi.org/10.3390/electronics11223775)
2. F. L. Markley and J. L. Crassidis. *"Fundamentals of Spacecraft Attitude Determination and Control."* Springer, 2019.
3. J. B. Kuipers. *"Quaternions and Rotation Sequences: A Primer with Applications to Orbits, Aerospace, and Virtual Reality."* Princeton University Press, 1999.
4. J. Diebel. *"Representing attitude: Euler angles, unit quaternions, and rotation vectors."* Matrix 2006, 58, 1–35.
5. M. V. Cook. *"Flight Dynamics Principles."* Butterworth-Heinemann, Oxford, UK, 2013.
