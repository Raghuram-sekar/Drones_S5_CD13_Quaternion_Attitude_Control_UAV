<p align="center">
  <img src="logo-branding-amrita-universiy-2024.jpeg" alt="Amrita Vishwa Vidyapeetham Logo" width="400"/>
</p>

# Project Report: Quaternion Attitude Control System of Highly Maneuverable Aircraft

**Course:** 21AER314 / Drone Technologies & Flight Control  
**Department:** Aerospace Engineering  
**Institution:** Amrita School of Engineering, Amrita Vishwa Vidyapeetham  
**Repository Name:** `Drones_S5_CD13_Quaternion_Attitude_Control_UAV`  
**Group Number:** 13 (Section C/D)  
**Review Target:** First Project Review (SITL Simulation & Math Report)  

---

## Team Information

| S.No | Name | Roll Number | Section |
| :---: | :--- | :--- | :---: |
| 1 | Raghuram Sekar | CB.SC.U4AIE24247 | C |
| 2 | Athul Adithyan N | CB.SC.U4AIE24306 | D |
| 3 | Amogh Dey | CB.SC.U4AIE24303 | D |
| 4 | Prakhar Goyal | CB.SC.U4AIE24366 | D |
| 5 | Kneev S Jain | CB.SC.U4AIE24364 | D |

---

## Executive Summary


This project report details the design, mathematical derivation, software-in-the-loop (SITL) simulation, and comparative analysis of a non-singular **Quaternion Proportional ($Q_P$) Attitude Controller** for highly maneuverable fixed-wing unmanned aerial vehicles (UAVs) and aerobatic drones. 

Conventional Euler angle controllers ($\phi, \theta, \psi$) suffer from gimbal lock singularities at extreme pitch angles ($\theta = \pm 90^{\circ}$) and severe cross-coupling between elevator and rudder surfaces at steep bank angles ($80^{\circ} - 90^{\circ}$). By executing outer-loop attitude control entirely within unit quaternion space ($SO(3)$), the proposed controller achieves singularity-free attitude tracking, shortest-path rotation arcs via SLERP, and natural aerodynamic decoupling. 

In 6-DOF dynamic flight simulations modeling an Extra 330SC aircraft, the quaternion controller maintained pitch tracking error below $0.27^{\circ}$ across all bank angles up to $90^{\circ}$ knife-edge flight, whereas the classical Euler controller exhibited severe failure with yaw tracking error exploding to $30.50^{\circ}$.

---

## 1. Introduction & Background


Flight attitude control forms the core inner loop of an aircraft autopilot system. While Euler angles provide an intuitive visualization of roll ($\phi$), pitch ($\theta$), and yaw ($\psi$), they introduce mathematical singularities and control degradation under aggressive maneuvering:

1. **Gimbal Lock Singularity:** At $\theta = \pm 90^{\circ}$, the matrix transforming body angular rates $(p, q, r)$ to Euler angle rates $(\dot{\phi}, \dot{\theta}, \dot{\psi})$ becomes singular:

$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi \sec\theta & \cos\phi \sec\theta \end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

As $\theta \to \pm 90^{\circ}$, $\tan\theta \to \infty$ and $\sec\theta \to \infty$, causing numeric overflow and control failure.

2. **Control Surface Cross-Coupling:** When an aircraft banks at $90^{\circ}$ (knife-edge orientation), the physical elevator produces yawing moment relative to the inertial frame, and the rudder produces pitching moment. Classical Euler PID controllers attempt to correct pitch error by deflecting the elevator, causing unexpected yaw acceleration.

To overcome these shortcomings, this project implements a unit quaternion attitude control architecture as described by Michał Gołąbek et al. (MDPI Electronics 2022).

---

## 2. Mathematical Methodology & Formulation


<p align="center">
  <img src="figures/methodology_control_architecture.png" alt="Cascaded Quaternion Control Architecture Diagram" width="850"/>
  <br>
  <em>Figure 2.1: Cascaded Quaternion Attitude Control & Inner-Loop Rate Loop System Architecture.</em>
</p>

### 2.1 Quaternion Algebra & Rotations
A unit quaternion $q \in \mathbb{H}$ represents orientation without singularities:

$$
q = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix} = q_w + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}, \quad \|q\| = 1
$$

The **Hamilton Product** ($p \otimes q$) is non-commutative and corresponds to composite spatial rotations:

$$
p \otimes q = \begin{bmatrix} 
p_w q_w - p_x q_x - p_y q_y - p_z q_z \\
p_w q_x + p_x q_w + p_y q_z - p_z q_y \\
p_w q_y - p_x q_z + p_y q_w + p_z q_x \\
p_w q_z + p_x q_y - p_y q_x + p_z q_w 
\end{bmatrix}
$$

### 2.2 Attitude Error Formulation ($q_{err}$)
Given current measured quaternion $q_{meas}$ and target setpoint quaternion $q_{sp}$, intrinsic orientation yields:

$$
q_{sp} = q_{meas} \otimes q_{err} \implies q_{err} = \bar{q}_{meas} \otimes q_{sp}
$$

where $\bar{q}_{meas} = [q_{w,meas}, -q_{x,meas}, -q_{y,meas}, -q_{z,meas}]^T$ is the conjugate.

### 2.3 Shortest-Path Rotation Logic
To ensure rotation along the shortest angular path ($\le 180^{\circ}$):

$$
q_{err,short} = \begin{cases} -q_{err} & \text{if } q_{w,err} < 0 \\ q_{err} & \text{if } q_{w,err} \ge 0 \end{cases}
$$

### 2.4 $Q_P$ Outer Loop & Rate Setpoint Derivation
The setpoint rate of rotation $\dot{q}_{sp}$ is proportional to $q_{err,short}$ via proportional gain $K_p$:

$$
\dot{q}_{sp} = K_p \cdot q_{err,short}
$$

Converting quaternion rate $\dot{q}_{sp}$ to body frame angular rate setpoints $\boldsymbol{\omega}_{sp} = [\omega_{x,sp}, \omega_{y,sp}, \omega_{z,sp}]^T$:

$$
\boldsymbol{\omega}_{sp} = 2 \, \bar{q}_u \otimes \dot{q}_{sp} \implies \begin{bmatrix} \omega_{x,sp} \\ \omega_{y,sp} \\ \omega_{z,sp} \end{bmatrix} = 2 \cdot K_p \cdot \begin{bmatrix} q_{x,err,short} \\ q_{y,err,short} \\ q_{z,err,short} \end{bmatrix}
$$

---

## 3. Step-by-Step Numerical Toy Example


### Problem Scenario Setup

An aircraft is flying in a **$90^{\circ}$ Knife-Edge Bank Turn** (measured attitude $q_{meas}$) and receives a setpoint command from the guidance computer to **pitch up by $30^{\circ}$** (setpoint attitude $q_{sp}$).

#### Given System Parameters:

1. **Measured Aircraft Attitude ($q_{meas}$):** Banked at $90^{\circ}$ roll ($\phi = 90^{\circ}, \theta = 0^{\circ}, \psi = 0^{\circ}$):

$$
q_{meas} = \begin{bmatrix} \cos(45^{\circ}) \\ \sin(45^{\circ}) \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} 0.7071 \\ 0.7071 \\ 0.0000 \\ 0.0000 \end{bmatrix}
$$

2. **Target Setpoint Attitude ($q_{sp}$):** Pitching up by $30^{\circ}$ ($\phi = 0^{\circ}, \theta = 30^{\circ}, \psi = 0^{\circ}$):

$$
q_{sp} = \begin{bmatrix} \cos(15^{\circ}) \\ 0 \\ \sin(15^{\circ}) \\ 0 \end{bmatrix} = \begin{bmatrix} 0.9659 \\ 0.0000 \\ 0.2588 \\ 0.0000 \end{bmatrix}
$$

3. **Current Measured Gyro Speeds:** Currently stationary:

$$
\boldsymbol{\omega}_{meas} = \begin{bmatrix} 0.0 \\ 0.0 \\ 0.0 \end{bmatrix} \text{ rad/s}
$$

4. **Current Airspeed:** $V = 30\text{ m/s}$ (Trim airspeed $V_0 = 30\text{ m/s}$).

5. **Autopilot Gains:**
   - Outer Loop $Q_P$ Gain: $K_p = 3.5$
   - Inner Loop Roll Rate PID Gains: $K_{P,rate} = 0.5$, $K_{I,rate} = 2.0$, $K_{D,rate} = 0.1$
   - Time Step: $\Delta t = 0.01\text{ s}$
   - Accumulated Past Roll Error: $\sum (e_{\omega,x} \cdot \Delta t) = 0.05\text{ rad}$

---

### ─── OUTER LOOP ($Q_P$ CONTROLLER) ───

#### STEP 1: Compute Measured Quaternion Conjugate

Flip the imaginary vector signs of $q_{meas}$ to create the conjugate:

$$
\bar{q}_{meas} = \begin{bmatrix} 0.7071 \\ -0.7071 \\ 0.0000 \\ 0.0000 \end{bmatrix}
$$

#### STEP 2: Compute Attitude Error Quaternion

Execute the Hamilton Product ($q_{err} = \bar{q}_{meas} \otimes q_{sp}$):

$$
q_{err} = \begin{bmatrix} 0.7071 \\ -0.7071 \\ 0.0000 \\ 0.0000 \end{bmatrix} \otimes \begin{bmatrix} 0.9659 \\ 0.0000 \\ 0.2588 \\ 0.0000 \end{bmatrix}
$$

Expanding the 4 components:

$$
q_{w,err} = (0.7071)(0.9659) - (-0.7071)(0.0) - (0)(0.2588) - (0)(0) = 0.6830
$$

$$
q_{x,err} = (0.7071)(0.0) + (-0.7071)(0.9659) + (0)(0) - (0)(0.2588) = -0.6830
$$

$$
q_{y,err} = (0.7071)(0.2588) - (-0.7071)(0) + (0)(0.9659) + (0)(0) = 0.1830
$$

$$
q_{z,err} = (0.7071)(0) + (-0.7071)(0.2588) - (0)(0) + (0)(0.9659) = -0.1830
$$

Resulting Error Quaternion:

$$
q_{err} = \begin{bmatrix} 0.6830 \\ -0.6830 \\ 0.1830 \\ -0.1830 \end{bmatrix}
$$

#### STEP 3: Shortest-Path Arc Check

Check the scalar component $q_{w,err}$:
- Since $q_{w,err} = 0.6830 \ge 0$, no sign inversion is needed!

$$
q_{err,short} = \begin{bmatrix} 0.6830 \\ -0.6830 \\ 0.1830 \\ -0.1830 \end{bmatrix}
$$

#### STEP 4: Compute Outer Loop Target Spin Speeds

Apply the $Q_P$ Master Control Formula ($\boldsymbol{\omega}_{sp} = 2 \cdot K_p \cdot \mathbf{q}_{v,err,short}$) with $K_p = 3.5$:

- **Target Roll Spin Speed ($\omega_{x,sp}$):**

$$
\omega_{x,sp} = 2 \times 3.5 \times (-0.6830) = -4.7811\text{ rad/s } (-273.94^{\circ}/\text{s})
$$

- **Target Pitch Spin Speed ($\omega_{y,sp}$):**

$$
\omega_{y,sp} = 2 \times 3.5 \times (+0.1830) = +1.2811\text{ rad/s } (+73.40^{\circ}/\text{s})
$$

- **Target Yaw Spin Speed ($\omega_{z,sp}$):**

$$
\omega_{z,sp} = 2 \times 3.5 \times (-0.1830) = -1.2811\text{ rad/s } (-73.40^{\circ}/\text{s})
$$

> 🔗 **Outer Loop Output:** The Outer Loop outputs target spin speeds $\boldsymbol{\omega}_{sp} = [-4.7811, +1.2811, -1.2811]^T \text{ rad/s}$ down to the Inner Loop!

---

### ─── INNER LOOP (3-AXIS RATE PID CONTROLLER) ───

Follow the **Roll Axis ($x$)** through the Inner Rate PID Controller:

#### STEP 5: Calculate Roll Rate Error

$$
e_{\omega,x} = \omega_{x,sp} - \omega_{x,meas} = -4.7811 - 0.0 = -4.7811\text{ rad/s}
$$

#### STEP 6: Compute the 3 Individual PID Terms (Roll Axis)

1. **P Term (Instant Proportional Push):**

$$
P = K_{P,rate} \times e_{\omega,x} = 0.5 \times (-4.7811) = -2.39055
$$

2. **I Term (Accumulated Wind/Bias Fixer):**

$$
I = K_{I,rate} \times \sum (e_{\omega,x} \cdot \Delta t) = 2.0 \times (+0.05) = +0.10000
$$

3. **D Term (Braking Force):**
   Since current gyro speed $\omega_{x,meas} = 0.0$ and previous gyro speed is $0.0$:

$$
D = -K_{D,rate} \times \left( \frac{0.0 - 0.0}{0.01} \right) = 0.00000
$$

4. **Combine Raw PID Output ($u_{raw}$):**

$$
u_{raw} = P + I + D = -2.39055 + 0.10000 + 0.00000 = -2.29055
$$

#### STEP 7: Apply Dynamic Airspeed Scaling & Output Flap Command

To account for flight speed $V = 30\text{ m/s}$ and trim speed $V_0 = 30\text{ m/s}$:

$$
\text{Speed Scale} = \left( \frac{V_0}{V} \right)^2 = \left( \frac{30}{30} \right)^2 = 1.0
$$

$$
u_{cmd} = u_{raw} \times 1.0 = -2.29055\text{ rad}
$$

Finally, pass through actuator saturation limits (maximum aileron deflection command $\pm 1.0$ normalized):

$$
\delta_A = \text{clip}(-2.29055, -1.0, +1.0) = -1.0000 \quad (\text{Maximum Aileron Deflection Command!})
$$

---


## 4. Results & Simulation Benchmarking


### 4.1 Quantitative Tracking RMSE Benchmark Table

| Bank Angle Turn | Quaternion ($Q_P$) Roll / Pitch / Yaw RMSE | Euler PID Roll / Pitch / Yaw RMSE | Physical Evaluation |
| :---: | :--- | :--- | :--- |
| **30° Bank** | 1.43° / 0.26° / 3.08° | 1.43° / 1.55° / 2.70° | Both controllers track cleanly. |
| **60° Bank** | 2.85° / 0.26° / 3.07° | 2.84° / 2.69° / 1.73° | Quaternion pitch error stays below 0.26°. |
| **80° Bank** | 3.80° / 0.27° / 3.07° | 3.77° / 3.48° / 1.96° | Euler pitch error increases due to coupling. |
| **90° Knife-Edge** | **4.28° / 0.27° / 3.07°** | **11.64° / 13.32° / 30.50°** | **Euler yaw error explodes to 30.5°; Quaternion remains rock-solid.** |

### 4.2 Benchmark Figures
All simulation plots generated by `simulation.py`:
- **30° Bank Turn:** ![30 Deg](figures/fig_30deg_turn.png)
- **60° Bank Turn:** ![60 Deg](figures/fig_60deg_turn.png)
- **80° Bank Turn:** ![80 Deg](figures/fig_80deg_turn.png)
- **90° Knife-Edge Turn:** ![90 Deg](figures/fig_90deg_turn.png)
- **Error Comparison Chart:** ![Error Comparison](figures/fig_error_comparison.png)

---

## 5. Conclusion & Next Steps


The SITL simulation results conclusively validate the quaternion-based attitude controller's superiority over Euler-based architectures during aggressive flight. For the final evaluation, we will expand this controller across the four required simulation platforms:
1. `gym-pybullet-drones`
2. `MuJoCo`
3. `Gazebo`
4. `ArduPilot SITL`

---

## References

1. Gołąbek, M., Welcer, M., Szczepański, C., Krawczyk, M., Zajdel, A., & Borodacz, K. (2022). *Quaternion Attitude Control System of Highly Maneuverable Aircraft*. Electronics, 11(22), 3775. MDPI.
2. Markley, F. L., & Crassidis, J. L. (2019). *Fundamentals of Spacecraft Attitude Determination and Control*. Springer.
3. Kuipers, J. B. (1999). *Quaternions and Rotation Sequences*. Princeton University Press.
