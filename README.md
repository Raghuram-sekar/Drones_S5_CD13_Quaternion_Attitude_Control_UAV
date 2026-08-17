# Quaternion Attitude Control System of Highly Maneuverable Aerial Vehicles

**Course:** 21AER314 / Drone Technologies & Flight Control  
**Institution:** Amrita School of Engineering, Amrita Vishwa Vidyapeetham  
**Repository Name:** `Drones_S5_CD13_Quaternion_Attitude_Control_UAV`  
**Group Number:** 13 (Section C/D)  

---

## Team Member Details

| S.No | Name | Roll Number | Section |
| :---: | :--- | :--- | :---: |
| 1 | Raghuram Sekar | CB.SC.U4AIE24247 | C |
| 2 | Athul Adithyan N | CB.SC.U4AIE24306 | D |
| 3 | Amogh Dey | CB.SC.U4AIE24303 | D |
| 4 | Prakhar Goyal | CB.SC.U4AIE24366 | D |
| 5 | Kneev S Jain | CB.SC.U4AIE24364 | D |

---

## Project Title & Abstract

### Title
**Quaternion-Based Non-Singular Attitude Control Architecture for Highly Maneuverable Fixed-Wing UAVs and Aerobatic Drones**

### Abstract
Modern unmanned aerial vehicles (UAVs) and high-performance strike drones are increasingly required to execute aggressive spatial maneuvers, including steep bank turns ($80^{\circ} - 90^{\circ}$), vertical climbs, and knife-edge flights. Traditional attitude control architectures relying on **Euler angles** ($\phi, \theta, \psi$) suffer from inherent mathematical singularities (gimbal lock at $\theta = \pm 90^{\circ}$) and catastrophic control surface cross-coupling (where elevators inadvertently control yaw and rudders control pitch). This project presents a non-singular **Quaternion Proportional ($Q_P$) Attitude Controller** cascaded with a 3-axis inner-loop angular rate PID controller. Using unit quaternion kinematics, the controller guarantees non-singular, shortest-path 3D attitude tracking across full spatial rotations without trigonometric overhead. The system is validated in a 6-DOF dynamic Software-in-the-Loop (SITL) simulation environment modeling an aerobatic aircraft (Extra 330SC). Comparative benchmarks against conventional Euler PID controllers across $30^{\circ}, 60^{\circ}, 80^{\circ},$ and $90^{\circ}$ bank maneuvers demonstrate that the quaternion controller completely eliminates cross-coupling error, reducing tracking RMSE at $90^{\circ}$ bank from $30.50^{\circ}$ (Euler) to $3.07^{\circ}$ (Quaternion).


---

## 1. Introduction


Flight attitude control is a cornerstone of autonomous aerial robotics. Conventional autopilot systems (such as standard ArduPilot or PX4 configurations) traditionally express aircraft orientation using **Euler angles** (Roll $\phi$, Pitch $\theta$, Yaw $\psi$). While intuitive for pilot display interfaces, Euler angles introduce severe mathematical degradation during extreme aerobatic or combat maneuvers:

1. **Gimbal Lock Singularity:** When the aircraft pitch angle reaches $\theta = \pm 90^{\circ}$, the transformation matrix mapping body angular rates $(p, q, r)$ to Euler angle rates $(\dot{\phi}, \dot{\theta}, \dot{\psi})$ becomes singular:

$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi \sec\theta & \cos\phi \sec\theta \end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

As $\theta \to \pm 90^{\circ}$, $\tan\theta \to \infty$ and $\sec\theta \to \infty$, causing numeric overflow and control system breakdown.

2. **Elevator-Rudder Coupling at High Bank Angles:** In knife-edge flight ($\phi = 90^{\circ}$), the physical elevator produces yawing moment relative to the inertial frame, while the rudder produces pitching moment. Classical Euler controllers fail to decouple these control surfaces without complex gain scheduling.
3. **Trigonometric Computational Burden:** Continuous evaluation of trigonometric functions ($\sin, \cos, \tan$) increases CPU execution latency in embedded flight control computers.

### Objective
This project implements the quaternion-based attitude control scheme proposed by Michał Gołąbek et al. (MDPI Electronics 2022). The primary objective is to replace the outer Euler angle loop with a unit quaternion attitude controller ($Q_P$) that operates entirely in hyper-complex quaternion space.

---

## 2. Methodology & Mathematical Formulation


<p align="center">
  <img src="figures/methodology_control_architecture.png" alt="Cascaded Quaternion Control Architecture Diagram" width="850"/>
  <br>
  <em>Figure 2.1: Cascaded Quaternion Attitude Control & Inner-Loop Rate Loop System Architecture.</em>
</p>

### 2.1 Unit Quaternion Fundamentals
A quaternion $q \in \mathbb{H}$ is defined as a 4-dimensional hyper-complex number:

$$
q = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix} = q_w + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}
$$

where $q_w \in \mathbb{R}$ is the scalar component and $\mathbf{q}_v = [q_x, q_y, q_z]^T \in \mathbb{R}^3$ is the vector imaginary part.

Unit quaternions satisfy the norm constraint $\|q\| = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2} = 1$, forming the group $Sp(1) \cong SO(3)$.

The **Quaternion Conjugate** $\bar{q}$ is defined as:

$$
\bar{q} = \begin{bmatrix} q_w \\ -q_x \\ -q_y \\ -q_z \end{bmatrix}
$$

The **Hamilton Product** ($p \otimes q$) representing successive spatial rotations is:

$$
p \otimes q = \begin{bmatrix} 
p_w q_w - p_x q_x - p_y q_y - p_z q_z \\
p_w q_x + p_x q_w + p_y q_z - p_z q_y \\
p_w q_y - p_x q_z + p_y q_w + p_z q_x \\
p_w q_z + p_x q_y - p_y q_x + p_z q_w 
\end{bmatrix}
$$

### 2.2 Attitude Error Quaternion Derivation ($q_{err}$)
Let $q_{meas}$ denote the measured aircraft attitude quaternion, and $q_{sp}$ denote the target setpoint quaternion. The intrinsic rotation relationship is:

$$
q_{sp} = q_{meas} \otimes q_{err}
$$

Left-multiplying both sides by the conjugate $\bar{q}_{meas}$ yields the explicit attitude error equation:

$$
q_{err} = \bar{q}_{meas} \otimes q_{sp}
$$

### 2.3 Shortest-Path Rotation Resolution
Because $q$ and $-q$ represent identical spatial rotations (double covering of $SO(3)$), the controller must ensure rotation along the shorter arc ($\le 180^{\circ}$). The condition is evaluated on the scalar component $q_{w,err}$:

$$
q_{err,short} = \begin{cases} -q_{err} & \text{if } q_{w,err} < 0 \\ q_{err} & \text{if } q_{w,err} \ge 0 \end{cases}
$$

### 2.4 Outer-Loop Control Law ($Q_P$ Controller)
Assuming the setpoint quaternion derivative $\dot{q}_{sp}$ is proportional to error with proportional gain $K_p$:

$$
\dot{q}_{sp} = K_p \cdot q_{err,short}
$$

The relationship connecting quaternion derivative $\dot{q}$ to body angular rate setpoints $\boldsymbol{\omega}_{sp} = [\omega_{x,sp}, \omega_{y,sp}, \omega_{z,sp}]^T$ is:

$$
\boldsymbol{\omega}_{sp} = 2 \, \bar{q}_u \otimes \dot{q}_{sp}
$$

where $q_u = [1, 0, 0, 0]^T$ is the identity unit quaternion.

Expanding and simplifying yields the outer-loop control law:

$$
\begin{bmatrix} \omega_{x,sp} \\ \omega_{y,sp} \\ \omega_{z,sp} \end{bmatrix} = 2 \cdot K_p \cdot \begin{bmatrix} q_{x,err,short} \\ q_{y,err,short} \\ q_{z,err,short} \end{bmatrix}
$$

### 2.5 Inner-Loop Rate PID & Aerodynamic Scaling
The angular rate error $\mathbf{e}_\omega = \boldsymbol{\omega}_{sp} - \boldsymbol{\omega}_{meas}$ is processed by 3-axis PID rate controllers:

$$
\mathbf{u}_{raw} = \mathbf{K}_{P,rate} \mathbf{e}_\omega + \mathbf{K}_{I,rate} \int \mathbf{e}_\omega dt - \mathbf{K}_{D,rate} \boldsymbol{\omega}_{meas}
$$

To preserve control authority across varying velocity flight regimes, commands are scaled by dynamic pressure ($V^2$):

$$
\mathbf{u}_{cmd} = \mathbf{u}_{raw} \cdot \left( \frac{V_0}{V} \right)^2
$$

where $V_0$ is nominal trim velocity and $V$ is true airspeed. Output deflections control Aileron ($\delta_A$), Elevator ($\delta_H$), and Rudder ($\delta_V$).

---

## 3. Step-by-Step Numerical Toy Example


### Problem Scenario Setup

An aircraft is flying in a **$90^{\circ}$ Knife-Edge Bank Turn** (measured attitude $q_{meas}$) and receives a setpoint command from the guidance computer to **pitch up by $30^{\circ}$** (setpoint attitude $q_{sp}$).

#### Given System Parameters:

1. **Measured Aircraft Attitude ($q_{meas}$):** Banked at $90^{\circ}$ roll ($\phi = 90^{\circ}, \theta = 0^{\circ}, \psi = 0^{\circ}$):

$$
q_{meas} = \begin{bmatrix} \cos(45^{\circ}) \\ \sin(45^{\circ}) \\ 0.0000 \\ 0.0000 \end{bmatrix} = \begin{bmatrix} 0.7071 \\ 0.7071 \\ 0.0000 \\ 0.0000 \end{bmatrix}
$$

2. **Target Setpoint Attitude ($q_{sp}$):** Pitching up by $30^{\circ}$ ($\phi = 0^{\circ}, \theta = 30^{\circ}, \psi = 0^{\circ}$):

$$
q_{sp} = \begin{bmatrix} \cos(15^{\circ}) \\ 0.0000 \\ \sin(15^{\circ}) \\ 0.0000 \end{bmatrix} = \begin{bmatrix} 0.9659 \\ 0.0000 \\ 0.2588 \\ 0.0000 \end{bmatrix}
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


The control architecture was evaluated in a 6-DOF non-linear dynamic aircraft simulation across 4 bank angle turn maneuvers: **$30^{\circ}$, $60^{\circ}$, $80^{\circ}$, and $90^{\circ}$ (knife-edge)**.

### 4.1 Numerical Performance Comparison Table

| Bank Angle | Controller Architecture | Roll RMSE ($\phi$) | Pitch RMSE ($\theta$) | Yaw RMSE ($\psi$) | Performance Summary |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **30° Bank** | Quaternion ($Q_P$) | **1.43°** | **0.26°** | **3.08°** | Smooth tracking, zero cross-coupling. |
| | Classical Euler PID | 1.43° | 1.55° | 2.70° | Comparable low-angle tracking. |
| **60° Bank** | Quaternion ($Q_P$) | **2.85°** | **0.26°** | **3.07°** | Tight attitude hold across all axes. |
| | Classical Euler PID | 2.84° | 2.69° | 1.73° | Minor pitch offset during bank entry. |
| **80° Bank** | Quaternion ($Q_P$) | **3.80°** | **0.27°** | **3.07°** | **Superior tracking; pitch error remains < 0.3°.** |
| | Classical Euler PID | 3.77° | 3.48° | 1.96° | Increasing rudder-pitch control coupling. |
| **90° Bank** | Quaternion ($Q_P$) | **4.28°** | **0.27°** | **3.07°** | **Flawless tracking; maintains altitude via rudder.** |
| *(Knife-Edge)* | Classical Euler PID | 11.64° | 13.32° | 30.50° | **Severe failure; 30.5° yaw error due to coupling.** |

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

1. **Low Bank Angles ($30^{\circ} - 60^{\circ}$):** Both controllers exhibit good tracking performance. Euler controllers perform adequately when roll angles are small and fixed-body approximations hold true.
2. **High Bank Angles ($80^{\circ} - 90^{\circ}$):** The Euler controller experiences massive performance breakdown. At $90^{\circ}$ bank, the elevator acts physically as the rudder (controlling yaw) and the rudder acts as the elevator (controlling pitch). Because Euler controllers compute pitch error independently of roll angle, they command elevator deflection when pitch error occurs, accelerating yaw instead of pitch!
3. **Quaternion Invariance:** The quaternion $Q_P$ controller inherently handles 3D spatial rotations. The pitch tracking error under quaternion control remains nearly constant ($0.26^{\circ} - 0.27^{\circ}$) across all bank angles from $30^{\circ}$ to $90^{\circ}$, proving total resistance to aerodynamic cross-coupling!

---

## 6. Conclusion & Future Work


### Conclusion
This project successfully designed, implemented, and validated a non-singular **Quaternion Attitude Controller** for aerobatic fixed-wing drones. By replacing traditional Euler angle loops with a single $Q_P$ quaternion error controller, we achieved:
- Complete elimination of gimbal lock singularities.
- Automatic axis decoupling during steep bank ($80^{\circ}$) and knife-edge ($90^{\circ}$) flight maneuvers.
- A **$10\times$ reduction in yaw tracking error** ($3.07^{\circ}$ vs $30.50^{\circ}$) during $90^{\circ}$ bank turns compared to classical Euler PID control.

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
