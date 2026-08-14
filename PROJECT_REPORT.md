<p align="center">
  <img src="logo-branding-amrita-universiy-2024.jpeg" alt="Amrita Vishwa Vidyapeetham Logo" width="400"/>
</p>

# Project Report: Quaternion Attitude Control System of Highly Maneuverable Aircraft

**Course:** 21AER314 / Drone Technologies & Flight Control  
**Department:** Aerospace Engineering  
**Institution:** Amrita School of Engineering, Amrita Vishwa Vidyapeetham  
**Repository Name:** `Drones_S5_CD13_Quaternion_Attitude_Control_UAV`  
**Review Target:** First Project Review (SITL Simulation & Math Report)  

---

## Team Information

- **Student Name (Lead):** Raghuram S (`AM.EN.U4AERO22001`, `raghuram@amrita.edu`)
- **Team Member 2:** [Name] (`AM.EN.U4AERO22xxx`, `email2@amrita.edu`)
- **Team Member 3:** [Name] (`AM.EN.U4AERO22yyy`, `email3@amrita.edu`)

---

## Executive Summary

**Keywords:** `Avionics`, `Attitude Control`, `Unit Quaternions`, `Gimbal Lock`, `SITL Simulation`, `Cross-Coupling Resistance`

This project report details the design, mathematical derivation, software-in-the-loop (SITL) simulation, and comparative analysis of a non-singular **Quaternion Proportional ($Q\_P$) Attitude Controller** for highly maneuverable fixed-wing unmanned aerial vehicles (UAVs) and aerobatic drones. 

Conventional Euler angle controllers ($\phi, \theta, \psi$) suffer from gimbal lock singularities at extreme pitch angles ($\theta = \pm 90^\circ$) and severe cross-coupling between elevator and rudder surfaces at steep bank angles ($80^\circ - 90^\circ$). By executing outer-loop attitude control entirely within unit quaternion space ($SO(3)$), the proposed controller achieves singularity-free attitude tracking, shortest-path rotation arcs via SLERP, and natural aerodynamic decoupling. 

In 6-DOF dynamic flight simulations modeling an Extra 330SC aircraft, the quaternion controller maintained pitch tracking error below $0.27^\circ$ across all bank angles up to $90^\circ$ knife-edge flight, whereas the classical Euler controller exhibited severe failure with yaw tracking error exploding to $30.50^\circ$.

---

## 1. Introduction & Background

**Keywords:** `Aerobatic UAVs`, `Flight Dynamics`, `Euler Angle Limitations`, `Non-Singular Kinematics`

Flight attitude control forms the core inner loop of an aircraft autopilot system. While Euler angles provide an intuitive visualization of roll ($\phi$), pitch ($\theta$), and yaw ($\psi$), they introduce mathematical singularities and control degradation under aggressive maneuvering:

1. **Gimbal Lock Singularity:** At $\theta = \pm 90^\circ$, the matrix transforming body angular rates to Euler angle rates becomes singular:
   $$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi \sec\theta & \cos\phi \sec\theta \end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$
   As $\theta \to \pm 90^\circ$, $\tan\theta \to \infty$ and $\sec\theta \to \infty$, causing numeric overflow and control failure.

2. **Control Surface Cross-Coupling:** When an aircraft banks at $90^\circ$ (knife-edge orientation), the physical elevator produces yawing moment relative to the inertial frame, and the rudder produces pitching moment. Classical Euler PID controllers attempt to correct pitch error by deflecting the elevator, causing unexpected yaw acceleration.

To overcome these shortcomings, this project implements a unit quaternion attitude control architecture as described by Michał Gołąbek et al. (MDPI Electronics 2022).

---

## 2. Mathematical Methodology & Formulation

**Keywords:** `Quaternion Algebra`, `Hamilton Product`, `Attitude Error`, `Shortest Path Arc`, `Angular Rate Setpoints`

### 2.1 Quaternion Algebra & Rotations
A unit quaternion $q \in \mathbb{H}$ represents orientation without singularities:
$$q = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix} = q_w + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}, \quad \|q\| = 1$$

The **Hamilton Product** ($p \otimes q$) is non-commutative and corresponds to composite spatial rotations:
$$p \otimes q = \begin{bmatrix} 
p_w q_w - p_x q_x - p_y q_y - p_z q_z \\
p_w q_x + p_x q_w + p_y q_z - p_z q_y \\
p_w q_y - p_x q_z + p_y q_w + p_z q_x \\
p_w q_z + p_x q_y - p_y q_x + p_z q_w 
\end{bmatrix}$$

### 2.2 Attitude Error Formulation ($q_{\text{err}}$)
Given current measured quaternion $q_{\text{meas}}$ and target setpoint quaternion $q_{\text{sp}}$, intrinsic orientation yields:
$$q_{\text{sp}} = q_{\text{meas}} \otimes q_{\text{err}} \implies q_{\text{err}} = \bar{q}_{\text{meas}} \otimes q_{\text{sp}}$$
where $\bar{q}_{\text{meas}} = [q_{w,\text{meas}}, -q_{x,\text{meas}}, -q_{y,\text{meas}}, -q_{z,\text{meas}}]^T$ is the conjugate.

### 2.3 Shortest-Path Rotation Logic
To ensure rotation along the shortest angular path ($\le 180^\circ$):
$$q_{\text{err,short}} = \begin{cases} -q_{\text{err}} & \text{if } q_{w,\text{err}} < 0 \\ q_{\text{err}} & \text{if } q_{w,\text{err}} \ge 0 \end{cases}$$

### 2.4 $Q\_P$ Outer Loop & Rate Setpoint Derivation
The setpoint rate of rotation $\dot{q}_{\text{sp}}$ is proportional to $q_{\text{err,short}}$ via proportional gain $K_p$:
$$\dot{q}_{\text{sp}} = K_p \cdot q_{\text{err,short}}$$

Converting quaternion rate $\dot{q}_{\text{sp}}$ to body frame angular rate setpoints $\mathbf{\omega}_{\text{sp}} = [\omega_{x,\text{sp}}, \omega_{y,\text{sp}}, \omega_{z,\text{sp}}]^T$:
$$\mathbf{\omega}_{\text{sp}} = 2 \, \bar{q}_u \otimes \dot{q}_{\text{sp}} \implies \begin{bmatrix} \omega_{x,\text{sp}} \\ \omega_{y,\text{sp}} \\ \omega_{z,\text{sp}} \end{bmatrix} = 2 \cdot K_p \cdot \begin{bmatrix} q_{x,\text{err,short}} \\ q_{y,\text{err,short}} \\ q_{z,\text{err,short}} \end{bmatrix}$$

---

## 3. Step-by-Step Numerical Toy Example

**Keywords:** `Toy Example`, `Knife-Edge Orientation`, `Quaternion Step-by-Step Calculation`

### Scenario Setup:
An aircraft is operating in a $90^\circ$ knife-edge bank orientation ($q_{\text{meas}}$) and receives a setpoint command to pitch up by $30^\circ$ ($q_{\text{sp}}$). Controller gain $K_p = 3.5$.

1. **Input Quaternions:**
   $$q_{\text{meas}} = [0.7071, 0.7071, 0.0000, 0.0000]^T$$
   $$q_{\text{sp}} = [0.9659, 0.0000, 0.2588, 0.0000]^T$$

2. **Conjugate Evaluation:**
   $$\bar{q}_{\text{meas}} = [0.7071, -0.7071, 0.0000, 0.0000]^T$$

3. **Error Quaternion Calculation:**
   $$q_{\text{err}} = \bar{q}_{\text{meas}} \otimes q_{\text{sp}} = [0.6830, -0.6830, 0.1830, -0.1830]^T$$

4. **Shortest-Path Check:**
   $$q_{w,\text{err}} = 0.6830 \ge 0 \implies q_{\text{err,short}} = [0.6830, -0.6830, 0.1830, -0.1830]^T$$

5. **Angular Rate Setpoint Outputs:**
   $$\omega_{x,\text{sp}} = 2 \times 3.5 \times (-0.6830) = -4.7811 \text{ rad/s } (-273.94^\circ/\text{s})$$
   $$\omega_{y,\text{sp}} = 2 \times 3.5 \times (+0.1830) = +1.2811 \text{ rad/s } (+73.40^\circ/\text{s})$$
   $$\omega_{z,\text{sp}} = 2 \times 3.5 \times (-0.1830) = -1.2811 \text{ rad/s } (-73.40^\circ/\text{s})$$

---

## 4. Results & Simulation Benchmarking

**Keywords:** `RMSE Evaluation`, `6-DOF Dynamics`, `Comparative Plots`, `Airspeed Scaling`

### 4.1 Quantitative Tracking RMSE Benchmark Table

| Bank Angle Turn | Quaternion ($Q\_P$) Roll / Pitch / Yaw RMSE | Euler PID Roll / Pitch / Yaw RMSE | Physical Evaluation |
| :---: | :---: | :---: | :--- |
| **$30^\circ$ Bank** | $1.43^\circ \;\vert\; 0.26^\circ \;\vert\; 3.08^\circ$ | $1.43^\circ \;\vert\; 1.55^\circ \;\vert\; 2.70^\circ$ | Both controllers track cleanly. |
| **$60^\circ$ Bank** | $2.85^\circ \;\vert\; 0.26^\circ \;\vert\; 3.07^\circ$ | $2.84^\circ \;\vert\; 2.69^\circ \;\vert\; 1.73^\circ$ | Quaternion pitch error stays below $0.26^\circ$. |
| **$80^\circ$ Bank** | $3.80^\circ \;\vert\; 0.27^\circ \;\vert\; 3.07^\circ$ | $3.77^\circ \;\vert\; 3.48^\circ \;\vert\; 1.96^\circ$ | Euler pitch error increases due to coupling. |
| **$90^\circ$ Knife-Edge** | **$4.28^\circ \;\vert\; 0.27^\circ \;\vert\; 3.07^\circ$** | **$11.64^\circ \;\vert\; 13.32^\circ \;\vert\; 30.50^\circ$** | **Euler yaw error explodes to $30.5^\circ$; Quaternion remains rock-solid.** |

### 4.2 Benchmark Figures
All simulation plots generated by `simulation.py`:
- **30° Bank Turn:** ![30 Deg](figures/fig_30deg_turn.png)
- **60° Bank Turn:** ![60 Deg](figures/fig_60deg_turn.png)
- **80° Bank Turn:** ![80 Deg](figures/fig_80deg_turn.png)
- **90° Knife-Edge Turn:** ![90 Deg](figures/fig_90deg_turn.png)
- **Error Comparison Chart:** ![Error Comparison](figures/fig_error_comparison.png)

---

## 5. Conclusion & Next Steps

**Keywords:** `Conclusion`, `ArduPilot`, `Gazebo`, `MuJoCo`, `PyBullet`

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
