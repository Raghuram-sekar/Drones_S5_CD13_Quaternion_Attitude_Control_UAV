# First Review Presentation Slides: Quaternion Attitude Control System

**Course:** 21AER314 Drone Technologies & Flight Control  
**Institution:** Amrita School of Engineering, Amrita Vishwa Vidyapeetham  
**Repository Name:** `Drones_S5_CD13_Quaternion_Attitude_Control_UAV`  
**Group Number:** 13 (Section C/D)  

---

## Slide 1: Title & Team Details
- **Title:** Quaternion Attitude Control System of Highly Maneuverable Aircraft & Drones
- **Base Paper:** Michał Gołąbek et al., MDPI Electronics 2022
- **Faculty Guide:** Prof. Sunil Kumar Sir

### Team Members (Group 13):
1. Raghuram Sekar | CB.SC.U4AIE24247 | Sec C
2. Athul Adithyan N | CB.SC.U4AIE24306 | Sec D
3. Amogh Dey | CB.SC.U4AIE24303 | Sec D
4. Prakhar Goyal | CB.SC.U4AIE24366 | Sec D
5. Kneev S Jain | CB.SC.U4AIE24364 | Sec D

---

## Slide 2: Problem Statement & Motivation
- **The Challenge:** Modern aerobatic drones & fighter UAVs execute extreme bank turns ($80^{\circ} - 90^{\circ}$).
- **Euler Angle Limitations:**
  1. **Gimbal Lock Singularity:** Matrix singularity at pitch $\theta = \pm 90^{\circ}$.
  2. **Control Cross-Coupling:** At $90^{\circ}$ bank, elevator controls yaw and rudder controls pitch. Euler controllers fail without complex gain scheduling.
  3. **Computation:** Heavy trigonometric function overhead.

---

## Slide 3: Proposed Solution - Quaternion $Q_P$ Controller
- **Cascaded Architecture:**
  - **Outer Loop:** Unit Quaternion Proportional ($Q_P$) controller working in hyper-complex $SO(3)$ space.
  - **Inner Loop:** 3-Axis Angular Rate PID controllers with dynamic airspeed scaling ($V^2$).
- **Key Advantages:**
  - Non-singular over full $360^{\circ}$ spatial rotations.
  - Shortest-path rotation arc enforcement ($q_{w,err} \ge 0$).
  - Automatic cross-axis decoupling.

---

## Slide 4: Mathematical Formulation
- **Attitude Error:**

$$
q_{err} = \bar{q}_{meas} \otimes q_{sp}
$$

- **Shortest Arc Logic:**

$$
\text{If } q_{w,err} < 0 \implies q_{err,short} = -q_{err}
$$

- **Body Angular Rate Setpoints:**

$$
\begin{bmatrix} \omega_{x,sp} \\ \omega_{y,sp} \\ \omega_{z,sp} \end{bmatrix} = 2 \cdot K_p \cdot \begin{bmatrix} q_{x,err,short} \\ q_{y,err,short} \\ q_{z,err,short} \end{bmatrix}
$$

---

## Slide 5: Numerical Toy Example Walkthrough
- **Knife-Edge Orientation ($90^{\circ}$ Roll, $0^{\circ}$ Pitch):** $q_{meas} = [0.7071, 0.7071, 0, 0]^T$
- **Pitch Setpoint ($30^{\circ}$ Pitch):** $q_{sp} = [0.9659, 0, 0.2588, 0]^T$
- **Error Quaternion:** $q_{err} = [0.6830, -0.6830, 0.1830, -0.1830]^T$
- **Output Rates ($K_p = 3.5$):**
  - Roll Rate: $-4.78 \text{ rad/s } (-273.94^{\circ}/\text{s})$
  - Pitch Rate: $+1.28 \text{ rad/s } (+73.40^{\circ}/\text{s})$
  - Yaw Rate: $-1.28 \text{ rad/s } (-73.40^{\circ}/\text{s})$

---

## Slide 6: Simulation Results & Benchmarking
- **6-DOF Dynamic Aircraft SITL Simulation (Extra 330SC):**
  - **30° Turn:** Quaternion Yaw RMSE $3.08^{\circ}$ vs Euler $2.70^{\circ}$.
  - **60° Turn:** Quaternion Yaw RMSE $3.07^{\circ}$ vs Euler $1.73^{\circ}$.
  - **80° Turn:** Quaternion Pitch RMSE $0.27^{\circ}$ vs Euler $3.48^{\circ}$.
  - **90° Knife-Edge Turn:** **Quaternion Yaw RMSE $3.07^{\circ}$ vs Euler Yaw RMSE $30.50^{\circ}$!**

---

## Slide 7: Conclusion & Roadmap for Final Evaluation
- **Conclusion:** Quaternion $Q_P$ controller successfully eliminates gimbal lock and cross-coupling errors, outperforming classical Euler PID control.
- **Roadmap for Final Evaluation:** Expand simulation across all 4 platforms:
  1. `gym-pybullet-drones`
  2. `MuJoCo`
  3. `Gazebo`
  4. `ArduPilot SITL`
