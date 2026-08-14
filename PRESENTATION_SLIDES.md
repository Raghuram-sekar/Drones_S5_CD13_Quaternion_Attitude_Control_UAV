# First Review Presentation Slides: Quaternion Attitude Control System

**Course:** 21AER314 Drone Technologies & Flight Control  
**Institution:** Amrita School of Engineering, Amrita Vishwa Vidyapeetham  
**Repository Name:** `Drones_S5_CD13_Quaternion_Attitude_Control_UAV`  

---

## Slide 1: Title & Team Details
- **Title:** Quaternion Attitude Control System of Highly Maneuverable Aircraft & Drones
- **Base Paper:** Michał Gołąbek et al., MDPI Electronics 2022
- **Team Lead:** Raghuram S (`AM.EN.U4AERO22001`)
- **Team Members:** Member 2 (`AM.EN.U4AERO22xxx`), Member 3 (`AM.EN.U4AERO22yyy`)
- **Faculty Guide:** Prof. Sunil Kumar Sir

---

## Slide 2: Problem Statement & Motivation
- **The Challenge:** Modern aerobatic drones & fighter UAVs execute extreme bank turns ($80^\circ - 90^\circ$).
- **Euler Angle Limitations:**
  1. **Gimbal Lock Singularity:** Matrix singularity at pitch $\theta = \pm 90^\circ$.
  2. **Control Cross-Coupling:** At $90^\circ$ bank, elevator controls yaw and rudder controls pitch. Euler controllers fail without complex gain scheduling.
  3. **Computation:** Heavy trigonometric function overhead.

---

## Slide 3: Proposed Solution - Quaternion $Q\_P$ Controller
- **Cascaded Architecture:**
  - **Outer Loop:** Unit Quaternion Proportional ($Q\_P$) controller working in hyper-complex $SO(3)$ space.
  - **Inner Loop:** 3-Axis Angular Rate PID controllers with dynamic airspeed scaling ($V^2$).
- **Key Advantages:**
  - Non-singular over full $360^\circ$ spatial rotations.
  - Shortest-path rotation arc enforcement ($q_{w,\text{err}} \ge 0$).
  - Automatic cross-axis decoupling.

---

## Slide 4: Mathematical Formulation
- **Attitude Error:**
  $$q_{\text{err}} = \bar{q}_{\text{meas}} \otimes q_{\text{sp}}$$
- **Shortest Arc Logic:**
  $$\text{If } q_{w,\text{err}} < 0 \implies q_{\text{err,short}} = -q_{\text{err}}$$
- **Body Angular Rate Setpoints:**
  $$\begin{bmatrix} \omega_{x,\text{sp}} \\ \omega_{y,\text{sp}} \\ \omega_{z,\text{sp}} \end{bmatrix} = 2 \cdot K_p \cdot \begin{bmatrix} q_{x,\text{err,short}} \\ q_{y,\text{err,short}} \\ q_{z,\text{err,short}} \end{bmatrix}$$

---

## Slide 5: Numerical Toy Example Walkthrough
- **Knife-Edge Orientation ($90^\circ$ Roll, $0^\circ$ Pitch):** $q_{\text{meas}} = [0.7071, 0.7071, 0, 0]^T$
- **Pitch Setpoint ($30^\circ$ Pitch):** $q_{\text{sp}} = [0.9659, 0, 0.2588, 0]^T$
- **Error Quaternion:** $q_{\text{err}} = [0.6830, -0.6830, 0.1830, -0.1830]^T$
- **Output Rates ($K_p = 3.5$):**
  - Roll Rate: $-4.78 \text{ rad/s } (-273.94^\circ/\text{s})$
  - Pitch Rate: $+1.28 \text{ rad/s } (+73.40^\circ/\text{s})$
  - Yaw Rate: $-1.28 \text{ rad/s } (-73.40^\circ/\text{s})$

---

## Slide 6: Simulation Results & Benchmarking
- **6-DOF Dynamic Aircraft SITL Simulation (Extra 330SC):**
  - **$30^\circ$ Turn:** Quaternion Yaw RMSE $3.08^\circ$ vs Euler $2.70^\circ$.
  - **$60^\circ$ Turn:** Quaternion Yaw RMSE $3.07^\circ$ vs Euler $1.73^\circ$.
  - **$80^\circ$ Turn:** Quaternion Pitch RMSE $0.27^\circ$ vs Euler $3.48^\circ$.
  - **$90^\circ$ Knife-Edge Turn:** **Quaternion Yaw RMSE $3.07^\circ$ vs Euler Yaw RMSE $30.50^\circ$!**

---

## Slide 7: Conclusion & Roadmap for Final Evaluation
- **Conclusion:** Quaternion $Q\_P$ controller successfully eliminates gimbal lock and cross-coupling errors, outperforming classical Euler PID control.
- **Roadmap for Final Evaluation:** Expand simulation across all 4 platforms:
  1. `gym-pybullet-drones`
  2. `MuJoCo`
  3. `Gazebo`
  4. `ArduPilot SITL`
