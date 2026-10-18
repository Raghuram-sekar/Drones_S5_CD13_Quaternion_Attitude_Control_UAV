"""
=============================================================================
Interactive 6-DOF Quadrotor Quaternion Flight Simulator & Control Dashboard
=============================================================================
Direct PyBullet Native Physics Implementation matching:
  - drones_pybullet_final.ipynb
  - Quaternion_Controller_Data_Driven.ipynb
  - Michał Gołąbek et al. (MDPI Electronics 2022)

Key Features:
  1. Real PyBullet 3D Physics Simulation (6-DOF Rigid-Body Dynamics, True Aerodynamics)
  2. Faithful Golabek et al. (2022) Q_P Attitude Controller (Eq. 11, 17-19, 27)
  3. Interactive Controls in PyBullet Viewport:
     - [Play / Pause] Button
     - [Reset / Re-initialize] Button
     - [Trajectory Mode] Dropdown: Figure-8, Circular Orbit, Hover Waypoint
     - [Flight Speed] & [Altitude] Sliders
  4. Real-time 3D Trajectory Visualization:
     - Dynamic 3D Red Mathematical Target Path
     - Cyan Trailing Path of Actual Drone Flight
  5. Synchronized Live 3-Axis Attitude Telemetry (Roll, Pitch, Yaw)
=============================================================================
"""

import sys
import math
import time
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


# =====================================================================
# 1. FAITHFUL QUATERNION ATTITUDE CONTROLLER (FROM YOUR NOTEBOOK)
# =====================================================================
class NotebookQuaternionController:
    """
    Direct implementation from drones_pybullet_final.ipynb & Golabek et al. (2022).
    """
    def __init__(self, mass=0.030, gravity=9.81):
        self.mass = mass
        self.gravity_mag = mass * gravity
        
        # Outer Position Loop Gains (m/s^2 per m and m/s error)
        self.P_COEFF_FOR = np.array([8.0, 8.0, 10.0])
        self.I_COEFF_FOR = np.array([1.5, 1.5, 2.0])
        self.D_COEFF_FOR = np.array([5.5, 5.5, 6.0])

        # Outer Loop Q_P Attitude Controller Parameters (Eq. 27 & Fig. 1)
        self.KP_QUAT = 12.0
        self.MAX_RATE_SP = 10.0  # rad/s

        # Inner Loop 3-Axis Angular Rate PI-D Controller
        self.KP_RATE = np.array([0.0014, 0.0014, 0.0012])
        self.KI_RATE = np.array([0.0001, 0.0001, 0.00005])
        self.KD_RATE = np.array([0.0002, 0.0002, 0.00005])

        self.reset_all_states()

    def reset_all_states(self):
        self.integral_pos_e = np.zeros(3)
        self.rate_integral = np.zeros(3)
        self.prev_omega_meas = np.zeros(3)
        self.d_omega_filt = np.zeros(3)
        self.first_step = True

    @staticmethod
    def quat_multiply(q1, q2):
        """Hamilton product (q1 * q2) in [qw, qx, qy, qz] format (Eq. 8)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def quat_conjugate(q):
        """Unit quaternion conjugate [qw, -qx, -qy, -qz] (Eq. 7)."""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def compute_control(self, dt, cur_pos, cur_vel, cur_quat_xyzw, cur_ang_vel_body,
                        target_pos, target_vel, target_acc, target_yaw):
        # -------------------------------------------------------------
        # 1. GUIDANCE SYSTEM (GS): Position PID + Force Synthesis
        # -------------------------------------------------------------
        pos_e = target_pos - cur_pos
        vel_e = target_vel - cur_vel

        self.integral_pos_e += pos_e * dt
        self.integral_pos_e = np.clip(self.integral_pos_e, -2.0, 2.0)
        self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, 0.15)

        target_force = (
            self.mass * (
                self.P_COEFF_FOR * pos_e
                + self.I_COEFF_FOR * self.integral_pos_e
                + self.D_COEFF_FOR * vel_e
                + target_acc
            )
            + np.array([0.0, 0.0, self.gravity_mag])
        )

        # Orthonormal desired attitude frame (R_sp)
        norm_f = np.linalg.norm(target_force)
        z_body_des = target_force / norm_f if norm_f > 1e-4 else np.array([0.0, 0.0, 1.0])

        x_c_des = np.array([math.cos(target_yaw), math.sin(target_yaw), 0.0])
        y_body_des = np.cross(z_body_des, x_c_des)
        norm_y = np.linalg.norm(y_body_des)
        y_body_des = y_body_des / norm_y if norm_y > 1e-4 else np.array([0.0, 1.0, 0.0])
        x_body_des = np.cross(y_body_des, z_body_des)

        R_sp = np.column_stack((x_body_des, y_body_des, z_body_des))
        q_sp_xyzw = R.from_matrix(R_sp).as_quat()

        # Convert to paper's [qw, qx, qy, qz] convention
        q_sp = np.array([q_sp_xyzw[3], q_sp_xyzw[0], q_sp_xyzw[1], q_sp_xyzw[2]])
        q_meas = np.array([cur_quat_xyzw[3], cur_quat_xyzw[0], cur_quat_xyzw[1], cur_quat_xyzw[2]])
        q_sp /= np.linalg.norm(q_sp)
        q_meas /= np.linalg.norm(q_meas)

        # -------------------------------------------------------------
        # 2. Q_P ATTITUDE CONTROLLER (Equations 11, 17-19, 27)
        # -------------------------------------------------------------
        q_err = self.quat_multiply(self.quat_conjugate(q_meas), q_sp)

        # Shortest-path check
        q_err_short = -q_err if q_err[0] < 0.0 else q_err

        # Eq. 27: w_sp = 2 * Kp * q_err_short
        omega_sp = 2.0 * self.KP_QUAT * q_err_short[1:4]
        omega_sp = np.clip(omega_sp, -self.MAX_RATE_SP, self.MAX_RATE_SP)

        # -------------------------------------------------------------
        # 3. INNER ANGULAR RATE PI-D CONTROLLER
        # -------------------------------------------------------------
        omega_err = omega_sp - cur_ang_vel_body

        self.rate_integral += omega_err * dt
        self.rate_integral = np.clip(self.rate_integral, -0.5, 0.5)

        if self.first_step:
            self.d_omega_filt = np.zeros(3)
            self.first_step = False
        else:
            raw_d = (cur_ang_vel_body - self.prev_omega_meas) / dt
            self.d_omega_filt = 0.2 * raw_d + 0.8 * self.d_omega_filt
        self.prev_omega_meas = cur_ang_vel_body.copy()

        torques = (
            self.KP_RATE * omega_err
            + self.KI_RATE * self.rate_integral
            - self.KD_RATE * self.d_omega_filt
        )

        R_meas = R.from_quat(cur_quat_xyzw).as_matrix()
        thrust_scalar = max(0.0, float(np.dot(target_force, R_meas[:, 2])))

        return thrust_scalar, torques, q_sp, q_err_short, pos_e


# =====================================================================
# 2. TRAJECTORY GENERATOR
# =====================================================================
def get_trajectory_setpoint(mode, t, speed=1.0, alt=1.5, scale=2.0):
    if mode == 0:  # Figure-8 (Lemniscate)
        if t < 2.0:
            theta, theta_dot, theta_ddot = 0.0, 0.0, 0.0
        elif t < 4.0:
            theta_ddot = speed / 2.0
            theta_dot = theta_ddot * (t - 2.0)
            theta = 0.5 * theta_ddot * (t - 2.0)**2
        else:
            theta_ddot = 0.0
            theta_dot = speed
            theta = speed + speed * (t - 4.0)

        x = scale * math.sin(theta)
        y = scale * math.sin(theta) * math.cos(theta)
        z = alt

        vx = scale * math.cos(theta) * theta_dot
        vy = scale * math.cos(2.0 * theta) * theta_dot
        vz = 0.0

        ax = -scale * math.sin(theta) * (theta_dot**2) + scale * math.cos(theta) * theta_ddot
        ay = -2.0 * scale * math.sin(2.0 * theta) * (theta_dot**2) + scale * math.cos(2.0 * theta) * theta_ddot
        az = 0.0

    elif mode == 1:  # Circle
        theta = speed * max(0.0, t - 1.0) * 0.8
        theta_dot = speed * 0.8 if t >= 1.0 else 0.0
        x = scale * math.cos(theta)
        y = scale * math.sin(theta)
        z = alt

        vx = -scale * math.sin(theta) * theta_dot
        vy = scale * math.cos(theta) * theta_dot
        vz = 0.0

        ax = -scale * math.cos(theta) * (theta_dot**2)
        ay = -scale * math.sin(theta) * (theta_dot**2)
        az = 0.0

    else:  # Static Hover Waypoint
        x, y, z = 0.0, 0.0, alt
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0

    target_pos = np.array([x, y, z])
    target_vel = np.array([vx, vy, vz])
    target_acc = np.array([ax, ay, az])

    if abs(vx) > 1e-3 or abs(vy) > 1e-3:
        target_yaw = math.atan2(vy, vx)
    else:
        target_yaw = math.pi / 4

    return target_pos, target_vel, target_acc, target_yaw


# =====================================================================
# 3. PYBULLET QUADROTOR MULTI-BODY CREATOR
# =====================================================================
def create_quadrotor_multibody(init_pos):
    """
    Constructs an aerodynamic Crazyflie-like quadrotor with colored cross-arms and rotor disks.
    """
    arm_length = 0.10
    body_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.015])
    body_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.015], rgbaColor=[0.1, 0.1, 0.15, 1.0])

    # Rotor link properties
    link_masses = [0.002, 0.002, 0.002, 0.002]
    link_col_indices = [-1, -1, -1, -1]
    link_vis_indices = []

    rotor_coords = [
        [arm_length, 0.0, 0.005],
        [-arm_length, 0.0, 0.005],
        [0.0, arm_length, 0.005],
        [0.0, -arm_length, 0.005]
    ]

    for i in range(4):
        # Front rotors red, rear rotors cyan
        color = [1.0, 0.2, 0.2, 0.9] if i < 2 else [0.0, 0.8, 1.0, 0.9]
        vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.035, length=0.004, rgbaColor=color)
        link_vis_indices.append(vis_shape)

    drone_id = p.createMultiBody(
        baseMass=0.027,  # Crazyflie 27g
        baseCollisionShapeIndex=body_col,
        baseVisualShapeIndex=body_vis,
        basePosition=init_pos,
        baseOrientation=[0, 0, 0, 1],
        linkMasses=link_masses,
        linkCollisionShapeIndices=link_col_indices,
        linkVisualShapeIndices=link_vis_indices,
        linkPositions=rotor_coords,
        linkOrientations=[[0, 0, 0, 1]] * 4,
        linkInertialFramePositions=[[0, 0, 0]] * 4,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
        linkParentIndices=[0, 0, 0, 0],
        linkJointTypes=[p.JOINT_FIXED] * 4,
        linkJointAxis=[[0, 0, 1]] * 4
    )
    return drone_id


# =====================================================================
# 4. MAIN INTERACTIVE SIMULATION ENGINE
# =====================================================================
def main():
    print("="*75)
    print("  6-DOF QUADROTOR QUATERNION FLIGHT SIMULATOR (GOLABEK ET AL. 2022)")
    print("  Direct Native PyBullet Engine + Interactive GUI Controls")
    print("="*75)

    # 1. Connect to PyBullet GUI with clean viewing configuration
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.resetDebugVisualizerCamera(cameraDistance=3.8, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 1.5])
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")

    # 2. Add Native Interactive Control Buttons & Sliders in PyBullet
    param_play = p.addUserDebugParameter(" [1] PLAY / PAUSE (Click to Toggle)", 1, 0, 0)
    param_reset = p.addUserDebugParameter(" [2] RESET SIMULATION (Click to Re-arm)", 1, 0, 0)
    param_mode = p.addUserDebugParameter(" [3] TRAJECTORY (0: Fig-8 | 1: Circle | 2: Hover)", 0, 2, 0)
    param_speed = p.addUserDebugParameter(" [4] FLIGHT SPEED MULTIPLIER", 0.5, 2.5, 1.0)
    param_alt = p.addUserDebugParameter(" [5] ALTITUDE Z (m)", 0.5, 3.0, 1.5)

    # State variables for button click detection
    prev_play_clicks = 0
    prev_reset_clicks = 0
    is_running = True
    current_mode = 0

    init_pos = np.array([0.0, 0.0, 1.5])
    drone_id = create_quadrotor_multibody(init_pos)
    ctrl = NotebookQuaternionController(mass=0.035, gravity=9.81)

    # 3. Setup Synchronized Live Telemetry Plot
    plt.ion()
    fig, (ax_r, ax_p, ax_y) = plt.subplots(3, 1, figsize=(6, 7))
    fig.canvas.manager.set_window_title('Live 3-Axis Attitude Telemetry (Golabek et al. Q_P)')
    
    for ax, title, col in zip([ax_r, ax_p, ax_y], ['Roll φ [deg]', 'Pitch θ [deg]', 'Yaw ψ [deg]'], ['#FF5252', '#69F0AE', '#448AFF']):
        ax.set_title(title, fontweight='bold', fontsize=10)
        ax.set_xlim(0, 15)
        ax.grid(True, linestyle='--', alpha=0.6)
    ax_r.set_ylim(-35, 35)
    ax_p.set_ylim(-35, 35)
    ax_y.set_ylim(-190, 190)

    line_roll, = ax_r.plot([], [], color='#FF5252', lw=2.0, label='Measured Roll')
    line_pitch, = ax_p.plot([], [], color='#69F0AE', lw=2.0, label='Measured Pitch')
    line_yaw, = ax_y.plot([], [], color='#448AFF', lw=2.0, label='Measured Yaw')
    fig.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Pre-draw Target Trajectory in PyBullet
    def redraw_target_path(mode, alt):
        p.removeAllUserDebugItems()
        # Redraw parameters need re-registering if debug items removed, but user lines don't affect params
        theta_samples = np.linspace(0, 2*math.pi, 200)
        xs, ys, zs = [], [], []
        for th in theta_samples:
            if mode == 0:
                xs.append(2.0 * math.sin(th))
                ys.append(2.0 * math.sin(th) * math.cos(th))
                zs.append(alt)
            elif mode == 1:
                xs.append(2.0 * math.cos(th))
                ys.append(2.0 * math.sin(th))
                zs.append(alt)
            else:
                xs.append(0.0)
                ys.append(0.0)
                zs.append(alt)

        for j in range(len(xs) - 1):
            p.addUserDebugLine([xs[j], ys[j], zs[j]], [xs[j+1], ys[j+1], zs[j+1]], [1, 0, 0], 2.5, 0)

    redraw_target_path(0, 1.5)

    sim_time = 0.0
    dt = 1.0 / 240.0  # 240 Hz PyBullet physics rate
    t_hist, roll_hist, pitch_hist, yaw_hist = [], [], [], []
    prev_drone_pos = init_pos.copy()
    step_count = 0

    print(">> Simulator Running! Use PyBullet GUI on top-right to Pause, Resume, or Reset.")

    # 4. Main Simulation Loop
    try:
        while p.isConnected():
            # Check Play/Pause Click
            play_clicks = p.readUserDebugParameter(param_play)
            if play_clicks != prev_play_clicks:
                prev_play_clicks = play_clicks
                is_running = not is_running
                print(f"[STATUS] Simulation {'RESUMED' if is_running else 'PAUSED'}")

            # Check Reset Click
            reset_clicks = p.readUserDebugParameter(param_reset)
            if reset_clicks != prev_reset_clicks:
                prev_reset_clicks = reset_clicks
                sim_time = 0.0
                ctrl.reset_all_states()
                p.resetBasePositionAndOrientation(drone_id, init_pos, [0, 0, 0, 1])
                p.resetBaseVelocity(drone_id, [0, 0, 0], [0, 0, 0])
                prev_drone_pos = init_pos.copy()
                t_hist.clear()
                roll_hist.clear()
                pitch_hist.clear()
                yaw_hist.clear()
                p.removeAllUserDebugItems()
                mode_val = int(round(p.readUserDebugParameter(param_mode)))
                alt_val = p.readUserDebugParameter(param_alt)
                redraw_target_path(mode_val, alt_val)
                print("[STATUS] Simulation RESET to initial condition.")

            # Read Mode & Alt sliders
            mode_val = int(round(p.readUserDebugParameter(param_mode)))
            speed_val = p.readUserDebugParameter(param_speed)
            alt_val = p.readUserDebugParameter(param_alt)

            if mode_val != current_mode:
                current_mode = mode_val
                redraw_target_path(current_mode, alt_val)

            if is_running:
                # 1. Trajectory Setpoint
                target_pos, target_vel, target_acc, target_yaw = get_trajectory_setpoint(
                    current_mode, sim_time, speed=speed_val, alt=alt_val, scale=2.0
                )

                # 2. Get PyBullet Drone State
                drone_pos, drone_orn = p.getBasePositionAndOrientation(drone_id)
                cur_pos = np.array(drone_pos)
                cur_quat_xyzw = np.array(drone_orn)  # [x, y, z, w]

                lin_vel, ang_vel_world = p.getBaseVelocity(drone_id)
                cur_vel = np.array(lin_vel)
                R_mat = R.from_quat(cur_quat_xyzw).as_matrix()
                cur_ang_vel_body = R_mat.T @ np.array(ang_vel_world)

                # 3. Compute Control Law (Golabek et al. Q_P)
                thrust_scalar, torques, q_sp, q_err_short, pos_e = ctrl.compute_control(
                    dt, cur_pos, cur_vel, cur_quat_xyzw, cur_ang_vel_body,
                    target_pos, target_vel, target_acc, target_yaw
                )

                # 4. Apply Forces & Moments to PyBullet Rigid-Body
                thrust_world = R_mat @ np.array([0.0, 0.0, thrust_scalar])
                p.applyExternalForce(drone_id, -1, thrust_world, [0, 0, 0], p.WORLD_FRAME)
                p.applyExternalTorque(drone_id, -1, R_mat @ torques, p.WORLD_FRAME)

                # Aerodynamic Damping
                p.applyExternalForce(drone_id, -1, -0.015 * cur_vel, [0, 0, 0], p.WORLD_FRAME)
                p.applyExternalTorque(drone_id, -1, -0.0005 * np.array(ang_vel_world), p.WORLD_FRAME)

                # Step Physics
                p.stepSimulation()
                sim_time += dt

                # Draw Blue Trailing Path
                if step_count % 8 == 0:
                    p.addUserDebugLine(prev_drone_pos, cur_pos, [0, 0.8, 1], 2.0, 0)
                    prev_drone_pos = cur_pos.copy()

                # Record Telemetry for Plotting
                euler = R.from_quat(cur_quat_xyzw).as_euler('xyz', degrees=True)
                t_hist.append(sim_time)
                roll_hist.append(euler[0])
                pitch_hist.append(euler[1])
                yaw_hist.append(euler[2])

                if len(t_hist) > 300:
                    t_hist.pop(0)
                    roll_hist.pop(0)
                    pitch_hist.pop(0)
                    yaw_hist.pop(0)

                # 5. Flush Telemetry Plot (every 24 steps -> 10 Hz graph update)
                if step_count % 24 == 0:
                    line_roll.set_data(t_hist, roll_hist)
                    line_pitch.set_data(t_hist, pitch_hist)
                    line_yaw.set_data(t_hist, yaw_hist)
                    for ax in [ax_r, ax_p, ax_y]:
                        ax.set_xlim(max(0, sim_time - 12), sim_time + 1)
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                step_count += 1
                time.sleep(dt * 0.4)  # Smooth real-time visualization pace
            else:
                # When paused, keep rendering without stepping physics
                time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        plt.close('all')


if __name__ == "__main__":
    main()
