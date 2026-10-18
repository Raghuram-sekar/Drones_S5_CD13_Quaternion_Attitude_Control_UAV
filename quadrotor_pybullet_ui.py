"""
=============================================================================
Interactive 6-DOF Quadrotor Quaternion Flight Simulator & Control Dashboard
=============================================================================
EXACT gym-pybullet-drones implementation matching drones_pybullet_final.ipynb
with full interactive UI controls:
  - [1] PLAY / PAUSE: Click to toggle pause/fly
  - [2] RESET SIMULATION: Click to restart from origin
  - [3] TRAJECTORY SLIDER: 0 = Figure-8 | 1 = Circle | 2 = Hover
  - [4] SPEED MULTIPLIER: 0.5x to 2.0x
  - [5] ALTITUDE SLIDER: 0.5m to 3.0m
  - Live 3D Trajectory in CtrlAviary PyBullet Window
  - Synchronized Real-time Telemetry Dashboard (Roll, Pitch, Yaw)
=============================================================================
"""

import math
import time
import numpy as np
import pybullet as p
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl


class PaperQuaternionController(DSLPIDControl):
    """
    Implements the cascaded Quaternion Attitude Control System from Golabek et al. (2022):
      - Outer Guidance System (Position PD + Accel Feedforward -> Setpoint Quaternion q_sp)
      - External Attitude Loop: Q_P Attitude Controller (Eq. 10, 17-19, 27) + Saturation
      - Internal Rate Loop: 3-axis Angular Rate PI-D Controllers
    """
    def __init__(self, drone_model):
        super().__init__(drone_model=drone_model)
        
        # 1. Outer Loop Q_P Attitude Controller Parameters (Eq. 27 & Figure 1)
        self.KP_QUAT = 12.0
        self.MAX_RATE_SP = 10.0  # Saturation limit [rad/s]
        
        # 2. Inner Loop 3-Axis Angular Rate PI-D Gains (matched to Crazyflie 2.x PWM mixer)
        self.KP_RATE = np.array([4500.0, 4500.0, 3500.0])
        self.KI_RATE = np.array([250.0,  250.0,  150.0])
        self.KD_RATE = np.array([15.0,   15.0,   0.0])
        
        self.reset_all_states()

    def reset_all_states(self):
        self.reset()
        self.integral_pos_e = np.zeros(3)
        self.rate_integral = np.zeros(3)
        self.prev_omega_meas = np.zeros(3)
        self.d_omega_filt = np.zeros(3)
        self.first_step = True

    @staticmethod
    def quat_multiply(q1, q2):
        """Hamilton product (q1 * q2) for [qw, qx, qy, qz] scalar-first format (Eq. 8)."""
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
        """Unit quaternion inverse/conjugate in [qw, qx, qy, qz] format (Eq. 7)."""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def computeControlFromState(self, control_timestep, state, target_pos, target_vel, target_acc, target_yaw):
        cur_pos = state[0:3]
        cur_quat_xyzw = state[3:7]      # PyBullet format: [qx, qy, qz, qw]
        cur_vel = state[10:13]
        cur_ang_vel_world = state[13:16]

        # 1. GUIDANCE SYSTEM (GS): Position PID + Force Synthesis
        pos_e = target_pos - cur_pos
        vel_e = target_vel - cur_vel

        self.integral_pos_e = self.integral_pos_e + pos_e * control_timestep
        self.integral_pos_e = np.clip(self.integral_pos_e, -2.0, 2.0)
        self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, 0.15)

        mass = self.GRAVITY / 9.8
        target_force = (
            mass * (
                self.P_COEFF_FOR * pos_e
                + self.I_COEFF_FOR * self.integral_pos_e
                + self.D_COEFF_FOR * vel_e
                + target_acc
            )
            + np.array([0.0, 0.0, self.GRAVITY])
        )

        R_meas = R.from_quat(cur_quat_xyzw).as_matrix()
        scalar_thrust = max(0.0, np.dot(target_force, R_meas[:, 2]))
        thrust = (math.sqrt(scalar_thrust / (4.0 * self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE

        # Construct desired 3D orthonormal basis (R_sp)
        norm_tf = np.linalg.norm(target_force)
        z_body_des = target_force / norm_tf if norm_tf > 1e-4 else np.array([0.0, 0.0, 1.0])
        x_c_des = np.array([math.cos(target_yaw), math.sin(target_yaw), 0.0])
        y_body_des = np.cross(z_body_des, x_c_des)
        norm_yb = np.linalg.norm(y_body_des)
        y_body_des = y_body_des / norm_yb if norm_yb > 1e-4 else np.array([0.0, 1.0, 0.0])
        x_body_des = np.cross(y_body_des, z_body_des)

        R_sp = np.column_stack((x_body_des, y_body_des, z_body_des))
        q_sp_xyzw = R.from_matrix(R_sp).as_quat()

        q_sp = np.array([q_sp_xyzw[3], q_sp_xyzw[0], q_sp_xyzw[1], q_sp_xyzw[2]])
        q_meas = np.array([cur_quat_xyzw[3], cur_quat_xyzw[0], cur_quat_xyzw[1], cur_quat_xyzw[2]])
        q_sp /= np.linalg.norm(q_sp)
        q_meas /= np.linalg.norm(q_meas)

        # 2. Q_P ATTITUDE CONTROLLER (Equations 11, 17-19, 27)
        q_err = self.quat_multiply(self.quat_conjugate(q_meas), q_sp)
        q_err_short = -q_err if q_err[0] < 0.0 else q_err

        omega_sp = 2.0 * self.KP_QUAT * q_err_short[1:4]
        omega_sp = np.clip(omega_sp, -self.MAX_RATE_SP, self.MAX_RATE_SP)

        # 3. INNER ANGULAR RATE PI-D CONTROLLER
        omega_meas = R_meas.T @ cur_ang_vel_world
        omega_err = omega_sp - omega_meas

        self.rate_integral += omega_err * control_timestep
        self.rate_integral = np.clip(self.rate_integral, -0.5, 0.5)

        if self.first_step:
            self.d_omega_filt = np.zeros(3)
            self.first_step = False
        else:
            raw_d_omega = (omega_meas - self.prev_omega_meas) / control_timestep
            self.d_omega_filt = 0.2 * raw_d_omega + 0.8 * self.d_omega_filt
        self.prev_omega_meas = omega_meas.copy()

        target_torques = (
            self.KP_RATE * omega_err
            + self.KI_RATE * self.rate_integral
            - self.KD_RATE * self.d_omega_filt
        )
        target_torques = np.clip(target_torques, -3200.0, 3200.0)

        # Actuator Mixer into 4 Motor RPMs
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
        rpm = self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST

        return rpm, pos_e, q_err_short


def calculate_trajectory(mode, t, speed=1.0, alt=1.5, scale=2.0):
    if mode == 0:  # Figure-8
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

    else:  # Hover Waypoint
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


def run_interactive_simulation():
    INIT_XYZ = np.array([[0.0, 0.0, 1.5]])
    INIT_RPY = np.array([[0.0, 0.0, math.pi/4]])

    # 1. Initialize exact CtrlAviary Environment from drones_pybullet_final.ipynb
    env = CtrlAviary(gui=True, record=False, initial_xyzs=INIT_XYZ, initial_rpys=INIT_RPY)
    ctrl = PaperQuaternionController(drone_model=env.DRONE_MODEL)

    ctrl.P_COEFF_FOR = np.array([8.0, 8.0, 10.0])
    ctrl.I_COEFF_FOR = np.array([1.5, 1.5, 2.0])
    ctrl.D_COEFF_FOR = np.array([5.5, 5.5, 6.0])

    obs, info = env.reset()
    client = env.CLIENT

    # Position camera comfortably showing drone and trajectory
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0, 0, 1.5], physicsClientId=client)

    # 2. Add Native Interactive Control Buttons & Sliders in PyBullet Viewport
    param_play = p.addUserDebugParameter(" [1] PLAY / PAUSE (Click to Toggle)", 1, 0, 0, physicsClientId=client)
    param_reset = p.addUserDebugParameter(" [2] RESET SIMULATION (Click to Restart)", 1, 0, 0, physicsClientId=client)
    param_mode = p.addUserDebugParameter(" [3] TRAJECTORY (0: Fig-8 | 1: Circle | 2: Hover)", 0, 2, 0, physicsClientId=client)
    param_speed = p.addUserDebugParameter(" [4] FLIGHT SPEED MULTIPLIER", 0.5, 2.0, 1.0, physicsClientId=client)
    param_alt = p.addUserDebugParameter(" [5] ALTITUDE Z (m)", 0.5, 3.0, 1.5, physicsClientId=client)

    # Initial read to record baseline click state
    prev_play_clicks = p.readUserDebugParameter(param_play, physicsClientId=client)
    prev_reset_clicks = p.readUserDebugParameter(param_reset, physicsClientId=client)
    is_running = True
    current_mode = 0

    # 3. Setup Lightweight Synchronized Telemetry Dashboard
    plt.ion()
    fig, (ax_roll, ax_pitch, ax_yaw) = plt.subplots(3, 1, figsize=(5.5, 7.0))
    fig.canvas.manager.set_window_title('Live Flight Telemetry (Golabek et al. Q_P Controller)')

    for ax, title, col in zip([ax_roll, ax_pitch, ax_yaw], ['Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'], ['#FF5252', '#69F0AE', '#448AFF']):
        ax.set_title(title, fontweight='bold', fontsize=9)
        ax.set_xlim(0, 15)
        ax.grid(True, linestyle='--', alpha=0.5)
    ax_roll.set_ylim(-1.0, 1.0)
    ax_pitch.set_ylim(-1.0, 1.0)
    ax_yaw.set_ylim(-3.5, 3.5)

    line_roll, = ax_roll.plot([], [], color='#FF5252', lw=1.8)
    line_pitch, = ax_pitch.plot([], [], color='#69F0AE', lw=1.8)
    line_yaw, = ax_yaw.plot([], [], color='#448AFF', lw=1.8)

    fig.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Path lines tracking
    target_lines = []
    trail_lines = []

    def draw_path(mode, alt):
        nonlocal target_lines
        for lid in target_lines:
            try:
                p.removeUserDebugItem(lid, physicsClientId=client)
            except Exception:
                pass
        target_lines.clear()

        th_samples = np.linspace(0, 2*math.pi, 200)
        xs, ys, zs = [], [], []
        for th in th_samples:
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
            lid = p.addUserDebugLine([xs[j], ys[j], zs[j]], [xs[j+1], ys[j+1], zs[j+1]], [1, 0, 0], 2.0, 0, physicsClientId=client)
            target_lines.append(lid)

    draw_path(0, 1.5)

    time_data, roll_data, pitch_data, yaw_data = [], [], [], []
    prev_actual_pos = obs[0][0:3]
    sim_time = 0.0
    step_i = 0

    print(">> CtrlAviary Flight Simulator Active & Running smoothly!")
    print(">> Click '[1] PLAY / PAUSE' or '[2] RESET SIMULATION' on the top-right panel to interact.")

    try:
        while True:
            # Check PyBullet connection
            try:
                if not p.isConnected(client):
                    break
            except Exception:
                break

            # Play / Pause Toggle
            play_clicks = p.readUserDebugParameter(param_play, physicsClientId=client)
            if play_clicks != prev_play_clicks:
                prev_play_clicks = play_clicks
                is_running = not is_running
                print(f"[STATUS] Simulation {'RESUMED' if is_running else 'PAUSED'}")

            # Reset Trigger
            reset_clicks = p.readUserDebugParameter(param_reset, physicsClientId=client)
            if reset_clicks != prev_reset_clicks:
                prev_reset_clicks = reset_clicks
                sim_time = 0.0
                obs, info = env.reset()
                ctrl.reset_all_states()
                prev_actual_pos = obs[0][0:3]
                time_data.clear()
                roll_data.clear()
                pitch_data.clear()
                yaw_data.clear()
                for lid in trail_lines:
                    try:
                        p.removeUserDebugItem(lid, physicsClientId=client)
                    except Exception:
                        pass
                trail_lines.clear()
                m = int(round(p.readUserDebugParameter(param_mode, physicsClientId=client)))
                a = p.readUserDebugParameter(param_alt, physicsClientId=client)
                draw_path(m, a)
                print("[STATUS] Simulation RESET to initial condition.")

            # Read sliders
            m = int(round(p.readUserDebugParameter(param_mode, physicsClientId=client)))
            spd = p.readUserDebugParameter(param_speed, physicsClientId=client)
            alt = p.readUserDebugParameter(param_alt, physicsClientId=client)

            if m != current_mode:
                current_mode = m
                draw_path(current_mode, alt)

            if is_running:
                current_state = obs[0]
                current_pos = current_state[0:3]
                current_rpy = current_state[7:10]

                target_pos, target_vel, target_acc, target_yaw = calculate_trajectory(
                    current_mode, sim_time, speed=spd, alt=alt, scale=2.0
                )

                # Record Telemetry
                time_data.append(sim_time)
                roll_data.append(current_rpy[0])
                pitch_data.append(current_rpy[1])
                yaw_data.append(current_rpy[2])

                # Draw Blue Trailing Path
                if step_i % 6 == 0:
                    lid = p.addUserDebugLine(prev_actual_pos, current_pos, [0, 0, 1], 2.0, 0, physicsClientId=client)
                    trail_lines.append(lid)
                    if len(trail_lines) > 250:
                        old_lid = trail_lines.pop(0)
                        try:
                            p.removeUserDebugItem(old_lid, physicsClientId=client)
                        except Exception:
                            pass
                    prev_actual_pos = current_pos

                # Lightweight 10 Hz Telemetry Plot Refresh
                if step_i % 24 == 0:
                    try:
                        line_roll.set_data(time_data, roll_data)
                        line_pitch.set_data(time_data, pitch_data)
                        line_yaw.set_data(time_data, yaw_data)
                        for ax in [ax_roll, ax_pitch, ax_yaw]:
                            ax.set_xlim(max(0, sim_time - 12), sim_time + 1)
                        fig.canvas.flush_events()
                    except Exception:
                        pass

                # Compute exact paper control law
                action, pos_error, quat_error = ctrl.computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=current_state,
                    target_pos=target_pos,
                    target_vel=target_vel,
                    target_acc=target_acc,
                    target_yaw=target_yaw
                )

                # Step through exact Gym environment physics
                obs, reward, terminated, truncated, info = env.step(np.array([action]))
                sim_time += env.CTRL_TIMESTEP * spd
                step_i += 1
                time.sleep(env.CTRL_TIMESTEP)
            else:
                # Keep GUI responsive while paused
                fig.canvas.flush_events()
                time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exiting simulator: {e}")
    finally:
        try:
            env.close()
        except Exception:
            pass
        try:
            plt.close('all')
        except Exception:
            pass


if __name__ == "__main__":
    run_interactive_simulation()
