"""
=============================================================================
Interactive 6-DOF Quadrotor Quaternion Flight Simulator & Control Dashboard
=============================================================================
Implements:
  1. Outer Guidance System: Position PID + Analytical Accel Feedforward -> q_sp
  2. Outer Quaternion Attitude Loop: Golabek et al. (2022) Q_P Controller
     - Eq. 11: q_err = conjugate(q_meas) (x) q_sp
     - Eq. 17-19: Shortest rotation check
     - Eq. 27: w_sp = 2 * Kp * q_err_short
  3. Inner Rate Loop: 3-Axis Angular Rate PI-D Controller
  4. Real-time 6-DOF Quadrotor Physical Dynamics Engine (Inertia, Torques, RPMs)
  5. Interactive GUI Controls:
     - [Play / Pause], [Reset], [Speed Multiplier]
     - Trajectory selector: Figure-8, Circle, Helix/Spiral, Waypoints
     - Custom Start & Target Altitude sliders
     - Live 3D Trajectory & Attitude Rendering + Real-time Telemetry
=============================================================================
"""

import sys
import math
import time
import numpy as np
import tkinter as tk
from tkinter import ttk
from scipy.spatial.transform import Rotation as R

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection


# =====================================================================
# 1. 6-DOF QUADROTOR QUATERNION CONTROLLER (Golabek et al. 2022)
# =====================================================================
class QuadrotorQuaternionController:
    def __init__(self, mass=0.030, gravity=9.81):
        self.mass = mass
        self.g = gravity
        
        # Position Loop PID Gains (m/s^2 per m and m/s error)
        self.KP_POS = np.array([6.5, 6.5, 9.0])
        self.KI_POS = np.array([1.0, 1.0, 1.8])
        self.KD_POS = np.array([4.2, 4.2, 5.5])
        self.pos_integral = np.zeros(3)

        # Outer Loop Q_P Controller (Eq. 27)
        self.KP_QUAT = 12.0
        self.MAX_RATE_SP = 12.0  # rad/s

        # Inner Loop Angular Rate PI-D Gains
        self.KP_RATE = np.array([0.08, 0.08, 0.06])
        self.KI_RATE = np.array([0.01, 0.01, 0.005])
        self.KD_RATE = np.array([0.003, 0.003, 0.001])
        
        self.rate_integral = np.zeros(3)
        self.prev_omega_meas = np.zeros(3)
        self.d_omega_filt = np.zeros(3)
        self.first_step = True

    def reset(self):
        self.pos_integral = np.zeros(3)
        self.rate_integral = np.zeros(3)
        self.prev_omega_meas = np.zeros(3)
        self.d_omega_filt = np.zeros(3)
        self.first_step = True

    @staticmethod
    def quat_multiply(q1, q2):
        """Hamilton product (q1 * q2) for [qw, qx, qy, qz] scalar-first format."""
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
        """Unit quaternion conjugate [qw, -qx, -qy, -qz]."""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def compute_control(self, dt, pos, vel, quat, omega_body, target_pos, target_vel, target_acc, target_yaw):
        # 1. Position PID & Force Synthesis
        pos_e = target_pos - pos
        vel_e = target_vel - vel

        self.pos_integral += pos_e * dt
        self.pos_integral = np.clip(self.pos_integral, -1.5, 1.5)

        target_force = (
            self.mass * (
                self.KP_POS * pos_e
                + self.KI_POS * self.pos_integral
                + self.KD_POS * vel_e
                + target_acc
            )
            + np.array([0.0, 0.0, self.mass * self.g])
        )

        # 2. Orthonormal Rotation Basis (Z_body aligns with target_force)
        norm_force = np.linalg.norm(target_force)
        if norm_force < 1e-4:
            z_body_des = np.array([0.0, 0.0, 1.0])
        else:
            z_body_des = target_force / norm_force

        x_c_des = np.array([math.cos(target_yaw), math.sin(target_yaw), 0.0])
        y_body_des = np.cross(z_body_des, x_c_des)
        norm_y = np.linalg.norm(y_body_des)
        if norm_y < 1e-4:
            y_body_des = np.array([0.0, 1.0, 0.0])
        else:
            y_body_des /= norm_y
        x_body_des = np.cross(y_body_des, z_body_des)

        R_sp = np.column_stack((x_body_des, y_body_des, z_body_des))
        q_sp_xyzw = R.from_matrix(R_sp).as_quat()
        q_sp = np.array([q_sp_xyzw[3], q_sp_xyzw[0], q_sp_xyzw[1], q_sp_xyzw[2]])
        q_sp /= np.linalg.norm(q_sp)

        # 3. Outer Q_P Attitude Loop (Eq. 11, 17-19, 27)
        q_meas = quat / np.linalg.norm(quat)
        q_err = self.quat_multiply(self.quat_conjugate(q_meas), q_sp)

        if q_err[0] < 0.0:
            q_err_short = -q_err
        else:
            q_err_short = q_err

        omega_sp = 2.0 * self.KP_QUAT * q_err_short[1:4]
        omega_sp = np.clip(omega_sp, -self.MAX_RATE_SP, self.MAX_RATE_SP)

        # 4. Inner Angular Rate PI-D Controller
        omega_err = omega_sp - omega_body
        self.rate_integral += omega_err * dt
        self.rate_integral = np.clip(self.rate_integral, -0.3, 0.3)

        if self.first_step:
            self.d_omega_filt = np.zeros(3)
            self.first_step = False
        else:
            raw_d = (omega_body - self.prev_omega_meas) / dt
            self.d_omega_filt = 0.25 * raw_d + 0.75 * self.d_omega_filt
        self.prev_omega_meas = omega_body.copy()

        torques = (
            self.KP_RATE * omega_err
            + self.KI_RATE * self.rate_integral
            - self.KD_RATE * self.d_omega_filt
        )

        # Collective Thrust along measured Body Z
        R_meas = R.from_quat([q_meas[1], q_meas[2], q_meas[3], q_meas[0]]).as_matrix()
        total_thrust = max(0.0, float(np.dot(target_force, R_meas[:, 2])))

        return total_thrust, torques, q_sp, q_err_short, pos_e


# =====================================================================
# 2. 6-DOF QUADROTOR DYNAMICS SIMULATOR
# =====================================================================
class QuadrotorDynamics:
    def __init__(self, mass=0.030, init_pos=None):
        self.mass = mass
        self.g = 9.81
        self.J = np.diag([1.4e-5, 1.4e-5, 2.2e-5])
        self.J_inv = np.linalg.inv(self.J)
        
        self.init_pos = np.array([0.0, 0.0, 1.5]) if init_pos is None else np.array(init_pos, dtype=float)
        self.reset()

    def reset(self, init_pos=None):
        if init_pos is not None:
            self.init_pos = np.array(init_pos, dtype=float)
        self.pos = self.init_pos.copy()
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]
        self.omega = np.zeros(3)                     # body rate [p, q, r]

    def step(self, dt, thrust, torques):
        # Rotation matrix from body to world
        R_mat = R.from_quat([self.quat[1], self.quat[2], self.quat[3], self.quat[0]]).as_matrix()

        # Translational Dynamics: m * a = R * [0, 0, T] - [0, 0, m*g] - drag
        force_body = np.array([0.0, 0.0, thrust])
        force_world = R_mat @ force_body - np.array([0.0, 0.0, self.mass * self.g])
        drag = 0.02 * self.vel
        acc_world = (force_world - drag) / self.mass

        self.vel += acc_world * dt
        self.pos += self.vel * dt

        # Ground collision constraint
        if self.pos[2] < 0.0:
            self.pos[2] = 0.0
            self.vel = np.zeros(3)

        # Rotational Dynamics: J * w_dot + w x (J * w) = torques - drag
        gyro_moment = np.cross(self.omega, self.J @ self.omega)
        damping = 0.001 * self.omega
        omega_dot = self.J_inv @ (torques - gyro_moment - damping)
        self.omega += omega_dot * dt

        # Quaternion Kinematics: q_dot = 0.5 * q (x) [0, omega]
        qw, qx, qy, qz = self.quat
        p, q, r_rate = self.omega
        q_dot = 0.5 * np.array([
            -qx*p - qy*q - qz*r_rate,
             qw*p + qy*r_rate - qz*q,
             qw*q - qx*r_rate + qz*p,
             qw*r_rate + qx*q - qy*p
        ])

        self.quat += q_dot * dt
        self.quat /= np.linalg.norm(self.quat)


# =====================================================================
# 3. CONFIGURABLE TRAJECTORY GENERATOR
# =====================================================================
class TrajectoryGenerator:
    @staticmethod
    def get_setpoint(mode, t, base_alt=1.5, scale=2.0, speed=1.0):
        if mode == "Figure-8 (Lemniscate)":
            theta = speed * t * 0.75
            theta_dot = speed * 0.75
            theta_ddot = 0.0

            x = scale * math.sin(theta)
            y = scale * math.sin(theta) * math.cos(theta)
            z = base_alt

            vx = scale * math.cos(theta) * theta_dot
            vy = scale * math.cos(2.0 * theta) * theta_dot
            vz = 0.0

            ax = -scale * math.sin(theta) * (theta_dot**2)
            ay = -2.0 * scale * math.sin(2.0 * theta) * (theta_dot**2)
            az = 0.0

        elif mode == "Circle":
            theta = speed * t * 0.8
            theta_dot = speed * 0.8

            x = scale * math.cos(theta)
            y = scale * math.sin(theta)
            z = base_alt

            vx = -scale * math.sin(theta) * theta_dot
            vy = scale * math.cos(theta) * theta_dot
            vz = 0.0

            ax = -scale * math.cos(theta) * (theta_dot**2)
            ay = -scale * math.sin(theta) * (theta_dot**2)
            az = 0.0

        elif mode == "Helix / Spiral":
            theta = speed * t * 0.8
            theta_dot = speed * 0.8

            x = scale * math.cos(theta)
            y = scale * math.sin(theta)
            z = base_alt + 0.5 * math.sin(0.4 * speed * t)

            vx = -scale * math.sin(theta) * theta_dot
            vy = scale * math.cos(theta) * theta_dot
            vz = 0.5 * 0.4 * speed * math.cos(0.4 * speed * t)

            ax = -scale * math.cos(theta) * (theta_dot**2)
            ay = -scale * math.sin(theta) * (theta_dot**2)
            az = -0.5 * (0.4 * speed)**2 * math.sin(0.4 * speed * t)

        elif mode == "Waypoint Square":
            side = scale * 1.5
            period = 8.0 / speed
            phase = (t % period) / period
            z = base_alt

            if phase < 0.25:
                # 0 to +X
                s = phase / 0.25
                x = -side/2 + s * side
                y = -side/2
                vx = side / (0.25 * period)
                vy = 0.0
            elif phase < 0.50:
                # +X to +Y
                s = (phase - 0.25) / 0.25
                x = side/2
                y = -side/2 + s * side
                vx = 0.0
                vy = side / (0.25 * period)
            elif phase < 0.75:
                # -X
                s = (phase - 0.50) / 0.25
                x = side/2 - s * side
                y = side/2
                vx = -side / (0.25 * period)
                vy = 0.0
            else:
                # -Y back to start
                s = (phase - 0.75) / 0.25
                x = -side/2
                y = side/2 - s * side
                vx = 0.0
                vy = -side / (0.25 * period)
            vz = 0.0
            ax, ay, az = 0.0, 0.0, 0.0
        else:
            x, y, z = 0.0, 0.0, base_alt
            vx, vy, vz = 0.0, 0.0, 0.0
            ax, ay, az = 0.0, 0.0, 0.0

        target_pos = np.array([x, y, z])
        target_vel = np.array([vx, vy, vz])
        target_acc = np.array([ax, ay, az])

        if abs(vx) > 1e-3 or abs(vy) > 1e-3:
            target_yaw = math.atan2(vy, vx)
        else:
            target_yaw = 0.0

        return target_pos, target_vel, target_acc, target_yaw


# =====================================================================
# 4. MODERN TKINTER DASHBOARD APPLICATION
# =====================================================================
class DroneFlightApp:
    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Quaternion Attitude Control System (Golabek et al. 2022)")
        self.root.geometry("1300x820")
        self.root.configure(bg="#12131C")

        # Flight State & Parameters
        self.is_running = False
        self.sim_time = 0.0
        self.dt = 0.01  # 100 Hz physics/control tick
        self.speed_mult = 1.0
        self.base_alt = 1.5
        self.path_scale = 2.0
        self.traj_mode = "Figure-8 (Lemniscate)"

        # Initialize Physics & Controller
        self.drone = QuadrotorDynamics(mass=0.030, init_pos=[0.0, 0.0, 1.5])
        self.controller = QuadrotorQuaternionController(mass=0.030)

        # History Buffers for Visualization
        self.max_trail = 300
        self.actual_trail_x = []
        self.actual_trail_y = []
        self.actual_trail_z = []

        self.telemetry_time = []
        self.telemetry_roll = []
        self.telemetry_pitch = []
        self.telemetry_yaw = []
        self.telemetry_pos_err = []

        self._build_gui()
        self._precalc_trajectory_preview()
        self._update_telemetry_labels(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), 0.0)

        # Start master update loop (16ms -> ~60 FPS GUI)
        self.root.after(16, self._sim_step)

    def _build_gui(self):
        # Top Title Header
        header_frame = tk.Frame(self.root, bg="#1B1C2A", height=50)
        header_frame.pack(fill=tk.X, side=tk.TOP, padx=10, pady=(8, 4))

        title_lbl = tk.Label(
            header_frame,
            text="QUATERNION ATTITUDE CONTROL SYSTEM (Q_P CONTROLLER) — 6-DOF QUADROTOR FLIGHT SIMULATOR",
            font=("Segoe UI", 12, "bold"),
            bg="#1B1C2A",
            fg="#00E5FF"
        )
        title_lbl.pack(side=tk.LEFT, padx=15, pady=8)

        sub_lbl = tk.Label(
            header_frame,
            text="Michał Gołąbek et al. (MDPI Electronics 2022) | Real-time SITL",
            font=("Segoe UI", 9, "italic"),
            bg="#1B1C2A",
            fg="#8E92AC"
        )
        sub_lbl.pack(side=tk.RIGHT, padx=15, pady=8)

        # Main Layout Container
        main_container = tk.Frame(self.root, bg="#12131C")
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Left Column: Control Panel & Live Numbers
        sidebar = tk.Frame(main_container, bg="#1B1C2A", width=340)
        sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8), pady=0)
        sidebar.pack_propagate(False)

        self._build_sidebar(sidebar)

        # Right Column: Matplotlib 3D Viewport & Live Graphs
        viz_panel = tk.Frame(main_container, bg="#12131C")
        viz_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self._build_visualizations(viz_panel)

    def _build_sidebar(self, parent):
        pad_opts = {'padx': 12, 'pady': 6}

        # 1. Primary Action Buttons
        btn_box = tk.LabelFrame(parent, text="FLIGHT CONTROL EXECUTION", bg="#1B1C2A", fg="#00E5FF", font=("Segoe UI", 9, "bold"))
        btn_box.pack(fill=tk.X, **pad_opts)

        btn_row = tk.Frame(btn_box, bg="#1B1C2A")
        btn_row.pack(fill=tk.X, padx=8, pady=8)

        self.btn_play = tk.Button(
            btn_row, text="▶ START", bg="#00C853", fg="#FFFFFF",
            font=("Segoe UI", 10, "bold"), relief=tk.FLAT, width=9, cursor="hand2",
            command=self.toggle_play
        )
        self.btn_play.pack(side=tk.LEFT, padx=4)

        self.btn_pause = tk.Button(
            btn_row, text="⏸ PAUSE", bg="#FF9100", fg="#FFFFFF",
            font=("Segoe UI", 10, "bold"), relief=tk.FLAT, width=9, cursor="hand2",
            command=self.pause_sim
        )
        self.btn_pause.pack(side=tk.LEFT, padx=4)

        self.btn_reset = tk.Button(
            btn_row, text="🔄 RESET", bg="#D50000", fg="#FFFFFF",
            font=("Segoe UI", 10, "bold"), relief=tk.FLAT, width=9, cursor="hand2",
            command=self.reset_sim
        )
        self.btn_reset.pack(side=tk.LEFT, padx=4)

        # 2. Trajectory Mode Selector
        traj_box = tk.LabelFrame(parent, text="TRAJECTORY PATH SELECTOR", bg="#1B1C2A", fg="#00E5FF", font=("Segoe UI", 9, "bold"))
        traj_box.pack(fill=tk.X, **pad_opts)

        self.traj_var = tk.StringVar(value=self.traj_mode)
        traj_options = ["Figure-8 (Lemniscate)", "Circle", "Helix / Spiral", "Waypoint Square"]
        self.traj_combo = ttk.Combobox(traj_box, textvariable=self.traj_var, values=traj_options, state="readonly", font=("Segoe UI", 10))
        self.traj_combo.pack(fill=tk.X, padx=10, pady=8)
        self.traj_combo.bind("<<ComboboxSelected>>", self._on_traj_change)

        # 3. Interactive Flight Sliders
        slider_box = tk.LabelFrame(parent, text="FLIGHT PARAMETERS", bg="#1B1C2A", fg="#00E5FF", font=("Segoe UI", 9, "bold"))
        slider_box.pack(fill=tk.X, **pad_opts)

        # Speed Slider
        tk.Label(slider_box, text="Sim Speed Multiplier:", bg="#1B1C2A", fg="#CFD8DC", font=("Segoe UI", 9)).pack(anchor=tk.W, padx=10, pady=(4, 0))
        self.speed_slider = tk.Scale(
            slider_box, from_=0.2, to=3.0, resolution=0.1, orient=tk.HORIZONTAL,
            bg="#1B1C2A", fg="#FFFFFF", highlightthickness=0, troughcolor="#2A2B3D",
            command=self._on_speed_change
        )
        self.speed_slider.set(1.0)
        self.speed_slider.pack(fill=tk.X, padx=10, pady=(0, 4))

        # Altitude Slider
        tk.Label(slider_box, text="Flight Altitude Z (m):", bg="#1B1C2A", fg="#CFD8DC", font=("Segoe UI", 9)).pack(anchor=tk.W, padx=10, pady=(4, 0))
        self.alt_slider = tk.Scale(
            slider_box, from_=0.5, to=4.0, resolution=0.1, orient=tk.HORIZONTAL,
            bg="#1B1C2A", fg="#FFFFFF", highlightthickness=0, troughcolor="#2A2B3D",
            command=self._on_alt_change
        )
        self.alt_slider.set(1.5)
        self.alt_slider.pack(fill=tk.X, padx=10, pady=(0, 4))

        # Path Scale Slider
        tk.Label(slider_box, text="Path Radius / Scale (m):", bg="#1B1C2A", fg="#CFD8DC", font=("Segoe UI", 9)).pack(anchor=tk.W, padx=10, pady=(4, 0))
        self.scale_slider = tk.Scale(
            slider_box, from_=1.0, to=4.0, resolution=0.2, orient=tk.HORIZONTAL,
            bg="#1B1C2A", fg="#FFFFFF", highlightthickness=0, troughcolor="#2A2B3D",
            command=self._on_scale_change
        )
        self.scale_slider.set(2.0)
        self.scale_slider.pack(fill=tk.X, padx=10, pady=(0, 6))

        # 4. Live Telemetry Stream Box
        telem_box = tk.LabelFrame(parent, text="LIVE TELEMETRY STREAM", bg="#1B1C2A", fg="#00E5FF", font=("Segoe UI", 9, "bold"))
        telem_box.pack(fill=tk.BOTH, expand=True, **pad_opts)

        self.lbl_telem = tk.Label(
            telem_box,
            text="Initial State...",
            font=("Consolas", 8),
            bg="#0D0E15",
            fg="#76FF03",
            justify=tk.LEFT,
            anchor="nw",
            relief=tk.SUNKEN,
            padx=8,
            pady=8
        )
        self.lbl_telem.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

    def _build_visualizations(self, parent):
        # Matplotlib Figure with 2 Subplots:
        # Left: 3D Trajectory & Quadrotor Attitude
        # Right: Live 2D Telemetry Curves (Roll, Pitch, Yaw & Tracking Error)
        self.fig = Figure(figsize=(9.5, 7.5), facecolor="#12131C")
        self.fig.subplots_adjust(left=0.05, right=0.96, top=0.94, bottom=0.08, wspace=0.22)

        # 3D Spatial Subplot
        self.ax3d = self.fig.add_subplot(1, 2, 1, projection='3d')
        self.ax3d.set_facecolor("#12131C")
        self.ax3d.set_title("3D SPATIAL QUADROTOR FLIGHT", color="#00E5FF", fontsize=11, fontweight="bold", pad=10)
        self.ax3d.set_xlabel("X (m)", color="#8E92AC", labelpad=5)
        self.ax3d.set_ylabel("Y (m)", color="#8E92AC", labelpad=5)
        self.ax3d.set_zlabel("Z (m)", color="#8E92AC", labelpad=5)
        self.ax3d.tick_params(colors="#8E92AC", labelsize=8)

        # 2D Attitude & Error Subplots (Right side split into 2)
        self.ax_rpy = self.fig.add_subplot(2, 2, 2)
        self.ax_rpy.set_facecolor("#1B1C2A")
        self.ax_rpy.set_title("LIVE ATTITUDE ANGLES (deg)", color="#00E5FF", fontsize=10, fontweight="bold")
        self.ax_rpy.grid(True, color="#2E3048", linestyle="--", alpha=0.7)
        self.ax_rpy.tick_params(colors="#8E92AC", labelsize=8)

        self.ax_err = self.fig.add_subplot(2, 2, 4)
        self.ax_err.set_facecolor("#1B1C2A")
        self.ax_err.set_title("3D POSITION TRACKING ERROR ||e_pos|| (m)", color="#00E5FF", fontsize=10, fontweight="bold")
        self.ax_err.set_xlabel("Time (s)", color="#8E92AC", fontsize=9)
        self.ax_err.grid(True, color="#2E3048", linestyle="--", alpha=0.7)
        self.ax_err.tick_params(colors="#8E92AC", labelsize=8)

        # Static 3D Target Path Line
        self.line_target_path, = self.ax3d.plot([], [], [], 'r--', lw=1.8, label="Target Trajectory", alpha=0.85)
        # Dynamic Actual Drone Trajectory Trail
        self.line_actual_trail, = self.ax3d.plot([], [], [], color="#00E5FF", lw=2.2, label="Actual Drone Trail")
        # Target Point Marker
        self.point_target, = self.ax3d.plot([], [], [], 'mo', markersize=6, label="Target Setpoint")

        # 3D Quadrotor Body Arms (Cross configuration)
        self.drone_arm1, = self.ax3d.plot([], [], [], color="#FFFFFF", lw=3.0)
        self.drone_arm2, = self.ax3d.plot([], [], [], color="#FFFFFF", lw=3.0)
        self.rotor_pts, = self.ax3d.plot([], [], [], 'go', markersize=5)

        self.ax3d.legend(loc="upper right", facecolor="#1B1C2A", edgecolor="#2E3048", fontsize=7, labelcolor="#FFFFFF")

        # 2D Telemetry Lines
        self.line_roll, = self.ax_rpy.plot([], [], color="#FF5252", lw=1.5, label="Roll φ")
        self.line_pitch, = self.ax_rpy.plot([], [], color="#69F0AE", lw=1.5, label="Pitch θ")
        self.line_yaw, = self.ax_rpy.plot([], [], color="#448AFF", lw=1.5, label="Yaw ψ")
        self.ax_rpy.legend(loc="upper right", facecolor="#1B1C2A", edgecolor="#2E3048", fontsize=7, labelcolor="#FFFFFF")

        self.line_pos_err, = self.ax_err.plot([], [], color="#FFD700", lw=1.6, label="Pos Error")
        self.ax_err.legend(loc="upper right", facecolor="#1B1C2A", edgecolor="#2E3048", fontsize=7, labelcolor="#FFFFFF")

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

    def _precalc_trajectory_preview(self):
        """Draws the target path in the 3D space based on selected mode and scale."""
        t_samples = np.linspace(0, 15, 300)
        xs, ys, zs = [], [], []
        for ts in t_samples:
            p_des, _, _, _ = TrajectoryGenerator.get_setpoint(
                self.traj_mode, ts, base_alt=self.base_alt, scale=self.path_scale, speed=1.0
            )
            xs.append(p_des[0])
            ys.append(p_des[1])
            zs.append(p_des[2])

        self.line_target_path.set_data(xs, ys)
        self.line_target_path.set_3d_properties(zs)

        pad = self.path_scale + 0.8
        self.ax3d.set_xlim([-pad, pad])
        self.ax3d.set_ylim([-pad, pad])
        self.ax3d.set_zlim([0.0, max(3.5, self.base_alt + 1.2)])

    def _on_traj_change(self, event=None):
        self.traj_mode = self.traj_var.get()
        self.reset_sim()

    def _on_speed_change(self, val):
        self.speed_mult = float(val)

    def _on_alt_change(self, val):
        self.base_alt = float(val)
        self._precalc_trajectory_preview()

    def _on_scale_change(self, val):
        self.path_scale = float(val)
        self._precalc_trajectory_preview()

    def toggle_play(self):
        self.is_running = True
        self.btn_play.configure(bg="#2E7D32")
        self.btn_pause.configure(bg="#FF9100")

    def pause_sim(self):
        self.is_running = False
        self.btn_play.configure(bg="#00C853")
        self.btn_pause.configure(bg="#E65100")

    def reset_sim(self):
        self.is_running = False
        self.sim_time = 0.0
        self.btn_play.configure(bg="#00C853")
        self.btn_pause.configure(bg="#FF9100")

        # Get initial starting point for selected trajectory
        p0, _, _, _ = TrajectoryGenerator.get_setpoint(
            self.traj_mode, 0.0, base_alt=self.base_alt, scale=self.path_scale, speed=self.speed_mult
        )
        self.drone.reset(init_pos=p0)
        self.controller.reset()

        self.actual_trail_x.clear()
        self.actual_trail_y.clear()
        self.actual_trail_z.clear()

        self.telemetry_time.clear()
        self.telemetry_roll.clear()
        self.telemetry_pitch.clear()
        self.telemetry_yaw.clear()
        self.telemetry_pos_err.clear()

        self._precalc_trajectory_preview()
        self._update_telemetry_labels(p0, self.drone.quat, 0.0)
        self._redraw_visuals(p0, self.drone.quat, p0)

    def _sim_step(self):
        if self.is_running:
            # Advance simulation by dt * speed_mult
            target_pos, target_vel, target_acc, target_yaw = TrajectoryGenerator.get_setpoint(
                self.traj_mode, self.sim_time, base_alt=self.base_alt, scale=self.path_scale, speed=self.speed_mult
            )

            thrust, torques, q_sp, q_err_short, pos_e = self.controller.compute_control(
                self.dt,
                self.drone.pos,
                self.drone.vel,
                self.drone.quat,
                self.drone.omega,
                target_pos,
                target_vel,
                target_acc,
                target_yaw
            )

            self.drone.step(self.dt, thrust, torques)
            self.sim_time += self.dt * self.speed_mult

            # Record History
            self.actual_trail_x.append(self.drone.pos[0])
            self.actual_trail_y.append(self.drone.pos[1])
            self.actual_trail_z.append(self.drone.pos[2])
            if len(self.actual_trail_x) > self.max_trail:
                self.actual_trail_x.pop(0)
                self.actual_trail_y.pop(0)
                self.actual_trail_z.pop(0)

            # Euler Angles for Telemetry
            r_obj = R.from_quat([self.drone.quat[1], self.drone.quat[2], self.drone.quat[3], self.drone.quat[0]])
            euler = r_obj.as_euler('xyz', degrees=True)

            self.telemetry_time.append(self.sim_time)
            self.telemetry_roll.append(euler[0])
            self.telemetry_pitch.append(euler[1])
            self.telemetry_yaw.append(euler[2])
            self.telemetry_pos_err.append(np.linalg.norm(pos_e))

            if len(self.telemetry_time) > 200:
                self.telemetry_time.pop(0)
                self.telemetry_roll.pop(0)
                self.telemetry_pitch.pop(0)
                self.telemetry_yaw.pop(0)
                self.telemetry_pos_err.pop(0)

            # Update GUI Elements
            self._update_telemetry_labels(self.drone.pos, self.drone.quat, np.linalg.norm(pos_e))
            self._redraw_visuals(self.drone.pos, self.drone.quat, target_pos)

        self.root.after(16, self._sim_step)

    def _redraw_visuals(self, drone_pos, quat, target_pos):
        # 1. Update Actual Trail
        self.line_actual_trail.set_data(self.actual_trail_x, self.actual_trail_y)
        self.line_actual_trail.set_3d_properties(self.actual_trail_z)

        # 2. Update Target Marker
        self.point_target.set_data([target_pos[0]], [target_pos[1]])
        self.point_target.set_3d_properties([target_pos[2]])

        # 3. Draw 3D Quadrotor Cross-Arms
        # Arm length: 0.18m
        L = 0.22
        R_mat = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        arm1_body = np.array([[-L, 0, 0], [L, 0, 0]]).T
        arm2_body = np.array([[0, -L, 0], [0, L, 0]]).T
        rotors_body = np.array([[-L, 0, 0], [L, 0, 0], [0, -L, 0], [0, L, 0]]).T

        arm1_world = R_mat @ arm1_body + drone_pos[:, None]
        arm2_world = R_mat @ arm2_body + drone_pos[:, None]
        rotors_world = R_mat @ rotors_body + drone_pos[:, None]

        self.drone_arm1.set_data(arm1_world[0, :], arm1_world[1, :])
        self.drone_arm1.set_3d_properties(arm1_world[2, :])

        self.drone_arm2.set_data(arm2_world[0, :], arm2_world[1, :])
        self.drone_arm2.set_3d_properties(arm2_world[2, :])

        self.rotor_pts.set_data(rotors_world[0, :], rotors_world[1, :])
        self.rotor_pts.set_3d_properties(rotors_world[2, :])

        # 4. Update 2D Telemetry Graphs
        if len(self.telemetry_time) > 1:
            t_win = self.telemetry_time
            self.line_roll.set_data(t_win, self.telemetry_roll)
            self.line_pitch.set_data(t_win, self.telemetry_pitch)
            self.line_yaw.set_data(t_win, self.telemetry_yaw)

            self.ax_rpy.set_xlim([t_win[0], t_win[-1]])
            y_min = min(min(self.telemetry_roll), min(self.telemetry_pitch), min(self.telemetry_yaw), -10.0) - 5
            y_max = max(max(self.telemetry_roll), max(self.telemetry_pitch), max(self.telemetry_yaw), 10.0) + 5
            self.ax_rpy.set_ylim([y_min, y_max])

            self.line_pos_err.set_data(t_win, self.telemetry_pos_err)
            self.ax_err.set_xlim([t_win[0], t_win[-1]])
            err_max = max(max(self.telemetry_pos_err), 0.2) + 0.05
            self.ax_err.set_ylim([0.0, err_max])

        self.canvas.draw_idle()

    def _update_telemetry_labels(self, pos, quat, pos_err_norm):
        r_obj = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        euler = r_obj.as_euler('xyz', degrees=True)
        status_text = "FLYING [ACTIVE]" if self.is_running else "PAUSED / READY"
        status_color = "#00E5FF" if self.is_running else "#FF9100"

        text = (
            f"STATUS: {status_text}\n"
            f"SIM TIME: {self.sim_time:6.2f} s\n"
            f"SPEED   : {self.speed_mult:4.1f} x\n"
            f"---------------------------------\n"
            f"TARGET TRAJECTORY:\n"
            f"  Mode   : {self.traj_mode}\n"
            f"  Alt Z  : {self.base_alt:4.1f} m\n"
            f"  Scale  : {self.path_scale:4.1f} m\n"
            f"---------------------------------\n"
            f"DRONE POSITION [m]:\n"
            f"  X : {pos[0]:+6.2f}   Y : {pos[1]:+6.2f}\n"
            f"  Z : {pos[2]:+6.2f}\n"
            f"---------------------------------\n"
            f"UNIT QUATERNION q:\n"
            f"  qw: {quat[0]:+7.4f}\n"
            f"  qx: {quat[1]:+7.4f}\n"
            f"  qy: {quat[2]:+7.4f}\n"
            f"  qz: {quat[3]:+7.4f}\n"
            f"---------------------------------\n"
            f"EULER ANGLES [deg]:\n"
            f"  Roll  φ : {euler[0]:+6.2f}°\n"
            f"  Pitch θ : {euler[1]:+6.2f}°\n"
            f"  Yaw   ψ : {euler[2]:+6.2f}°\n"
            f"---------------------------------\n"
            f"TRACKING ACCURACY:\n"
            f"  Pos Error : {pos_err_norm:6.3f} m\n"
            f"  Gimbal Lock: ZERO (Sp(1))\n"
        )
        self.lbl_telem.config(text=text)


if __name__ == "__main__":
    root = tk.Tk()
    app = DroneFlightApp(root)
    root.mainloop()
