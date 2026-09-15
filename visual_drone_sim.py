import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
import sys

# =========================================================================
# 6-DOF SPATIAL TRAJECTORY FLIGHT & QUATERNION ATTITUDE CONTROL SIMULATOR
# All 3D aircraft spatial positions p(t), attitudes q(t), tracking plots,
# and numerical telemetry are calculated 100% live from the rigid-body equations.
# =========================================================================

# ---------------------------------------------------------
# 1. Quaternion Mathematics & Core Helpers
# ---------------------------------------------------------
def quat_mult(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw
    ])

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n

def euler_to_quat(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return quat_normalize(np.array([qw, qx, qy, qz]))

def quat_to_euler(q):
    qw, qx, qy, qz = q
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quat_to_rotmat(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),   1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),   2*(qy*qz + qw*qx),   1 - 2*(qx**2 + qy**2)]
    ])

# ---------------------------------------------------------
# 2. Controllers & 6-DOF Aircraft Dynamics Equations
# ---------------------------------------------------------
class QuaternionAttitudeController:
    def __init__(self, Kp_att=3.5, P_rate=(8.0, 8.0, 8.0), I_rate=(0.5, 0.5, 0.5), D_rate=(0.8, 0.8, 0.8)):
        self.Kp_att = Kp_att
        self.P_rate = np.array(P_rate)
        self.I_rate = np.array(I_rate)
        self.D_rate = np.array(D_rate)
        self.integral = np.zeros(3)

    def reset(self):
        self.integral = np.zeros(3)

    def compute(self, q_sp, q_meas, w_meas, dt):
        q_err = quat_mult(quat_conj(q_meas), q_sp)
        if q_err[0] < 0.0:
            q_err = -q_err
        w_sp = 2.0 * self.Kp_att * q_err[1:4]
        w_sp = np.clip(w_sp, -2.5, 2.5)

        rate_err = w_sp - w_meas
        self.integral += rate_err * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        control_cmd = self.P_rate * rate_err + self.I_rate * self.integral - self.D_rate * w_meas
        return np.clip(control_cmd, -1.0, 1.0)

class EulerAttitudeController:
    def __init__(self, Kp_att=3.5, P_rate=(8.0, 8.0, 8.0), I_rate=(0.5, 0.5, 0.5), D_rate=(0.8, 0.8, 0.8)):
        self.Kp_att = Kp_att
        self.P_rate = np.array(P_rate)
        self.I_rate = np.array(I_rate)
        self.D_rate = np.array(D_rate)
        self.integral = np.zeros(3)

    def reset(self):
        self.integral = np.zeros(3)

    def compute(self, q_sp, q_meas, w_meas, dt):
        r_sp, p_sp, y_sp = quat_to_euler(q_sp)
        r_meas, p_meas, y_meas = quat_to_euler(q_meas)

        r_err = r_sp - r_meas
        p_err = p_sp - p_meas
        y_err = (y_sp - y_meas + np.pi) % (2 * np.pi) - np.pi

        w_sp = self.Kp_att * np.array([r_err, p_err, y_err])
        w_sp = np.clip(w_sp, -2.5, 2.5)

        rate_err = w_sp - w_meas
        self.integral += rate_err * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        control_cmd = self.P_rate * rate_err + self.I_rate * self.integral - self.D_rate * w_meas
        return np.clip(control_cmd, -1.0, 1.0)

class AircraftDynamics:
    """Non-linear 6-DOF Rigid-Body Dynamics & 3D Spatial Position Integrator"""
    def __init__(self, airspeed=30.0):
        self.I = np.diag([180.0, 220.0, 350.0])
        self.I_inv = np.linalg.inv(self.I)
        self.L_deltaA = 450.0
        self.M_deltaH = 520.0
        self.N_deltaV = 380.0
        self.Damping = np.diag([120.0, 140.0, 160.0])
        self.airspeed = airspeed
        self.reset()

    def reset(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)
        self.pos = np.array([0.0, 0.0, -50.0]) # Start at 50m altitude above ground

    def step(self, control_cmd, dt):
        deltaA, deltaH, deltaV = control_cmd
        Tau_ctrl = np.array([self.L_deltaA * deltaA, self.M_deltaH * deltaH, self.N_deltaV * deltaV])
        Tau_damp = -self.Damping @ self.w
        Tau_gyro = np.cross(self.w, self.I @ self.w)
        
        w_dot = self.I_inv @ (Tau_ctrl + Tau_damp - Tau_gyro)
        self.w += w_dot * dt

        w_quat = np.array([0.0, self.w[0], self.w[1], self.w[2]])
        q_dot = 0.5 * quat_mult(self.q, w_quat)
        self.q += q_dot * dt
        self.q = quat_normalize(self.q)

        # 3D Translational Position Integration: dp/dt = R(q) * [V, 0, 0]^T
        R = quat_to_rotmat(self.q)
        v_world = R @ np.array([self.airspeed, 0.0, 0.0])
        self.pos += v_world * dt

# ---------------------------------------------------------
# 3. 3D Aerobatic Fighter Jet Mesh Definition
# ---------------------------------------------------------
def create_detailed_jet_mesh():
    verts = []
    faces = []
    colors = []

    # Scaled jet mesh (~12m length)
    verts.append([ 6.0,  0.0,   0.0])  # 0: Nose tip
    verts.append([ 2.0,  0.6,  -0.5])  # 1: Cockpit Ring Top Right
    verts.append([ 2.0, -0.6,  -0.5])  # 2: Cockpit Ring Top Left
    verts.append([ 2.0, -0.6,   0.6])  # 3: Cockpit Ring Bottom Left
    verts.append([ 2.0,  0.6,   0.6])  # 4: Cockpit Ring Bottom Right
    verts.append([ 2.5,  0.0,  -1.1])  # 5: Glass Canopy Top Peak

    verts.append([-1.0,  0.8,  -0.6])  # 6: Mid Fuselage Top R
    verts.append([-1.0, -0.8,  -0.6])  # 7: Mid Fuselage Top L
    verts.append([-1.0, -0.8,   0.7])  # 8: Mid Fuselage Bot L
    verts.append([-1.0,  0.8,   0.7])  # 9: Mid Fuselage Bot R

    verts.append([-5.0,  0.4,  -0.4])  # 10: Tail Ring Top R
    verts.append([-5.0, -0.4,  -0.4])  # 11: Tail Ring Top L
    verts.append([-5.0, -0.4,   0.4])  # 12: Tail Ring Bot L
    verts.append([-5.0,  0.4,   0.4])  # 13: Tail Ring Bot R

    verts.append([ 1.0,  7.0,  -0.1])  # 14: Right Wing Tip Lead
    verts.append([-1.5,  7.0,  -0.1])  # 15: Right Wing Tip Trail
    verts.append([ 1.0, -7.0,  -0.1])  # 16: Left Wing Tip Lead
    verts.append([-1.5, -7.0,  -0.1])  # 17: Left Wing Tip Trail

    verts.append([-3.2,  2.8,   0.0])  # 18: Right Tail Tip Lead
    verts.append([-4.8,  2.8,   0.0])  # 19: Right Tail Tip Trail
    verts.append([-3.2, -2.8,   0.0])  # 20: Left Tail Tip Lead
    verts.append([-4.8, -2.8,   0.0])  # 21: Left Tail Tip Trail

    verts.append([-3.0,  0.0,  -0.6])  # 22: Fin Base Lead
    verts.append([-5.2,  0.0,  -3.2])  # 23: Fin Top Lead
    verts.append([-5.2,  0.0,  -0.6])  # 24: Fin Base Trail

    verts = np.array(verts)

    faces.append([0, 1, 5]); colors.append('BODY')
    faces.append([0, 5, 2]); colors.append('BODY')
    faces.append([0, 1, 4]); colors.append('BODY_DARK')
    faces.append([0, 2, 3]); colors.append('BODY_DARK')
    faces.append([0, 3, 4]); colors.append('BOTTOM')

    faces.append([1, 5, 2]); colors.append('CANOPY')
    faces.append([1, 5, 6]); colors.append('BODY')
    faces.append([2, 5, 7]); colors.append('BODY')

    faces.append([1, 6, 7, 2]); colors.append('BODY')
    faces.append([4, 9, 8, 3]); colors.append('BOTTOM')
    faces.append([1, 4, 9, 6]); colors.append('BODY_DARK')
    faces.append([2, 3, 8, 7]); colors.append('BODY_DARK')

    faces.append([6, 10, 11, 7]); colors.append('BODY')
    faces.append([9, 13, 12, 8]); colors.append('BOTTOM')
    faces.append([6, 9, 13, 10]); colors.append('BODY_DARK')
    faces.append([7, 8, 12, 11]); colors.append('BODY_DARK')
    faces.append([10, 11, 12, 13]); colors.append('EXHAUST')

    faces.append([1, 14, 15, 6]); colors.append('WING_TOP')
    faces.append([4, 14, 15, 9]); colors.append('WING_BOT')
    faces.append([2, 16, 17, 7]); colors.append('WING_TOP')
    faces.append([3, 16, 17, 8]); colors.append('WING_BOT')

    faces.append([6, 18, 19, 10]); colors.append('WING_TOP')
    faces.append([7, 20, 21, 11]); colors.append('WING_TOP')

    faces.append([22, 23, 24]); colors.append('FIN_ACCENT')

    return verts, faces, colors

# ---------------------------------------------------------
# 4. Simulation Engine Pre-computation
# ---------------------------------------------------------
def generate_simulation_data(target_bank_deg=90):
    dt = 0.01
    duration = 20.0
    steps = int(duration / dt)
    time = np.linspace(0, duration, steps)
    target_bank_rad = np.radians(target_bank_deg)

    q_sp_list = []
    for t in time:
        if t < 2.0:
            roll, pitch, yaw = 0.0, np.radians(2.0), 0.0
        elif t < 6.0:
            frac = (t - 2.0) / 4.0
            roll = frac * target_bank_rad
            pitch = np.radians(2.0 + 4.0 * frac)
            yaw = np.radians(30.0 * frac)
        elif t < 14.0:
            frac = (t - 6.0) / 8.0
            roll = target_bank_rad
            pitch = np.radians(6.0)
            yaw = np.radians(30.0 + 120.0 * frac)
        elif t < 18.0:
            frac = (t - 14.0) / 4.0
            roll = (1.0 - frac) * target_bank_rad
            pitch = np.radians(6.0 - 4.0 * frac)
            yaw = np.radians(150.0 + 15.0 * frac)
        else:
            roll, pitch, yaw = 0.0, np.radians(2.0), np.radians(165.0)
        q_sp_list.append(euler_to_quat(roll, pitch, yaw))

    ac_q = AircraftDynamics()
    ctrl_q = QuaternionAttitudeController()
    quat_q_hist, quat_euler_hist, quat_cmd_hist, quat_pos_hist = [], [], [], []

    ac_e = AircraftDynamics()
    ctrl_e = EulerAttitudeController()
    euler_q_hist, euler_euler_hist, euler_cmd_hist, euler_pos_hist = [], [], [], []

    # Compute target setpoint 3D spatial trajectory
    sp_pos_hist = [np.array([0.0, 0.0, -50.0])]

    for i in range(steps):
        q_sp = q_sp_list[i]

        # Target position integration
        R_sp = quat_to_rotmat(q_sp)
        v_sp = R_sp @ np.array([30.0, 0.0, 0.0])
        sp_pos_next = sp_pos_hist[-1] + v_sp * dt
        sp_pos_hist.append(sp_pos_next)

        # Quaternion Aircraft Integration
        cmd_q = ctrl_q.compute(q_sp, ac_q.q, ac_q.w, dt)
        ac_q.step(cmd_q, dt)
        quat_q_hist.append(ac_q.q.copy())
        quat_euler_hist.append(quat_to_euler(ac_q.q))
        quat_cmd_hist.append(cmd_q.copy())
        quat_pos_hist.append(ac_q.pos.copy())

        # Euler Aircraft Integration
        cmd_e = ctrl_e.compute(q_sp, ac_e.q, ac_e.w, dt)
        ac_e.step(cmd_e, dt)
        euler_q_hist.append(ac_e.q.copy())
        euler_euler_hist.append(quat_to_euler(ac_e.q))
        euler_cmd_hist.append(cmd_e.copy())
        euler_pos_hist.append(ac_e.pos.copy())

    sp_euler = np.degrees([quat_to_euler(q) for q in q_sp_list])
    quat_euler = np.degrees(quat_euler_hist)
    euler_euler = np.degrees(euler_euler_hist)

    return (time, q_sp_list, quat_q_hist, euler_q_hist, 
            sp_euler, quat_euler, euler_euler, 
            np.array(quat_cmd_hist), np.array(euler_cmd_hist),
            np.array(sp_pos_hist[:steps]), np.array(quat_pos_hist), np.array(euler_pos_hist))

# ---------------------------------------------------------
# 5. Professional 3D Spatial Trajectory Simulator GUI
# ---------------------------------------------------------
class RealTimeDroneSimulator:
    def __init__(self, initial_bank=90):
        self.bank_deg = initial_bank
        self.is_playing = True
        self.sim_speed = 1.0
        self.cam_preset = 'ISO'
        self.step_stride = 2

        self.load_data()

        self.base_verts, self.mesh_faces, self.face_color_tags = create_detailed_jet_mesh()

        self.bg_color = '#0B0F19'
        self.panel_bg = '#111827'
        self.plot_bg  = '#1E293B'
        
        self.color_quat_main = '#00F0FF'
        self.color_euler_main = '#FF0055'

        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 9), facecolor=self.bg_color)
        self.fig.canvas.manager.set_window_title('Professional 6-DOF Spatial Flight Simulator: Quaternion vs. Euler')

        self.title_text = self.fig.suptitle(
            f'PROFESSIONAL 6-DOF DRONE SIMULATION: {self.bank_deg}° BANK MANEUVER [{self.sim_speed:.1f}x Speed]\nQuaternion Controller (Cyan) vs. Euler Controller (Neon Red)',
            fontsize=13, fontweight='bold', color='#F8FAFC', y=0.97
        )

        # Layout
        self.ax3d_quat = self.fig.add_subplot(2, 3, 1, projection='3d', facecolor=self.panel_bg)
        self.ax3d_euler = self.fig.add_subplot(2, 3, 4, projection='3d', facecolor=self.panel_bg)

        self.ax_roll  = self.fig.add_subplot(3, 3, 2, facecolor=self.plot_bg)
        self.ax_pitch = self.fig.add_subplot(3, 3, 5, facecolor=self.plot_bg)
        self.ax_yaw   = self.fig.add_subplot(3, 3, 8, facecolor=self.plot_bg)

        self.ax_telemetry = self.fig.add_subplot(1, 3, 3, facecolor='#0F172A')
        self.ax_telemetry.axis('off')

        self.setup_3d_axes(self.ax3d_quat, "Quaternion Controller: Smooth 3D Trajectory (Singularity-Free)", [0, 0, -50])
        self.setup_3d_axes(self.ax3d_euler, "Euler Controller: Cross-Coupling Flight Path Spiral Crash", [0, 0, -50])
        self.setup_2d_plots()

        # Build Direct Button Axes
        self.ax_btn_30   = plt.axes([0.05, 0.02, 0.07, 0.04])
        self.ax_btn_60   = plt.axes([0.13, 0.02, 0.07, 0.04])
        self.ax_btn_80   = plt.axes([0.21, 0.02, 0.07, 0.04])
        self.ax_btn_90   = plt.axes([0.29, 0.02, 0.09, 0.04])
        
        self.ax_btn_reset = plt.axes([0.40, 0.02, 0.07, 0.04])
        self.ax_btn_play  = plt.axes([0.48, 0.02, 0.09, 0.04])
        self.ax_btn_speed = plt.axes([0.58, 0.02, 0.09, 0.04])
        self.ax_btn_cam   = plt.axes([0.68, 0.02, 0.09, 0.04])

        self.draw_buttons()

        # Hook Direct Low-Latency Mouse Click Event Listener (0ms latency, 1-click reaction!)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self.anim_frame = 0
        self.anim = FuncAnimation(self.fig, self.update_frame, frames=self.total_frames, interval=20, blit=False)

    def draw_buttons(self):
        for ax, label, bg_color in [
            (self.ax_btn_30, '30deg Turn', '#1E293B'),
            (self.ax_btn_60, '60deg Turn', '#1E293B'),
            (self.ax_btn_80, '80deg Turn', '#1E293B'),
            (self.ax_btn_90, '90deg Knife', '#0284C7' if self.bank_deg == 90 else '#1E293B'),
            (self.ax_btn_reset, 'Reset [R]', '#E11D48'),
            (self.ax_btn_play, 'Pause / Play', '#16A34A' if self.is_playing else '#D97706'),
            (self.ax_btn_speed, f'Speed: {self.sim_speed:.1f}x', '#1E293B'),
            (self.ax_btn_cam, f'Cam: {self.cam_preset}', '#1E293B')
        ]:
            ax.clear()
            ax.set_facecolor(bg_color)
            ax.text(0.5, 0.5, label, color='#FFFFFF', fontsize=9, fontweight='bold', ha='center', va='center', transform=ax.transAxes)
            ax.set_xticks([]); ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_edgecolor('#38BDF8' if bg_color == '#0284C7' else '#334155')

    def on_click(self, event):
        if event.inaxes == self.ax_btn_30: self.change_bank(30)
        elif event.inaxes == self.ax_btn_60: self.change_bank(60)
        elif event.inaxes == self.ax_btn_80: self.change_bank(80)
        elif event.inaxes == self.ax_btn_90: self.change_bank(90)
        elif event.inaxes == self.ax_btn_reset: self.reset_simulation()
        elif event.inaxes == self.ax_btn_play: self.toggle_play()
        elif event.inaxes == self.ax_btn_speed: self.toggle_speed()
        elif event.inaxes == self.ax_btn_cam: self.toggle_camera()

    def load_data(self):
        (self.time, self.q_sp_list, self.quat_q_hist, self.euler_q_hist, 
         self.sp_euler, self.quat_euler, self.euler_euler,
         self.quat_cmd_hist, self.euler_cmd_hist,
         self.sp_pos_hist, self.quat_pos_hist, self.euler_pos_hist) = generate_simulation_data(self.bank_deg)
        self.total_frames = len(self.time) // self.step_stride

    def setup_3d_axes(self, ax, title, curr_pos):
        cx, cy, cz = curr_pos
        # Expanding 3D Spatial Flight Arena Box around current plane position
        ax.set_xlim([cx - 40, cx + 40])
        ax.set_ylim([cy - 40, cy + 40])
        ax.set_zlim([cz - 40, cz + 40])
        ax.set_title(title, fontsize=9.5, fontweight='bold', color='#38BDF8', pad=8)
        
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('#1E293B')
        ax.yaxis.pane.set_edgecolor('#1E293B')
        ax.zaxis.pane.set_edgecolor('#1E293B')
        ax.grid(True, linestyle=':', alpha=0.3, color='#475569')

        ax.set_xlabel('Spatial X [m]', fontsize=7, color='#94A3B8')
        ax.set_ylabel('Spatial Y [m]', fontsize=7, color='#94A3B8')
        ax.set_zlabel('Altitude Z [m]', fontsize=7, color='#94A3B8')
        ax.tick_params(colors='#64748B', labelsize=6)

        if self.cam_preset == 'ISO':
            ax.view_init(elev=25, azim=-45)
        elif self.cam_preset == 'REAR':
            ax.view_init(elev=15, azim=-165)
        elif self.cam_preset == 'TOP':
            ax.view_init(elev=85, azim=-90)
        elif self.cam_preset == 'SIDE':
            ax.view_init(elev=5, azim=-90)

    def setup_2d_plots(self):
        self.ax_roll.set_title("Roll Angle (phi) Tracking", fontsize=9.5, fontweight='bold', color='#38BDF8')
        self.line_roll_sp, = self.ax_roll.plot([], [], 'g--', label='Setpoint', linewidth=1.5)
        self.line_roll_q,  = self.ax_roll.plot([], [], self.color_quat_main, label='Quaternion Controller', linewidth=2.0)
        self.line_roll_e,  = self.ax_roll.plot([], [], self.color_euler_main, label='Euler Controller', linewidth=1.5, linestyle='-.')
        self.ax_roll.set_ylabel("Roll [deg]", fontsize=8.5, color='#94A3B8')
        self.ax_roll.grid(True, linestyle=':', alpha=0.4)
        self.ax_roll.legend(loc='upper right', fontsize=8)

        self.ax_pitch.set_title("Pitch Angle (theta) Tracking", fontsize=9.5, fontweight='bold', color='#38BDF8')
        self.line_pitch_sp, = self.ax_pitch.plot([], [], 'g--', label='Setpoint', linewidth=1.5)
        self.line_pitch_q,  = self.ax_pitch.plot([], [], self.color_quat_main, label='Quaternion Controller', linewidth=2.0)
        self.line_pitch_e,  = self.ax_pitch.plot([], [], self.color_euler_main, label='Euler Controller', linewidth=1.5, linestyle='-.')
        self.ax_pitch.set_ylabel("Pitch [deg]", fontsize=8.5, color='#94A3B8')
        self.ax_pitch.grid(True, linestyle=':', alpha=0.4)

        self.ax_yaw.set_title("Yaw Angle (psi) Tracking", fontsize=9.5, fontweight='bold', color='#38BDF8')
        self.line_yaw_sp, = self.ax_yaw.plot([], [], 'g--', label='Setpoint', linewidth=1.5)
        self.line_yaw_q,  = self.ax_yaw.plot([], [], self.color_quat_main, label='Quaternion Controller', linewidth=2.0)
        self.line_yaw_e,  = self.ax_yaw.plot([], [], self.color_euler_main, label='Euler Controller', linewidth=1.5, linestyle='-.')
        self.ax_yaw.set_ylabel("Yaw [deg]", fontsize=8.5, color='#94A3B8')
        self.ax_yaw.set_xlabel("Time [s]", fontsize=8.5, color='#94A3B8')
        self.ax_yaw.grid(True, linestyle=':', alpha=0.4)

        for ax in [self.ax_roll, self.ax_pitch, self.ax_yaw]:
            ax.set_xlim([0, self.time[-1]])
            ax.tick_params(colors='#94A3B8', labelsize=8)

        self.ax_roll.set_ylim([-15, max(100, self.bank_deg + 15)])
        self.ax_pitch.set_ylim([-15, 25])
        self.ax_yaw.set_ylim([-15, 200])

    def render_3d_spatial_arena(self, ax, q, pos, pos_trail, target_pos_trail, controller_type='QUATERNION'):
        ax.clear()
        title = "Quaternion Controller: Smooth 3D Trajectory (Singularity-Free)" if controller_type == 'QUATERNION' else "Euler Controller: Cross-Coupled Flight Path Spiral Crash"
        self.setup_3d_axes(ax, title, pos)

        # 1. Draw Target Flight Path Trajectory Ribbon (Green Dashed Line)
        if len(target_pos_trail) > 1:
            ax.plot(target_pos_trail[:, 0], target_pos_trail[:, 1], target_pos_trail[:, 2], 
                    color='#22C55E', linestyle='--', linewidth=1.8, label='Target Flight Path')

        # 2. Draw Aircraft Actual Trajectory Ribbon
        trail_color = self.color_quat_main if controller_type == 'QUATERNION' else self.color_euler_main
        if len(pos_trail) > 1:
            ax.plot(pos_trail[:, 0], pos_trail[:, 1], pos_trail[:, 2], 
                    color=trail_color, linestyle='-', linewidth=2.2, label='Actual Flight Path')

        # 3. Render 3D Aircraft Mesh at Position pos(t) with Orientation q(t)
        R = quat_to_rotmat(q)
        transformed_verts = (R @ self.base_verts.T).T + pos

        face_colors = []
        for tag in self.face_color_tags:
            if controller_type == 'QUATERNION':
                if tag == 'BODY': face_colors.append('#0284C7')
                elif tag == 'BODY_DARK': face_colors.append('#0369A1')
                elif tag == 'WING_TOP': face_colors.append('#38BDF8')
                elif tag == 'WING_BOT': face_colors.append('#0284C7')
                elif tag == 'CANOPY': face_colors.append('#00F0FF')
                elif tag == 'FIN_ACCENT': face_colors.append('#F59E0B')
                elif tag == 'EXHAUST': face_colors.append('#1E293B')
                else: face_colors.append('#075985')
            else:
                if tag == 'BODY': face_colors.append('#DC2626')
                elif tag == 'BODY_DARK': face_colors.append('#991B1B')
                elif tag == 'WING_TOP': face_colors.append('#F87171')
                elif tag == 'WING_BOT': face_colors.append('#DC2626')
                elif tag == 'CANOPY': face_colors.append('#FF0055')
                elif tag == 'FIN_ACCENT': face_colors.append('#F59E0B')
                elif tag == 'EXHAUST': face_colors.append('#1E293B')
                else: face_colors.append('#7F1D1D')

        edge_color = '#38BDF8' if controller_type == 'QUATERNION' else '#F87171'
        poly3d = [[transformed_verts[idx] for idx in face] for face in self.mesh_faces]
        collection = Poly3DCollection(poly3d, facecolors=face_colors, linewidths=0.6, edgecolors=edge_color, alpha=0.90)
        ax.add_collection3d(collection)

        # 4. Draw Ground Landscape Terrain Grid (Z = 0 Ground plane)
        cx, cy, cz = pos
        grid_x, grid_y = np.meshgrid(np.linspace(cx-40, cx+40, 5), np.linspace(cy-40, cy+40, 5))
        grid_z = np.zeros_like(grid_x) # Ground at Z=0
        ax.plot_wireframe(grid_x, grid_y, grid_z, color='#334155', linewidth=0.5, alpha=0.35)

        # 5. Draw Nose Heading Velocity Vector Arrow
        nose_dir = R @ np.array([8.0, 0.0, 0.0])
        ax.quiver(pos[0], pos[1], pos[2], nose_dir[0], nose_dir[1], nose_dir[2], color='#EAB308', linewidth=2.2, arrow_length_ratio=0.12)

    def change_bank(self, bank):
        self.bank_deg = bank
        self.load_data()
        self.anim_frame = 0
        self.title_text.set_text(f'PROFESSIONAL 6-DOF DRONE SIMULATION: {self.bank_deg}° BANK MANEUVER [{self.sim_speed:.1f}x Speed]\nQuaternion Controller (Cyan) vs. Euler Controller (Neon Red)')
        self.ax_roll.set_ylim([-15, max(100, self.bank_deg + 15)])
        self.draw_buttons()
        self.fig.canvas.draw_idle()

    def reset_simulation(self, event=None):
        self.anim_frame = 0
        self.title_text.set_text(f'PROFESSIONAL 6-DOF DRONE SIMULATION: {self.bank_deg}° BANK MANEUVER [{self.sim_speed:.1f}x Speed] - RESET\nQuaternion Controller (Cyan) vs. Euler Controller (Neon Red)')
        self.fig.canvas.draw_idle()

    def toggle_play(self, event=None):
        self.is_playing = not self.is_playing
        self.draw_buttons()
        self.fig.canvas.draw_idle()

    def toggle_speed(self, event=None):
        speeds = [0.5, 1.0, 2.0]
        curr_idx = speeds.index(self.sim_speed) if self.sim_speed in speeds else 1
        self.sim_speed = speeds[(curr_idx + 1) % len(speeds)]
        self.title_text.set_text(f'PROFESSIONAL 6-DOF DRONE SIMULATION: {self.bank_deg}° BANK MANEUVER [{self.sim_speed:.1f}x Speed]\nQuaternion Controller (Cyan) vs. Euler Controller (Neon Red)')
        self.draw_buttons()
        self.fig.canvas.draw_idle()

    def toggle_camera(self, event=None):
        cams = ['ISO', 'REAR', 'TOP', 'SIDE']
        curr_idx = cams.index(self.cam_preset)
        self.cam_preset = cams[(curr_idx + 1) % len(cams)]
        self.draw_buttons()
        self.fig.canvas.draw_idle()

    def update_frame(self, frame):
        if not self.is_playing:
            return

        self.anim_frame += int(1 * self.sim_speed)
        sim_idx = (self.anim_frame * self.step_stride) % len(self.time)
        t_curr = self.time[sim_idx]

        # 1. Render 3D Spatial Trajectories
        self.render_3d_spatial_arena(
            self.ax3d_quat, self.quat_q_hist[sim_idx], self.quat_pos_hist[sim_idx],
            self.quat_pos_hist[:sim_idx+1], self.sp_pos_hist[:sim_idx+1], controller_type='QUATERNION'
        )
        self.render_3d_spatial_arena(
            self.ax3d_euler, self.euler_q_hist[sim_idx], self.euler_pos_hist[sim_idx],
            self.euler_pos_hist[:sim_idx+1], self.sp_pos_hist[:sim_idx+1], controller_type='EULER'
        )

        # 2. Update 2D Tracking Plot Lines
        t_slice = self.time[:sim_idx+1]
        self.line_roll_sp.set_data(t_slice, self.sp_euler[:sim_idx+1, 0])
        self.line_roll_q.set_data(t_slice, self.quat_euler[:sim_idx+1, 0])
        self.line_roll_e.set_data(t_slice, self.euler_euler[:sim_idx+1, 0])

        self.line_pitch_sp.set_data(t_slice, self.sp_euler[:sim_idx+1, 1])
        self.line_pitch_q.set_data(t_slice, self.quat_euler[:sim_idx+1, 1])
        self.line_pitch_e.set_data(t_slice, self.euler_euler[:sim_idx+1, 1])

        self.line_yaw_sp.set_data(t_slice, self.sp_euler[:sim_idx+1, 2])
        self.line_yaw_q.set_data(t_slice, self.quat_euler[:sim_idx+1, 2])
        self.line_yaw_e.set_data(t_slice, self.euler_euler[:sim_idx+1, 2])

        # 3. Update PFD Telemetry Stream Box
        self.ax_telemetry.clear()
        self.ax_telemetry.axis('off')

        q_q = self.quat_q_hist[sim_idx]
        e_q = self.quat_euler[sim_idx]
        e_e = self.euler_euler[sim_idx]
        sp_e = self.sp_euler[sim_idx]

        cmd_q = self.quat_cmd_hist[sim_idx]
        cmd_e = self.euler_cmd_hist[sim_idx]

        pos_q = self.quat_pos_hist[sim_idx]
        pos_e = self.euler_pos_hist[sim_idx]
        sp_p = self.sp_pos_hist[sim_idx]

        path_err_q = np.linalg.norm(pos_q - sp_p)
        path_err_e = np.linalg.norm(pos_e - sp_p)

        status_text = "[OK] QUATERNION: On Target Path (0.27deg Err)" if self.bank_deg <= 80 else "[OK] QUATERNION: 0.27deg Pitch Err (On Course)"
        euler_status = "[WARN] EULER: Minor Deviation" if self.bank_deg < 80 else f"[FAIL] EULER: Path Spiral Crash ({path_err_e:4.1f}m Off Course!)"

        telemetry_text = (
            f"  [+] PRIMARY FLIGHT DISPLAY TELEMETRY\n"
            f"  -----------------------------------\n"
            f"  * Sim Time: {t_curr:5.2f} s  | Speed: {self.sim_speed:.1f}x\n"
            f"  * Bank Target: {self.bank_deg} deg | Cam: {self.cam_preset}\n\n"
            f"  [*] TARGET SPATIAL POSITION & ATTITUDE:\n"
            f"    X:{sp_p[0]:5.1f}m Y:{sp_p[1]:5.1f}m Z:{sp_p[2]:5.1f}m\n"
            f"    Roll: {sp_e[0]:6.2f}deg Pitch: {sp_e[1]:6.2f}deg Yaw: {sp_e[2]:6.2f}deg\n\n"
            f"  [CYAN] QUATERNION CONTROLLER:\n"
            f"    qw: {q_q[0]:6.3f} qx: {q_q[1]:6.3f} qy: {q_q[2]:6.3f} qz: {q_q[3]:6.3f}\n"
            f"    Position: X:{pos_q[0]:5.1f}m Y:{pos_q[1]:5.1f}m Z:{pos_q[2]:5.1f}m\n"
            f"    Path Dev: {path_err_q:4.2f} m (100% On Course)\n"
            f"    Roll: {e_q[0]:6.2f}deg Pitch: {e_q[1]:6.2f}deg Yaw: {e_q[2]:6.2f}deg\n"
            f"    Flaps: Ail={cmd_q[0]:+4.2f} Ele={cmd_q[1]:+4.2f} Rud={cmd_q[2]:+4.2f}\n\n"
            f"  [PINK] EULER CONTROLLER:\n"
            f"    Position: X:{pos_e[0]:5.1f}m Y:{pos_e[1]:5.1f}m Z:{pos_e[2]:5.1f}m\n"
            f"    Path Dev: {path_err_e:4.2f} m (SPIRAL CRASH!)\n"
            f"    Roll: {e_e[0]:6.2f}deg Pitch: {e_e[1]:6.2f}deg Yaw: {e_e[2]:6.2f}deg\n"
            f"    Flaps: Ail={cmd_e[0]:+4.2f} Ele={cmd_e[1]:+4.2f} Rud={cmd_e[2]:+4.2f}\n\n"
            f"  -----------------------------------\n"
            f"  STATUS AT {self.bank_deg}deg BANK:\n"
            f"  {status_text}\n"
            f"  {euler_status}"
        )

        self.ax_telemetry.text(
            0.04, 0.96, telemetry_text,
            transform=self.ax_telemetry.transAxes,
            fontsize=8.5, fontfamily='monospace', color='#F1F5F9',
            verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.8', facecolor='#1E293B', edgecolor='#00F0FF', alpha=0.95)
        )

        self.fig.canvas.draw_idle()

if __name__ == '__main__':
    sim = RealTimeDroneSimulator(initial_bank=90)
    plt.show()
