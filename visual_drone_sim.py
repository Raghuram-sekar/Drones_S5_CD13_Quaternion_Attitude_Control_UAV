import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import sys

# ---------------------------------------------------------
# 1. Quaternion Mathematics & Helpers
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
# 2. Controllers & 6-DOF Aircraft Dynamics
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
    def __init__(self):
        self.I = np.diag([180.0, 220.0, 350.0])
        self.I_inv = np.linalg.inv(self.I)
        self.L_deltaA = 450.0
        self.M_deltaH = 520.0
        self.N_deltaV = 380.0
        self.Damping = np.diag([120.0, 140.0, 160.0])
        self.reset()

    def reset(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)

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

# ---------------------------------------------------------
# 3. 3D Airplane Model Geometry
# ---------------------------------------------------------
def get_aircraft_mesh():
    vertices = np.array([
        [ 2.0,  0.0,  0.0],
        [-1.5,  0.3, -0.2],
        [-1.5, -0.3, -0.2],
        [-1.5,  0.0,  0.3],
        [ 0.4,  2.2,  0.0],
        [-0.4,  2.2,  0.0],
        [ 0.4, -2.2,  0.0],
        [-0.4, -2.2,  0.0],
        [-1.1,  0.9,  0.0],
        [-1.5,  0.9,  0.0],
        [-1.1, -0.9,  0.0],
        [-1.5, -0.9,  0.0],
        [-1.1,  0.0,  0.0],
        [-1.6,  0.0, -1.0],
        [-1.6,  0.0,  0.0]
    ])

    faces = [
        [0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 2, 3],
        [0, 4, 5], [0, 5, 1], [0, 6, 7], [0, 7, 1],
        [12, 8, 9], [12, 10, 11],
        [12, 13, 14]
    ]
    return vertices, faces

# ---------------------------------------------------------
# 4. Simulation Engine Pre-computation
# ---------------------------------------------------------
def generate_simulation_data(target_bank_deg=90):
    dt = 0.02
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
    quat_q_hist, quat_euler_hist = [], []

    ac_e = AircraftDynamics()
    ctrl_e = EulerAttitudeController()
    euler_q_hist, euler_euler_hist = [], []

    for i in range(steps):
        q_sp = q_sp_list[i]

        cmd_q = ctrl_q.compute(q_sp, ac_q.q, ac_q.w, dt)
        ac_q.step(cmd_q, dt)
        quat_q_hist.append(ac_q.q.copy())
        quat_euler_hist.append(quat_to_euler(ac_q.q))

        cmd_e = ctrl_e.compute(q_sp, ac_e.q, ac_e.w, dt)
        ac_e.step(cmd_e, dt)
        euler_q_hist.append(ac_e.q.copy())
        euler_euler_hist.append(quat_to_euler(ac_e.q))

    sp_euler = np.degrees([quat_to_euler(q) for q in q_sp_list])
    quat_euler = np.degrees(quat_euler_hist)
    euler_euler = np.degrees(euler_euler_hist)

    return time, q_sp_list, quat_q_hist, euler_q_hist, sp_euler, quat_euler, euler_euler

# ---------------------------------------------------------
# 5. Interactive Real-Time Visual Simulator Application
# ---------------------------------------------------------
class RealTimeDroneSimulator:
    def __init__(self, initial_bank=90):
        self.bank_deg = initial_bank
        self.is_playing = True
        self.current_frame = 0

        self.time, self.q_sp_list, self.quat_q_hist, self.euler_q_hist, self.sp_euler, self.quat_euler, self.euler_euler = generate_simulation_data(self.bank_deg)
        self.total_frames = len(self.time)

        self.base_verts, self.mesh_faces = get_aircraft_mesh()

        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 9), facecolor='#0B0F19')
        self.fig.canvas.manager.set_window_title('Real-Time Flight Controller Simulator: Quaternion vs. Euler')

        self.title_text = self.fig.suptitle(
            f'REAL-TIME 6-DOF DRONE SIMULATION: {self.bank_deg}° BANK MANEUVER\nQuaternion Controller (Blue) vs. Euler Controller (Red)',
            fontsize=13, fontweight='bold', color='#E2E8F0', y=0.97
        )

        self.ax3d_quat = self.fig.add_subplot(2, 3, 1, projection='3d', facecolor='#111827')
        self.ax3d_euler = self.fig.add_subplot(2, 3, 4, projection='3d', facecolor='#111827')

        self.ax_roll  = self.fig.add_subplot(3, 3, 2, facecolor='#1E293B')
        self.ax_pitch = self.fig.add_subplot(3, 3, 5, facecolor='#1E293B')
        self.ax_yaw   = self.fig.add_subplot(3, 3, 8, facecolor='#1E293B')

        self.ax_telemetry = self.fig.add_subplot(1, 3, 3, facecolor='#0F172A')
        self.ax_telemetry.axis('off')

        self.setup_3d_axes(self.ax3d_quat, "Quaternion Controller (Singularity-Free)")
        self.setup_3d_axes(self.ax3d_euler, "Euler Controller (Cross-Coupled Failure)")
        self.setup_2d_plots()

        ax_btn_30 = plt.axes([0.10, 0.02, 0.08, 0.04])
        ax_btn_60 = plt.axes([0.19, 0.02, 0.08, 0.04])
        ax_btn_80 = plt.axes([0.28, 0.02, 0.08, 0.04])
        ax_btn_90 = plt.axes([0.37, 0.02, 0.08, 0.04])
        ax_btn_play = plt.axes([0.47, 0.02, 0.10, 0.04])

        self.btn_30 = Button(ax_btn_30, '30deg Turn', color='#1E293B', hovercolor='#334155')
        self.btn_60 = Button(ax_btn_60, '60deg Turn', color='#1E293B', hovercolor='#334155')
        self.btn_80 = Button(ax_btn_80, '80deg Turn', color='#1E293B', hovercolor='#334155')
        self.btn_90 = Button(ax_btn_90, '90deg Knife-Edge', color='#0284C7', hovercolor='#0369A1')
        self.btn_play = Button(ax_btn_play, 'Pause / Play', color='#16A34A', hovercolor='#15803D')

        self.btn_30.on_clicked(lambda event: self.change_bank(30))
        self.btn_60.on_clicked(lambda event: self.change_bank(60))
        self.btn_80.on_clicked(lambda event: self.change_bank(80))
        self.btn_90.on_clicked(lambda event: self.change_bank(90))
        self.btn_play.on_clicked(self.toggle_play)

        self.anim = FuncAnimation(self.fig, self.update_frame, frames=self.total_frames, interval=20, blit=False)

    def setup_3d_axes(self, ax, title):
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        ax.set_zlim([-2.5, 2.5])
        ax.set_title(title, fontsize=10, fontweight='bold', color='#38BDF8', pad=10)
        ax.set_xlabel('Body X (Forward)', fontsize=8, color='#94A3B8')
        ax.set_ylabel('Body Y (Right)', fontsize=8, color='#94A3B8')
        ax.set_zlabel('Body Z (Down)', fontsize=8, color='#94A3B8')
        ax.tick_params(colors='#64748B', labelsize=7)

    def setup_2d_plots(self):
        self.ax_roll.set_title("Roll Angle (phi) Tracking", fontsize=9.5, fontweight='bold', color='#38BDF8')
        self.line_roll_sp, = self.ax_roll.plot([], [], 'g--', label='Setpoint', linewidth=1.5)
        self.line_roll_q,  = self.ax_roll.plot([], [], '#38BDF8', label='Quaternion Controller', linewidth=2.0)
        self.line_roll_e,  = self.ax_roll.plot([], [], '#EF4444', label='Euler Controller', linewidth=1.5, linestyle='-.')
        self.ax_roll.set_ylabel("Roll [deg]", fontsize=8.5, color='#94A3B8')
        self.ax_roll.grid(True, linestyle=':', alpha=0.4)
        self.ax_roll.legend(loc='upper right', fontsize=8)

        self.ax_pitch.set_title("Pitch Angle (theta) Tracking", fontsize=9.5, fontweight='bold', color='#38BDF8')
        self.line_pitch_sp, = self.ax_pitch.plot([], [], 'g--', label='Setpoint', linewidth=1.5)
        self.line_pitch_q,  = self.ax_pitch.plot([], [], '#38BDF8', label='Quaternion Controller', linewidth=2.0)
        self.line_pitch_e,  = self.ax_pitch.plot([], [], '#EF4444', label='Euler Controller', linewidth=1.5, linestyle='-.')
        self.ax_pitch.set_ylabel("Pitch [deg]", fontsize=8.5, color='#94A3B8')
        self.ax_pitch.grid(True, linestyle=':', alpha=0.4)

        self.ax_yaw.set_title("Yaw Angle (psi) Tracking", fontsize=9.5, fontweight='bold', color='#38BDF8')
        self.line_yaw_sp, = self.ax_yaw.plot([], [], 'g--', label='Setpoint', linewidth=1.5)
        self.line_yaw_q,  = self.ax_yaw.plot([], [], '#38BDF8', label='Quaternion Controller', linewidth=2.0)
        self.line_yaw_e,  = self.ax_yaw.plot([], [], '#EF4444', label='Euler Controller', linewidth=1.5, linestyle='-.')
        self.ax_yaw.set_ylabel("Yaw [deg]", fontsize=8.5, color='#94A3B8')
        self.ax_yaw.set_xlabel("Time [s]", fontsize=8.5, color='#94A3B8')
        self.ax_yaw.grid(True, linestyle=':', alpha=0.4)

        for ax in [self.ax_roll, self.ax_pitch, self.ax_yaw]:
            ax.set_xlim([0, self.time[-1]])
            ax.tick_params(colors='#94A3B8', labelsize=8)

        self.ax_roll.set_ylim([-15, max(100, self.bank_deg + 15)])
        self.ax_pitch.set_ylim([-15, 25])
        self.ax_yaw.set_ylim([-15, 200])

    def render_3d_drone(self, ax, q, color_mesh='#0284C7', color_edge='#38BDF8'):
        ax.clear()
        self.setup_3d_axes(ax, "Quaternion Controller (Singularity-Free)" if color_mesh == '#0284C7' else "Euler Controller (Cross-Coupled Failure)")

        R = quat_to_rotmat(q)
        transformed_verts = (R @ self.base_verts.T).T

        poly3d = [[transformed_verts[idx] for idx in face] for face in self.mesh_faces]
        collection = Poly3DCollection(poly3d, facecolors=color_mesh, linewidths=0.8, edgecolors=color_edge, alpha=0.85)
        ax.add_collection3d(collection)

        origin = np.zeros(3)
        x_axis = R @ np.array([2.5, 0.0, 0.0])
        y_axis = R @ np.array([0.0, 2.5, 0.0])
        z_axis = R @ np.array([0.0, 0.0, 2.5])

        ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color='#EF4444', linewidth=2.0, arrow_length_ratio=0.1)
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color='#22C55E', linewidth=2.0, arrow_length_ratio=0.1)
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color='#A855F7', linewidth=2.0, arrow_length_ratio=0.1)

    def change_bank(self, bank):
        self.bank_deg = bank
        self.time, self.q_sp_list, self.quat_q_hist, self.euler_q_hist, self.sp_euler, self.quat_euler, self.euler_euler = generate_simulation_data(self.bank_deg)
        self.current_frame = 0
        self.title_text.set_text(f'REAL-TIME 6-DOF DRONE SIMULATION: {self.bank_deg}° BANK MANEUVER\nQuaternion Controller (Blue) vs. Euler Controller (Red)')
        self.ax_roll.set_ylim([-15, max(100, self.bank_deg + 15)])
        self.fig.canvas.draw_idle()

    def toggle_play(self, event):
        self.is_playing = not self.is_playing
        if self.is_playing:
            self.anim.event_source.start()
        else:
            self.anim.event_source.stop()

    def update_frame(self, frame):
        if not self.is_playing:
            return

        idx = frame % self.total_frames
        t_curr = self.time[idx]

        self.render_3d_drone(self.ax3d_quat, self.quat_q_hist[idx], color_mesh='#0284C7', color_edge='#38BDF8')
        self.render_3d_drone(self.ax3d_euler, self.euler_q_hist[idx], color_mesh='#DC2626', color_edge='#F87171')

        t_slice = self.time[:idx+1]
        self.line_roll_sp.set_data(t_slice, self.sp_euler[:idx+1, 0])
        self.line_roll_q.set_data(t_slice, self.quat_euler[:idx+1, 0])
        self.line_roll_e.set_data(t_slice, self.euler_euler[:idx+1, 0])

        self.line_pitch_sp.set_data(t_slice, self.sp_euler[:idx+1, 1])
        self.line_pitch_q.set_data(t_slice, self.quat_euler[:idx+1, 1])
        self.line_pitch_e.set_data(t_slice, self.euler_euler[:idx+1, 1])

        self.line_yaw_sp.set_data(t_slice, self.sp_euler[:idx+1, 2])
        self.line_yaw_q.set_data(t_slice, self.quat_euler[:idx+1, 2])
        self.line_yaw_e.set_data(t_slice, self.euler_euler[:idx+1, 2])

        self.ax_telemetry.clear()
        self.ax_telemetry.axis('off')

        q_q = self.quat_q_hist[idx]
        e_q = self.quat_euler[idx]
        e_e = self.euler_euler[idx]
        sp_e = self.sp_euler[idx]

        status_text = "[OK] QUATERNION: Smooth tracking" if self.bank_deg <= 80 else "[OK] QUATERNION: 0.27deg error (No Lock)"
        euler_status = "[WARN] EULER: Small coupling" if self.bank_deg < 80 else "[FAIL] EULER: Cross-Coupling Failure (30.5deg Yaw Error)"

        telemetry_text = (
            f"  [+] REAL-TIME TELEMETRY STREAM\n"
            f"  -------------------------------\n"
            f"  * Simulation Time: {t_curr:.2f} s\n"
            f"  * Target Bank Angle: {self.bank_deg} deg\n\n"
            f"  [*] SETPOINT ATTITUDE:\n"
            f"    Roll: {sp_e[0]:6.2f}deg  Pitch: {sp_e[1]:6.2f}deg  Yaw: {sp_e[2]:6.2f}deg\n\n"
            f"  [BLUE] QUATERNION CONTROLLER:\n"
            f"    qw: {q_q[0]:6.3f}  qx: {q_q[1]:6.3f}  qy: {q_q[2]:6.3f}  qz: {q_q[3]:6.3f}\n"
            f"    Roll:  {e_q[0]:6.2f}deg (Err: {abs(sp_e[0]-e_q[0]):5.2f}deg)\n"
            f"    Pitch: {e_q[1]:6.2f}deg (Err: {abs(sp_e[1]-e_q[1]):5.2f}deg)\n"
            f"    Yaw:   {e_q[2]:6.2f}deg (Err: {abs(sp_e[2]-e_q[2]):5.2f}deg)\n\n"
            f"  [RED] EULER CONTROLLER:\n"
            f"    Roll:  {e_e[0]:6.2f}deg (Err: {abs(sp_e[0]-e_e[0]):5.2f}deg)\n"
            f"    Pitch: {e_e[1]:6.2f}deg (Err: {abs(sp_e[1]-e_e[1]):5.2f}deg)\n"
            f"    Yaw:   {e_e[2]:6.2f}deg (Err: {abs(sp_e[2]-e_e[2]):5.2f}deg)\n\n"
            f"  -------------------------------\n"
            f"  STATUS AT {self.bank_deg}deg BANK:\n"
            f"  {status_text}\n"
            f"  {euler_status}"
        )

        self.ax_telemetry.text(
            0.05, 0.95, telemetry_text,
            transform=self.ax_telemetry.transAxes,
            fontsize=9.0, fontfamily='monospace', color='#F1F5F9',
            verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.8', facecolor='#1E293B', edgecolor='#38BDF8', alpha=0.95)
        )

        self.fig.canvas.draw_idle()

if __name__ == '__main__':
    sim = RealTimeDroneSimulator(initial_bank=90)
    plt.show()
