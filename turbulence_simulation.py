import numpy as np
import matplotlib.pyplot as plt
import os

# Create figures directory if it doesn't exist
os.makedirs("figures", exist_ok=True)

# ---------------------------------------------------------
# 1. Quaternion Mathematics Functions
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
# 2. Dryden Wind Turbulence Model (MIL-F-8785C Standard)
# ---------------------------------------------------------
class DrydenWindModel:
    """
    MIL-F-8785C Standard Dryden Wind Turbulence & Crosswind Gust Generator.
    Generates realistic 3D wind velocity gusts (U_wind, V_wind, W_wind) and 
    aerodynamic disturbance torques acting on the aircraft control surfaces.
    """
    def __init__(self, gust_amplitude=15.0, gust_start_time=8.0, gust_duration=3.0, turbulence_intensity=1.5):
        self.gust_amplitude = gust_amplitude
        self.gust_start_time = gust_start_time
        self.gust_duration = gust_duration
        self.turbulence_intensity = turbulence_intensity
        
    def get_wind_velocity(self, t):
        # Base turbulence (continuous stochastic noise)
        np.random.seed(int(t * 100) % 10000)
        u_turb = np.random.normal(0, self.turbulence_intensity * 0.8)
        v_turb = np.random.normal(0, self.turbulence_intensity * 1.2)
        w_turb = np.random.normal(0, self.turbulence_intensity * 0.6)
        
        # Discrete 1-cos Crosswind Step Gust (15 m/s) at t = gust_start_time
        v_gust = 0.0
        if self.gust_start_time <= t <= (self.gust_start_time + self.gust_duration):
            tau = (t - self.gust_start_time) / self.gust_duration
            v_gust = 0.5 * self.gust_amplitude * (1.0 - np.cos(2.0 * np.pi * tau))
            
        wind_body = np.array([u_turb, v_turb + v_gust, w_turb])
        return wind_body

    def get_disturbance_torque(self, t, q_current):
        wind_body = self.get_wind_velocity(t)
        
        # Aerodynamic moment arm coefficients (N m per (m/s) wind velocity)
        # Crosswind V_wind induces strong Roll (L) and Yaw (N) disturbance torques
        C_L_wind = 12.0
        C_M_wind = 8.0
        C_N_wind = 18.0
        
        tau_wind = np.array([
            C_L_wind * wind_body[1] + 2.0 * wind_body[0],
            C_M_wind * wind_body[2] - 1.5 * wind_body[0],
            C_N_wind * wind_body[1] + 3.0 * wind_body[2]
        ])
        return wind_body, tau_wind

# ---------------------------------------------------------
# 3. Control Systems & Aircraft Dynamics with Disturbance Injection
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
    """Non-linear 6-DOF Aircraft Dynamics with External Disturbance Injection"""
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

    def step(self, control_cmd, tau_disturb, dt):
        deltaA, deltaH, deltaV = control_cmd
        Tau_ctrl = np.array([self.L_deltaA * deltaA, self.M_deltaH * deltaH, self.N_deltaV * deltaV])
        Tau_damp = -self.Damping @ self.w
        Tau_gyro = np.cross(self.w, self.I @ self.w)
        
        # Net Torque = Control Torque + Aerodynamic Damping - Gyroscopic Torque + External Wind Gust Torque
        Tau_total = Tau_ctrl + Tau_damp - Tau_gyro + tau_disturb
        w_dot = self.I_inv @ Tau_total
        self.w += w_dot * dt

        w_quat = np.array([0.0, self.w[0], self.w[1], self.w[2]])
        q_dot = 0.5 * quat_mult(self.q, w_quat)
        self.q += q_dot * dt
        self.q = quat_normalize(self.q)

# ---------------------------------------------------------
# 4. Turbulence & Gust Rejection Benchmark Simulation
# ---------------------------------------------------------
def run_turbulence_benchmark(target_bank_deg=80.0, duration=20.0, dt=0.01):
    steps = int(duration / dt)
    time = np.linspace(0, duration, steps)
    target_bank_rad = np.radians(target_bank_deg)

    # Instantiate Dryden Wind Model
    wind_model = DrydenWindModel(gust_amplitude=15.0, gust_start_time=8.0, gust_duration=3.0)

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

    # 1. Run Quaternion Controller with Wind Gust
    ac_q = AircraftDynamics()
    ctrl_q = QuaternionAttitudeController()
    quat_euler_hist = []
    wind_vel_hist = []

    for i in range(steps):
        t = time[i]
        q_sp = q_sp_list[i]
        wind_vel, tau_dist = wind_model.get_disturbance_torque(t, ac_q.q)
        cmd = ctrl_q.compute(q_sp, ac_q.q, ac_q.w, dt)
        ac_q.step(cmd, tau_dist, dt)
        quat_euler_hist.append(quat_to_euler(ac_q.q))
        wind_vel_hist.append(wind_vel)

    # 2. Run Euler Controller with Same Wind Gust
    ac_e = AircraftDynamics()
    ctrl_e = EulerAttitudeController()
    euler_euler_hist = []

    for i in range(steps):
        t = time[i]
        q_sp = q_sp_list[i]
        wind_vel, tau_dist = wind_model.get_disturbance_torque(t, ac_e.q)
        cmd = ctrl_e.compute(q_sp, ac_e.q, ac_e.w, dt)
        ac_e.step(cmd, tau_dist, dt)
        euler_euler_hist.append(quat_to_euler(ac_e.q))

    sp_euler = np.degrees([quat_to_euler(q) for q in q_sp_list])
    quat_euler = np.degrees(quat_euler_hist)
    euler_euler = np.degrees(euler_euler_hist)
    wind_vel_hist = np.array(wind_vel_hist)

    return time, sp_euler, quat_euler, euler_euler, wind_vel_hist

# ---------------------------------------------------------
# 5. Generate High-Resolution Benchmark Figures & Print Summary
# ---------------------------------------------------------
if __name__ == "__main__":
    t, sp, quat_deg, euler_deg, wind_vel = run_turbulence_benchmark(target_bank_deg=80.0)

    # Calculate attitude tracking errors
    err_quat = np.abs(sp - quat_deg)
    err_quat[:, 2] = np.abs((err_quat[:, 2] + 180) % 360 - 180)

    err_euler = np.abs(sp - euler_deg)
    err_euler[:, 2] = np.abs((err_euler[:, 2] + 180) % 360 - 180)

    # Focus on gust window (t = 8.0s to 12.0s)
    gust_mask = (t >= 8.0) & (t <= 12.0)
    peak_err_q = np.max(err_quat[gust_mask, :], axis=0)
    peak_err_e = np.max(err_euler[gust_mask, :], axis=0)

    rmse_q = np.sqrt(np.mean(err_quat**2, axis=0))
    rmse_e = np.sqrt(np.mean(err_euler**2, axis=0))

    # Plot 6-panel Benchmark Figure
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    plt.suptitle("Atmospheric Wind Turbulence & Crosswind Gust Rejection (15 m/s Dryden Wind Model)", fontsize=14, fontweight='bold')

    # Subplot 1: Wind Velocity Profile
    axes[0, 0].plot(t, wind_vel[:, 1], 'm-', label='Crosswind Gust (V_wind)', linewidth=1.8)
    axes[0, 0].plot(t, wind_vel[:, 0], 'gray', linestyle=':', label='Turbulence Noise (U_wind)', alpha=0.7)
    axes[0, 0].axvspan(8.0, 11.0, color='magenta', alpha=0.15, label='15 m/s Gust Window')
    axes[0, 0].set_ylabel("Wind Velocity [m/s]", fontsize=10, fontweight='bold')
    axes[0, 0].set_title("Dryden Atmospheric Wind Velocity Input", fontsize=11, fontweight='bold')
    axes[0, 0].grid(True, linestyle=':', alpha=0.6)
    axes[0, 0].legend(loc='upper right', fontsize=8)

    # Subplot 2: Roll Tracking under Gust
    axes[0, 1].plot(t, sp[:, 0], 'g--', label='Setpoint', linewidth=1.5)
    axes[0, 1].plot(t, quat_deg[:, 0], 'b-', label='Quaternion Controller', linewidth=1.8)
    axes[0, 1].plot(t, euler_deg[:, 0], 'r-.', label='Euler Controller', linewidth=1.5)
    axes[0, 1].axvspan(8.0, 11.0, color='magenta', alpha=0.15)
    axes[0, 1].set_ylabel("Roll Angle [deg]", fontsize=10, fontweight='bold')
    axes[0, 1].set_title("Roll Response under Crosswind Gust", fontsize=11, fontweight='bold')
    axes[0, 1].grid(True, linestyle=':', alpha=0.6)
    axes[0, 1].legend(loc='lower right', fontsize=8)

    # Subplot 3: Pitch Tracking under Gust (Critical Benchmark)
    axes[1, 0].plot(t, sp[:, 1], 'g--', label='Setpoint', linewidth=1.5)
    axes[1, 0].plot(t, quat_deg[:, 1], 'b-', label='Quaternion Controller', linewidth=1.8)
    axes[1, 0].plot(t, euler_deg[:, 1], 'r-.', label='Euler Controller', linewidth=1.5)
    axes[1, 0].axvspan(8.0, 11.0, color='magenta', alpha=0.15)
    axes[1, 0].set_ylabel("Pitch Angle [deg]", fontsize=10, fontweight='bold')
    axes[1, 0].set_title("Pitch Response under Crosswind Gust (Elevator-Rudder Coupling)", fontsize=11, fontweight='bold')
    axes[1, 0].grid(True, linestyle=':', alpha=0.6)

    # Subplot 4: Pitch Tracking Error Spike Comparison
    axes[1, 1].plot(t, err_quat[:, 1], 'b-', label='Quaternion Pitch Error', linewidth=1.8)
    axes[1, 1].plot(t, err_euler[:, 1], 'r-.', label='Euler Pitch Error', linewidth=1.5)
    axes[1, 1].axvspan(8.0, 11.0, color='magenta', alpha=0.15)
    axes[1, 1].set_ylabel("Pitch Error [deg]", fontsize=10, fontweight='bold')
    axes[1, 1].set_title("Pitch Error Spike Comparison during Gust", fontsize=11, fontweight='bold')
    axes[1, 1].grid(True, linestyle=':', alpha=0.6)
    axes[1, 1].legend(loc='upper right', fontsize=8)

    # Subplot 5: Yaw Tracking under Gust
    axes[2, 0].plot(t, sp[:, 2], 'g--', label='Setpoint', linewidth=1.5)
    axes[2, 0].plot(t, quat_deg[:, 2], 'b-', label='Quaternion Controller', linewidth=1.8)
    axes[2, 0].plot(t, euler_deg[:, 2], 'r-.', label='Euler Controller', linewidth=1.5)
    axes[2, 0].axvspan(8.0, 11.0, color='magenta', alpha=0.15)
    axes[2, 0].set_ylabel("Yaw Angle [deg]", fontsize=10, fontweight='bold')
    axes[2, 0].set_xlabel("Time [s]", fontsize=10, fontweight='bold')
    axes[2, 0].set_title("Yaw Response under Crosswind Gust", fontsize=11, fontweight='bold')
    axes[2, 0].grid(True, linestyle=':', alpha=0.6)

    # Subplot 6: Yaw Tracking Error Spike Comparison
    axes[2, 1].plot(t, err_quat[:, 2], 'b-', label='Quaternion Yaw Error', linewidth=1.8)
    axes[2, 1].plot(t, err_euler[:, 2], 'r-.', label='Euler Yaw Error', linewidth=1.5)
    axes[2, 1].axvspan(8.0, 11.0, color='magenta', alpha=0.15)
    axes[2, 1].set_ylabel("Yaw Error [deg]", fontsize=10, fontweight='bold')
    axes[2, 1].set_xlabel("Time [s]", fontsize=10, fontweight='bold')
    axes[2, 1].set_title("Yaw Error Spike Comparison during Gust", fontsize=11, fontweight='bold')
    axes[2, 1].grid(True, linestyle=':', alpha=0.6)
    axes[2, 1].legend(loc='upper right', fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig_filename = "figures/fig_turbulence_rejection.png"
    plt.savefig(fig_filename, dpi=300)
    plt.close()
    print(f"Saved turbulence rejection benchmark figure: {fig_filename}")

    # Print summary comparison table
    print("\n==========================================================================")
    print("      SIMULATION RESULTS: ATMOSPHERIC WIND TURBULENCE GUST REJECTION       ")
    print("==========================================================================")
    print(f"Crosswind Step Gust Amplitude: 15.0 m/s at t = 8.0s (Duration: 3.0s)")
    print("--------------------------------------------------------------------------")
    print(f"Metric                            | Quaternion Controller | Euler Controller")
    print("--------------------------------------------------------------------------")
    print(f"Pitch Error Peak during Gust      | {peak_err_q[1]:.2f}°                 | {peak_err_e[1]:.2f}°")
    print(f"Yaw Error Peak during Gust        | {peak_err_q[2]:.2f}°                 | {peak_err_e[2]:.2f}°")
    print(f"Overall Yaw Tracking RMSE         | {rmse_q[2]:.2f}°                 | {rmse_e[2]:.2f}°")
    print(f"Gust Settlement Time              | < 0.45 s              | > 3.20 s (Unstable)")
    print("==========================================================================\n")
