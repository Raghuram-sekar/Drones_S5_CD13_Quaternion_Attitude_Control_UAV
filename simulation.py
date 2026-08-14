import numpy as np
import matplotlib.pyplot as plt
import os

# Create figures directory
os.makedirs("figures", exist_ok=True)

# ---------------------------------------------------------
# 1. Quaternion Mathematics Functions
# ---------------------------------------------------------
def quat_mult(p, q):
    """Hamilton product p (x) q"""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw
    ])

def quat_conj(q):
    """Quaternion conjugate q*"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_norm(q):
    """Quaternion norm"""
    return np.linalg.norm(q)

def quat_normalize(q):
    """Normalize quaternion to unit length"""
    n = quat_norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n

def euler_to_quat(roll, pitch, yaw):
    """Convert Euler angles (roll phi, pitch theta, yaw psi in radians) to unit quaternion ZYX order"""
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
    """Convert unit quaternion to Euler angles (roll, pitch, yaw in radians)"""
    qw, qx, qy, qz = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def slerp(q0, q1, t):
    """Spherical Linear Interpolation (SLERP) between q0 and q1 at parameter t in [0, 1]"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
        
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = q0 + t * (q1 - q0)
        return quat_normalize(result)
        
    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    
    return quat_normalize((s0 * q0) + (s1 * q1))

# ---------------------------------------------------------
# 2. Control System Algorithms
# ---------------------------------------------------------
class QuaternionAttitudeController:
    """Outer Q_P attitude controller + Inner 3-axis rate PID controller"""
    def __init__(self, Kp_att=3.5, P_rate=(8.0, 8.0, 8.0), I_rate=(0.5, 0.5, 0.5), D_rate=(0.8, 0.8, 0.8), rate_limit=2.5):
        self.Kp_att = Kp_att
        self.P_rate = np.array(P_rate)
        self.I_rate = np.array(I_rate)
        self.D_rate = np.array(D_rate)
        self.rate_limit = rate_limit
        self.integral = np.zeros(3)
        
    def reset(self):
        self.integral = np.zeros(3)

    def compute(self, q_sp, q_meas, w_meas, dt, airspeed=30.0, airspeed_ref=30.0):
        # 1. Compute attitude error quaternion: q_err = q_meas* (x) q_sp
        q_meas_conj = quat_conj(q_meas)
        q_err = quat_mult(q_meas_conj, q_sp)
        
        # 2. Shortest path check
        if q_err[0] < 0.0:
            q_err_short = -q_err
        else:
            q_err_short = q_err.copy()
            
        # 3. Outer loop Q_P controller output: w_sp = 2 * Kp * q_v_err_short
        w_sp = 2.0 * self.Kp_att * q_err_short[1:4]
        
        # Apply rate limits (saturation)
        w_sp = np.clip(w_sp, -self.rate_limit, self.rate_limit)
        
        # 4. Inner loop rate PID controller
        rate_err = w_sp - w_meas
        self.integral += rate_err * dt
        # Anti-windup
        self.integral = np.clip(self.integral, -1.0, 1.0)
        
        # Output: PI on rate error, D on measured body rate
        control_cmd = (self.P_rate * rate_err + 
                       self.I_rate * self.integral - 
                       self.D_rate * w_meas)
                       
        # Dynamic gain scaling by square of airspeed (aerodynamic scaling)
        speed_scale = (airspeed_ref / max(airspeed, 1.0))**2
        control_cmd *= speed_scale
        
        # Control surface deflections: [delta_A (aileron), delta_H (elevator), delta_V (rudder)]
        control_cmd = np.clip(control_cmd, -1.0, 1.0)
        return control_cmd, w_sp

class EulerAttitudeController:
    """Conventional cascaded Euler angle P controller + Inner 3-axis rate PID controller"""
    def __init__(self, Kp_att=3.5, P_rate=(8.0, 8.0, 8.0), I_rate=(0.5, 0.5, 0.5), D_rate=(0.8, 0.8, 0.8), rate_limit=2.5):
        self.Kp_att = Kp_att
        self.P_rate = np.array(P_rate)
        self.I_rate = np.array(I_rate)
        self.D_rate = np.array(D_rate)
        self.rate_limit = rate_limit
        self.integral = np.zeros(3)

    def reset(self):
        self.integral = np.zeros(3)

    def compute(self, q_sp, q_meas, w_meas, dt, airspeed=30.0, airspeed_ref=30.0):
        # Convert quaternions to Euler angles
        r_sp, p_sp, y_sp = quat_to_euler(q_sp)
        r_meas, p_meas, y_meas = quat_to_euler(q_meas)
        
        # Compute Euler angle errors (wrapping yaw error to [-pi, pi])
        r_err = r_sp - r_meas
        p_err = p_sp - p_meas
        y_err = y_sp - y_meas
        y_err = (y_err + np.pi) % (2 * np.pi) - np.pi
        
        euler_err = np.array([r_err, p_err, y_err])
        
        # Outer loop proportional controller
        w_sp = self.Kp_att * euler_err
        w_sp = np.clip(w_sp, -self.rate_limit, self.rate_limit)
        
        # Inner loop rate PID controller (same as quaternion controller for fair evaluation)
        rate_err = w_sp - w_meas
        self.integral += rate_err * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        
        control_cmd = (self.P_rate * rate_err + 
                       self.I_rate * self.integral - 
                       self.D_rate * w_meas)
                       
        speed_scale = (airspeed_ref / max(airspeed, 1.0))**2
        control_cmd *= speed_scale
        control_cmd = np.clip(control_cmd, -1.0, 1.0)
        return control_cmd, w_sp

# ---------------------------------------------------------
# 3. Aircraft Dynamic Simulation Environment (6-DOF Rotational Dynamics)
# ---------------------------------------------------------
class AircraftDynamics:
    """Extra 330SC aerobatic aircraft dynamics model"""
    def __init__(self):
        # Moments of Inertia (kg m^2)
        self.Ixx = 180.0
        self.Iyy = 220.0
        self.Izz = 350.0
        self.I = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.I_inv = np.linalg.inv(self.I)
        
        # Control authority moments (N m per unit deflection)
        self.L_deltaA = 450.0  # Roll control moment
        self.M_deltaH = 520.0  # Pitch control moment
        self.N_deltaV = 380.0  # Yaw control moment
        
        # Damping coefficients
        self.Damping = np.diag([120.0, 140.0, 160.0])
        
        self.reset()

    def reset(self, q_init=None):
        if q_init is None:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            self.q = quat_normalize(np.array(q_init))
        self.w = np.zeros(3)  # Body angular rate vector [wx, wy, wz]

    def step(self, control_cmd, dt):
        deltaA, deltaH, deltaV = control_cmd
        
        # Applied control torque
        Tau_ctrl = np.array([
            self.L_deltaA * deltaA,
            self.M_deltaH * deltaH,
            self.N_deltaV * deltaV
        ])
        
        # Aerodynamic rate damping torque
        Tau_damp = -self.Damping @ self.w
        
        # Gyroscopic cross-product torque: w x (I w)
        Tau_gyro = np.cross(self.w, self.I @ self.w)
        
        # Net angular acceleration
        w_dot = self.I_inv @ (Tau_ctrl + Tau_damp - Tau_gyro)
        
        # Euler integration for body angular rate
        self.w += w_dot * dt
        
        # Quaternion kinematic integration: q_dot = 0.5 * q (x) [0, w]
        qw, qx, qy, qz = self.q
        w_quat = np.array([0.0, self.w[0], self.w[1], self.w[2]])
        q_dot = 0.5 * quat_mult(self.q, w_quat)
        
        self.q += q_dot * dt
        self.q = quat_normalize(self.q)

# ---------------------------------------------------------
# 4. Scenario Simulation Runner
# ---------------------------------------------------------
def run_scenario(target_bank_deg, duration=20.0, dt=0.01):
    """Run turn maneuver simulation at specified bank angle target"""
    t_steps = int(duration / dt)
    time_grid = np.linspace(0, duration, t_steps)
    
    target_bank_rad = np.radians(target_bank_deg)
    
    q_setpoints = []
    for t in time_grid:
        if t < 2.0:
            roll = 0.0
            pitch = np.radians(2.0)
            yaw = 0.0
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
            roll = 0.0
            pitch = np.radians(2.0)
            yaw = np.radians(165.0)
            
        q_sp = euler_to_quat(roll, pitch, yaw)
        q_setpoints.append(q_sp)
        
    # Run Quaternion Controller Simulation
    ac_q = AircraftDynamics()
    ctrl_q = QuaternionAttitudeController()
    q_hist_quat = []
    w_hist_quat = []
    euler_hist_quat = []
    
    for i in range(t_steps):
        q_sp = q_setpoints[i]
        cmd, _ = ctrl_q.compute(q_sp, ac_q.q, ac_q.w, dt)
        ac_q.step(cmd, dt)
        
        q_hist_quat.append(ac_q.q.copy())
        w_hist_quat.append(ac_q.w.copy())
        euler_hist_quat.append(quat_to_euler(ac_q.q))
        
    # Run Euler Controller Simulation
    ac_e = AircraftDynamics()
    ctrl_e = EulerAttitudeController()
    q_hist_euler = []
    w_hist_euler = []
    euler_hist_euler = []
    
    for i in range(t_steps):
        q_sp = q_setpoints[i]
        cmd, _ = ctrl_e.compute(q_sp, ac_e.q, ac_e.w, dt)
        ac_e.step(cmd, dt)
        
        q_hist_euler.append(ac_e.q.copy())
        w_hist_euler.append(ac_e.w.copy())
        euler_hist_euler.append(quat_to_euler(ac_e.q))

    # Convert histories to numpy arrays
    euler_sp = np.array([quat_to_euler(q) for q in q_setpoints])
    euler_quat = np.array(euler_hist_quat)
    euler_euler = np.array(euler_hist_euler)
    
    return time_grid, np.degrees(euler_sp), np.degrees(euler_quat), np.degrees(euler_euler)

# ---------------------------------------------------------
# 5. Execute All Bank Angle Scenarios & Generate Figures
# ---------------------------------------------------------
if __name__ == "__main__":
    scenarios = [30, 60, 80, 90]
    rmse_results = {}

    for bank in scenarios:
        t, sp, quat_deg, euler_deg = run_scenario(bank)
        
        # Calculate attitude tracking errors (degrees)
        err_quat = np.abs(sp - quat_deg)
        err_quat[:, 2] = np.abs((err_quat[:, 2] + 180) % 360 - 180)
        
        err_euler = np.abs(sp - euler_deg)
        err_euler[:, 2] = np.abs((err_euler[:, 2] + 180) % 360 - 180)
        
        rmse_q = np.sqrt(np.mean(err_quat**2, axis=0))
        rmse_e = np.sqrt(np.mean(err_euler**2, axis=0))
        
        rmse_results[bank] = {'quat': rmse_q, 'euler': rmse_e}
        
        # Plot tracking performance & error
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        plt.suptitle(f"Attitude Control Performance Benchmarking: {bank}° Bank Angle Turn", fontsize=14, fontweight='bold')
        
        titles = ['Roll Angle (\\phi)', 'Pitch Angle (\\theta)', 'Yaw Angle (\\psi)']
        
        for idx in range(3):
            # Left column: Setpoint tracking
            axes[idx, 0].plot(t, sp[:, idx], 'g--', label='Setpoint', linewidth=1.8)
            axes[idx, 0].plot(t, quat_deg[:, idx], 'b-', label='Quaternion Controller', linewidth=1.8)
            axes[idx, 0].plot(t, euler_deg[:, idx], 'r-.', label='Euler Controller', linewidth=1.5)
            axes[idx, 0].set_ylabel(f"{titles[idx]} [deg]")
            axes[idx, 0].grid(True, linestyle=':', alpha=0.6)
            if idx == 0:
                axes[idx, 0].set_title("Setpoint Tracking Response")
                axes[idx, 0].legend(loc='best')
            if idx == 2:
                axes[idx, 0].set_xlabel("Time [s]")
                
            # Right column: Tracking Error Comparison
            axes[idx, 1].plot(t, err_quat[:, idx], 'b-', label='Quaternion Error', linewidth=1.8)
            axes[idx, 1].plot(t, err_euler[:, idx], 'r-.', label='Euler Error', linewidth=1.5)
            axes[idx, 1].set_ylabel(f"{titles[idx]} Error [deg]")
            axes[idx, 1].grid(True, linestyle=':', alpha=0.6)
            if idx == 0:
                axes[idx, 1].set_title("Tracking Error Comparison")
                axes[idx, 1].legend(loc='best')
            if idx == 2:
                axes[idx, 1].set_xlabel("Time [s]")
                
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        fig_filename = f"figures/fig_{bank}deg_turn.png"
        plt.savefig(fig_filename, dpi=300)
        plt.close()
        print(f"Saved figure: {fig_filename}")

    # Generate overall error comparison plot
    fig, ax = plt.subplots(figsize=(10, 6))
    bar_width = 0.35
    x = np.arange(len(scenarios))

    quat_yaw_rmse = [rmse_results[b]['quat'][2] for b in scenarios]
    euler_yaw_rmse = [rmse_results[b]['euler'][2] for b in scenarios]

    rects1 = ax.bar(x - bar_width/2, quat_yaw_rmse, bar_width, label='Quaternion Controller', color='#1f77b4')
    rects2 = ax.bar(x + bar_width/2, euler_yaw_rmse, bar_width, label='Euler Controller', color='#d62728')

    ax.set_ylabel('Yaw Tracking RMSE Error [deg]', fontsize=12, fontweight='bold')
    ax.set_title('Yaw Tracking Accuracy vs. Bank Angle (Demonstrating Cross-Coupling Resistance)', fontsize=13, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([f"{b}° Bank" for b in scenarios], fontsize=11, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, linestyle=':', alpha=0.6, axis='y')

    for rect in rects1:
        height = rect.get_height()
        ax.annotate(f'{height:.2f}°', xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)

    for rect in rects2:
        height = rect.get_height()
        ax.annotate(f'{height:.2f}°', xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    fig_err_filename = "figures/fig_error_comparison.png"
    plt.savefig(fig_err_filename, dpi=300)
    plt.close()
    print(f"Saved comparison figure: {fig_err_filename}")

    # Print summary comparison table
    print("\n==========================================================================")
    print("             SIMULATION RESULTS: ATTITUDE TRACKING RMSE (DEGREES)         ")
    print("==========================================================================")
    print(f"{'Bank Angle':<12} | {'Quaternion Controller (R / P / Y)':<32} | {'Euler Controller (R / P / Y)':<32}")
    print("--------------------------------------------------------------------------")
    for b in scenarios:
        q_r, q_p, q_y = rmse_results[b]['quat']
        e_r, e_p, e_y = rmse_results[b]['euler']
        print(f"{b:>2}° Bank Turn  | Roll: {q_r:.2f}°, Pitch: {q_p:.2f}°, Yaw: {q_y:.2f}°  | Roll: {e_r:.2f}°, Pitch: {e_p:.2f}°, Yaw: {e_y:.2f}°")
    print("==========================================================================\n")
