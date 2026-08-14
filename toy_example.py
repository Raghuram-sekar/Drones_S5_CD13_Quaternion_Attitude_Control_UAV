import numpy as np

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

def run_toy_example():
    print("==========================================================================")
    print("      NUMERICAL TOY EXAMPLE: QUATERNION ATTITUDE CONTROL EVALUATION       ")
    print("==========================================================================")
    
    # Initial state: Aircraft is banked at 90 degrees roll (knife-edge orientation)
    # q_meas corresponds to Roll = 90 deg, Pitch = 0 deg, Yaw = 0 deg
    # q = [cos(45 deg), sin(45 deg), 0, 0] = [0.7071, 0.7071, 0, 0]
    q_meas = np.array([0.70710678, 0.70710678, 0.0, 0.0])
    
    # Desired setpoint: Setpoint attitude with Roll = 0 deg, Pitch = 30 deg, Yaw = 0 deg
    # q_sp = [cos(15 deg), 0, sin(15 deg), 0] = [0.9659, 0, 0.2588, 0]
    q_sp = np.array([0.96592583, 0.0, 0.25881905, 0.0])
    
    print(f"\nStep 1: Input Quaternions")
    print(f"  Measured Attitude Quaternion (q_meas) : [qw={q_meas[0]:.4f}, qx={q_meas[1]:.4f}, qy={q_meas[2]:.4f}, qz={q_meas[3]:.4f}]")
    print(f"  Setpoint Attitude Quaternion (q_sp)   : [qw={q_sp[0]:.4f}, qx={q_sp[1]:.4f}, qy={q_sp[2]:.4f}, qz={q_sp[3]:.4f}]")
    
    # Step 2: Calculate Conjugate q_meas*
    q_meas_conj = quat_conj(q_meas)
    print(f"\nStep 2: Measured Quaternion Conjugate (q_meas*)")
    print(f"  q_meas* : [qw={q_meas_conj[0]:.4f}, qx={q_meas_conj[1]:.4f}, qy={q_meas_conj[2]:.4f}, qz={q_meas_conj[3]:.4f}]")
    
    # Step 3: Compute Attitude Error Quaternion via Hamilton Product
    # q_err = q_meas* (x) q_sp
    q_err = quat_mult(q_meas_conj, q_sp)
    print(f"\nStep 3: Compute Attitude Error Quaternion (q_err = q_meas* (x) q_sp)")
    print(f"  q_err   : [qw={q_err[0]:.4f}, qx={q_err[1]:.4f}, qy={q_err[2]:.4f}, qz={q_err[3]:.4f}]")
    
    # Step 4: Shortest-Path Rotation Check
    print(f"\nStep 4: Shortest-Path Check (qw_err >= 0)")
    if q_err[0] < 0:
        q_err_short = -q_err
        print(f"  qw_err < 0: Inverted quaternion for shortest path arc <= 180°")
    else:
        q_err_short = q_err.copy()
        print(f"  qw_err >= 0 ({q_err[0]:.4f}): Original quaternion is already shortest path arc")
    print(f"  q_err_short : [qw={q_err_short[0]:.4f}, qx={q_err_short[1]:.4f}, qy={q_err_short[2]:.4f}, qz={q_err_short[3]:.4f}]")
    
    # Step 5: Calculate Proportional Angular Rate Setpoints
    Kp = 3.5
    w_sp = 2.0 * Kp * q_err_short[1:4]
    print(f"\nStep 5: Compute Angular Rate Setpoints (w_sp = 2 * Kp * q_v_err_short)")
    print(f"  Gain Kp = {Kp}")
    print(f"  Roll Rate Setpoint (w_x_sp)  : 2 * {Kp} * ({q_err_short[1]:.4f}) = {w_sp[0]:.4f} rad/s ({np.degrees(w_sp[0]):.2f} deg/s)")
    print(f"  Pitch Rate Setpoint (w_y_sp) : 2 * {Kp} * ({q_err_short[2]:.4f}) = {w_sp[1]:.4f} rad/s ({np.degrees(w_sp[1]):.2f} deg/s)")
    print(f"  Yaw Rate Setpoint (w_z_sp)   : 2 * {Kp} * ({q_err_short[3]:.4f}) = {w_sp[2]:.4f} rad/s ({np.degrees(w_sp[2]):.2f} deg/s)")
    
    print("\n==========================================================================")
    print("  PHYSICAL INSIGHT: At 90° roll, the quaternion controller automatically  ")
    print("  generates roll, pitch, AND yaw rate commands to decouple structural    ")
    print("  body axes without requiring explicit Euler angle conversion or logic!    ")
    print("==========================================================================\n")

if __name__ == "__main__":
    run_toy_example()
