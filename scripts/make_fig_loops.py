import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Set global styles
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 11

def create_cascaded_loops_diagram():
    fig, ax = plt.subplots(figsize=(14, 7.5), dpi=300)
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8.5)
    ax.axis('off')

    # Background canvas
    fig.patch.set_facecolor('#F8FAFC')
    ax.set_facecolor('#F8FAFC')

    # Title
    ax.text(7.0, 8.1, "HOW THE DUAL CONTROL LOOPS WORK (INTUITIVE ARCHITECTURE)", 
            fontsize=16, fontweight='bold', ha='center', va='center', color='#0F172A')
    ax.text(7.0, 7.7, "Outer Attitude Loop determines WHERE to point and HOW FAST to spin  |  Inner Rate Loop drives MOTORS to hit that speed", 
            fontsize=11, ha='center', va='center', color='#475569', style='italic')

    # ==================== OUTER LOOP BOX ====================
    outer_box = patches.FancyBboxPatch((0.5, 2.2), 4.8, 5.0,
                                       boxstyle="round,pad=0.2,rounding_size=0.3",
                                       linewidth=2, edgecolor='#0284C7', facecolor='#F0F9FF')
    ax.add_patch(outer_box)
    ax.text(2.9, 6.9, "1. OUTER ATTITUDE LOOP (Q_P)", fontsize=13, fontweight='bold', color='#0369A1', ha='center')
    ax.text(2.9, 6.55, "Runs at 50 Hz  •  Goal: Find Target Spin Speed", fontsize=9.5, color='#0284C7', ha='center')

    # Step 1: Compare Target & Measured Attitude
    sub1 = patches.FancyBboxPatch((0.8, 4.7), 4.2, 1.5,
                                  boxstyle="round,pad=0.1,rounding_size=0.15",
                                  linewidth=1.2, edgecolor='#38BDF8', facecolor='#FFFFFF')
    ax.add_patch(sub1)
    ax.text(2.9, 5.85, "Step 1A: Where do we want to point?", fontsize=10.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(2.9, 5.45, "Target (q_sp) vs Measured (q_meas)", fontsize=9.5, color='#334155', ha='center')
    ax.text(2.9, 5.0, "Attitude Error:  q_err = q_meas* (x) q_sp", fontsize=10, fontweight='bold', color='#0284C7', ha='center')

    # Arrow inside outer loop
    ax.annotate('', xy=(2.9, 4.3), xytext=(2.9, 4.7),
                arrowprops=dict(facecolor='#0284C7', edgecolor='#0284C7', width=2, headwidth=7, headlength=7))

    # Step 2: Q_P Proportional Law
    sub2 = patches.FancyBboxPatch((0.8, 2.6), 4.2, 1.6,
                                  boxstyle="round,pad=0.1,rounding_size=0.15",
                                  linewidth=1.2, edgecolor='#38BDF8', facecolor='#FFFFFF')
    ax.add_patch(sub2)
    ax.text(2.9, 3.85, "Step 1B: How fast should we spin to get there?", fontsize=10.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(2.9, 3.45, "Multiply 3D vector error by 2 * K_p", fontsize=9.5, color='#334155', ha='center')
    ax.text(2.9, 2.95, "Target Spin Speed:  w_sp = 2 * K_p * q_v,err", fontsize=10, fontweight='bold', color='#0369A1', ha='center')

    # ==================== CONNECTOR TO INNER LOOP ====================
    ax.annotate('', xy=(5.9, 3.4), xytext=(5.3, 3.4),
                arrowprops=dict(facecolor='#0284C7', edgecolor='#0284C7', width=3, headwidth=9, headlength=9))
    ax.text(5.6, 3.8, "Target Speed\n[w_x, w_y, w_z]_sp", fontsize=9, fontweight='bold', color='#0369A1', ha='center', va='bottom')

    # ==================== INNER LOOP BOX ====================
    inner_box = patches.FancyBboxPatch((6.0, 2.2), 4.6, 5.0,
                                       boxstyle="round,pad=0.2,rounding_size=0.3",
                                       linewidth=2, edgecolor='#059669', facecolor='#ECFDF5')
    ax.add_patch(inner_box)
    ax.text(8.3, 6.9, "2. INNER RATE PID LOOP", fontsize=13, fontweight='bold', color='#047857', ha='center')
    ax.text(8.3, 6.55, "Runs at 250 Hz (5x Faster!)  •  Goal: Match Spin Speed", fontsize=9.5, color='#059669', ha='center')

    # Step 3: Compare Target Speed & Gyro Speed
    sub3 = patches.FancyBboxPatch((6.2, 4.7), 4.2, 1.5,
                                  boxstyle="round,pad=0.1,rounding_size=0.15",
                                  linewidth=1.2, edgecolor='#34D399', facecolor='#FFFFFF')
    ax.add_patch(sub3)
    ax.text(8.3, 5.85, "Step 2A: Are we spinning at that speed?", fontsize=10.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(8.3, 5.45, "Compare Target Speed vs Actual Gyro Speed", fontsize=9.5, color='#334155', ha='center')
    ax.text(8.3, 5.0, "Speed Error:  e_w = w_sp - w_gyro", fontsize=10, fontweight='bold', color='#047857', ha='center')

    # Arrow inside inner loop
    ax.annotate('', xy=(8.3, 4.3), xytext=(8.3, 4.7),
                arrowprops=dict(facecolor='#059669', edgecolor='#059669', width=2, headwidth=7, headlength=7))

    # Step 4: PID Motor Push
    sub4 = patches.FancyBboxPatch((6.2, 2.6), 4.2, 1.6,
                                  boxstyle="round,pad=0.1,rounding_size=0.15",
                                  linewidth=1.2, edgecolor='#34D399', facecolor='#FFFFFF')
    ax.add_patch(sub4)
    ax.text(8.3, 3.85, "Step 2B: Push the motors with PID!", fontsize=10.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(8.3, 3.45, "P (Push now) + I (Fix steady error) + D (Damp)", fontsize=9.5, color='#334155', ha='center')
    ax.text(8.3, 2.95, "Command:  u = Kp*e + Ki*Int(e) - Kd*dw/dt", fontsize=10, fontweight='bold', color='#065F46', ha='center')

    # ==================== CONNECTOR TO DRONE ====================
    ax.annotate('', xy=(11.3, 3.4), xytext=(10.6, 3.4),
                arrowprops=dict(facecolor='#059669', edgecolor='#059669', width=3, headwidth=9, headlength=9))
    ax.text(10.95, 3.8, "Motor Signals\n[u_x, u_y, u_z]", fontsize=9, fontweight='bold', color='#047857', ha='center', va='bottom')

    # ==================== DRONE PLANT BOX ====================
    plant_box = patches.FancyBboxPatch((11.4, 2.2), 2.1, 5.0,
                                        boxstyle="round,pad=0.2,rounding_size=0.3",
                                        linewidth=2, edgecolor='#D97706', facecolor='#FFFBEB')
    ax.add_patch(plant_box)
    ax.text(12.45, 6.9, "3. DRONE", fontsize=13, fontweight='bold', color='#B45309', ha='center')
    ax.text(12.45, 6.55, "Physics & Motors", fontsize=9.5, color='#D97706', ha='center')

    sub_drone = patches.FancyBboxPatch((11.6, 3.1), 1.7, 3.1,
                                       boxstyle="round,pad=0.1,rounding_size=0.15",
                                       linewidth=1.2, edgecolor='#FCD34D', facecolor='#FFFFFF')
    ax.add_patch(sub_drone)
    ax.text(12.45, 5.8, "Motor Spindown /\nSpeedup", fontsize=9.5, fontweight='bold', color='#0F172A', ha='center', va='center')
    ax.text(12.45, 4.7, "Generates\nPhysical\nTorques", fontsize=9.5, color='#475569', ha='center', va='center')
    ax.text(12.45, 3.6, "Roll, Pitch,\nYaw Motion", fontsize=9.5, fontweight='bold', color='#B45309', ha='center', va='center')

    # ==================== FEEDBACK LOOPS ====================
    # Fast Gyro Loop (Inner)
    ax.plot([12.45, 12.45, 8.3, 8.3], [2.2, 1.4, 1.4, 2.2], color='#059669', linewidth=2, linestyle='--')
    ax.annotate('', xy=(8.3, 2.2), xytext=(8.3, 1.7),
                arrowprops=dict(facecolor='#059669', edgecolor='#059669', width=1.5, headwidth=6, headlength=6))
    ax.text(10.375, 1.15, "Fast Gyro Rate Feedback (w_gyro at 250 Hz)", fontsize=9.5, fontweight='bold', color='#059669', ha='center')

    # Slower AHRS Loop (Outer)
    ax.plot([12.8, 12.8, 2.9, 2.9], [2.2, 0.45, 0.45, 2.2], color='#0284C7', linewidth=2, linestyle='-')
    ax.annotate('', xy=(2.9, 2.2), xytext=(2.9, 1.0),
                arrowprops=dict(facecolor='#0284C7', edgecolor='#0284C7', width=1.5, headwidth=6, headlength=6))
    ax.text(7.85, 0.2, "Attitude Sensor Fusion Feedback (q_meas from IMU + EKF at 50 Hz)", fontsize=9.5, fontweight='bold', color='#0284C7', ha='center')

    plt.tight_layout()
    plt.savefig('figures/fig_intuitive_loops.png', bbox_inches='tight', dpi=300)
    plt.close()
    print("Successfully generated figures/fig_intuitive_loops.png")

create_cascaded_loops_diagram()
