import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 11

def create_error_resolution_diagram():
    fig, ax = plt.subplots(figsize=(14, 7.5), dpi=300)
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8.5)
    ax.axis('off')

    fig.patch.set_facecolor('#F8FAFC')
    ax.set_facecolor('#F8FAFC')

    # Title
    ax.text(7.0, 8.1, "WHAT ATTITUDE ERROR ACTUALLY MEANS IN 3D SPACE", 
            fontsize=15, fontweight='bold', ha='center', va='center', color='#0F172A')
    ax.text(7.0, 7.7, "The Quaternion Error (q_err = q_meas* (x) q_sp) finds the SINGLE direct 3D rotation between two orientations", 
            fontsize=11, ha='center', va='center', color='#475569', style='italic')

    # ==================== LEFT PANEL: CURRENT ATTITUDE ====================
    left_box = patches.FancyBboxPatch((0.6, 2.0), 3.8, 5.2,
                                      boxstyle="round,pad=0.15,rounding_size=0.25",
                                      linewidth=2, edgecolor='#0284C7', facecolor='#F0F9FF')
    ax.add_patch(left_box)
    ax.text(2.5, 6.8, "CURRENT ATTITUDE (q_meas)", fontsize=12, fontweight='bold', color='#0369A1', ha='center')
    ax.text(2.5, 6.45, "90 deg Knife-Edge Bank (On its side)", fontsize=9.5, color='#0284C7', ha='center')

    # Aircraft schematic banked 90 deg
    # Vertical fuselage line
    ax.plot([2.5, 2.5], [3.8, 5.6], color='#0369A1', linewidth=4, solid_capstyle='round')
    # Horizontal wings (now vertical because banked 90 deg!)
    ax.plot([1.7, 3.3], [4.7, 4.7], color='#0284C7', linewidth=6, solid_capstyle='round')
    # Tailplane
    ax.plot([2.0, 3.0], [4.0, 4.0], color='#38BDF8', linewidth=4, solid_capstyle='round')
    # Bank angle indicator
    arc_bank = patches.Arc((2.5, 4.7), 2.2, 2.2, angle=0, theta1=0, theta2=90, color='#0284C7', linewidth=2.5, linestyle=':')
    ax.add_patch(arc_bank)
    ax.text(2.5, 3.4, "Roll (phi) = +90 deg\nPitch (theta) = 0 deg\nYaw (psi) = 0 deg", 
            fontsize=10, fontweight='bold', color='#0F172A', ha='center', va='top')
    ax.text(2.5, 2.4, "q_meas = [0.7071, 0.7071, 0, 0]^T", 
            fontsize=9.5, fontfamily='monospace', color='#0369A1', ha='center')

    # ==================== CENTER PANEL: QUATERNION ROTATION ====================
    center_box = patches.FancyBboxPatch((4.8, 2.0), 4.4, 5.2,
                                        boxstyle="round,pad=0.15,rounding_size=0.25",
                                        linewidth=2, edgecolor='#8B5CF6', facecolor='#F5F3FF')
    ax.add_patch(center_box)
    ax.text(7.0, 6.8, "3D ERROR RESOLUTION (q_err)", fontsize=12, fontweight='bold', color='#6D28D9', ha='center')
    ax.text(7.0, 6.45, "Hamilton Product: q_err = q_meas* (x) q_sp", fontsize=9.5, color='#8B5CF6', ha='center')

    # Central Sphere with Axis of Rotation
    sphere = patches.Circle((7.0, 4.6), 1.2, edgecolor='#8B5CF6', facecolor='#EDE9FE', linewidth=2)
    ax.add_patch(sphere)
    # Rotation Axis Vector n
    ax.annotate('', xy=(8.0, 5.5), xytext=(6.0, 3.7),
                arrowprops=dict(facecolor='#6D28D9', edgecolor='#6D28D9', width=3, headwidth=9, headlength=9))
    ax.text(8.1, 5.55, "Axis n_hat\n[-0.94, +0.25, -0.25]", fontsize=9, fontweight='bold', color='#6D28D9', va='bottom')

    # Curved rotation arc around axis
    arc_rot = patches.Arc((7.0, 4.6), 1.7, 1.0, angle=45, theta1=20, theta2=200, color='#10B981', linewidth=3.5)
    ax.add_patch(arc_rot)
    ax.annotate('', xy=(7.2, 5.3), xytext=(7.1, 5.1),
                arrowprops=dict(facecolor='#10B981', edgecolor='#10B981', width=2, headwidth=7, headlength=7))
    ax.text(7.0, 4.5, "theta = 93.8 deg\nSingle 3D Turn", fontsize=10, fontweight='bold', color='#047857', ha='center', va='center')

    ax.text(7.0, 3.1, "Euler-Axis:  n_hat = [-0.935, +0.251, -0.251]^T\nTurn Angle: theta_err = 93.82 deg", 
            fontsize=9.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(7.0, 2.4, "q_err = [0.6830, -0.6830, +0.1830, -0.1830]^T", 
            fontsize=9.5, fontfamily='monospace', color='#6D28D9', ha='center')

    # Connectors between boxes
    ax.annotate('', xy=(4.7, 4.6), xytext=(4.45, 4.6),
                arrowprops=dict(facecolor='#8B5CF6', edgecolor='#8B5CF6', width=2, headwidth=6, headlength=6))
    ax.annotate('', xy=(9.6, 4.6), xytext=(9.25, 4.6),
                arrowprops=dict(facecolor='#8B5CF6', edgecolor='#8B5CF6', width=2, headwidth=6, headlength=6))

    # ==================== RIGHT PANEL: TARGET ATTITUDE ====================
    right_box = patches.FancyBboxPatch((9.6, 2.0), 3.8, 5.2,
                                       boxstyle="round,pad=0.15,rounding_size=0.25",
                                       linewidth=2, edgecolor='#D97706', facecolor='#FFFBEB')
    ax.add_patch(right_box)
    ax.text(11.5, 6.8, "TARGET SETPOINT (q_sp)", fontsize=12, fontweight='bold', color='#B45309', ha='center')
    ax.text(11.5, 6.45, "30 deg Pitch-Up (Level Wings)", fontsize=9.5, color='#D97706', ha='center')

    # Aircraft schematic pitched up 30 deg (side/isometric view)
    # Pitched fuselage line
    ax.plot([10.5, 12.3], [4.2, 5.3], color='#B45309', linewidth=4, solid_capstyle='round')
    # Horizontal wings
    ax.plot([11.2, 11.6], [5.2, 4.3], color='#D97706', linewidth=6, solid_capstyle='round')
    # Pitch angle arc
    arc_pitch = patches.Arc((10.5, 4.2), 2.0, 2.0, angle=0, theta1=0, theta2=31, color='#D97706', linewidth=2.5, linestyle=':')
    ax.add_patch(arc_pitch)
    ax.text(11.5, 3.4, "Roll (phi) = 0 deg\nPitch (theta) = +30 deg\nYaw (psi) = 0 deg", 
            fontsize=10, fontweight='bold', color='#0F172A', ha='center', va='top')
    ax.text(11.5, 2.4, "q_sp = [0.9659, 0, +0.2588, 0]^T", 
            fontsize=9.5, fontfamily='monospace', color='#B45309', ha='center')

    # ==================== BOTTOM TAKEAWAY COMPARISON ====================
    compare_box = patches.FancyBboxPatch((0.6, 0.4), 12.8, 1.3,
                                         boxstyle="round,pad=0.15,rounding_size=0.2",
                                         linewidth=1.8, edgecolor='#059669', facecolor='#ECFDF5')
    ax.add_patch(compare_box)
    ax.text(1.0, 1.25, "KEY TAKEAWAY (WHY QUATERNIONS PREVENT ACCIDENTS):", 
            fontsize=10.5, fontweight='bold', color='#047857')
    ax.text(1.0, 0.9, "• Euler Autopilot: Sees 30 deg pitch error -> Blindly deflects elevator -> BUT at 90 deg bank, elevator yaws the drone -> CRASH!", 
            fontsize=9.5, color='#991B1B', fontweight='bold')
    ax.text(1.0, 0.6, "• Quaternion Autopilot: Computes single 3D vector [qx, qy, qz] -> Automatically uses Rudder for pitch and Elevator for yaw -> SMOOTH 93.8 deg TURN!", 
            fontsize=9.5, color='#047857', fontweight='bold')

    plt.tight_layout()
    plt.savefig('figures/fig_intuitive_error_resolution.png', bbox_inches='tight', dpi=300)
    plt.close()
    print("Successfully generated figures/fig_intuitive_error_resolution.png")

create_error_resolution_diagram()
