import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 11

def create_shortest_path_diagram():
    fig, ax = plt.subplots(figsize=(13, 7.5), dpi=300)
    ax.set_xlim(-6.5, 6.5)
    ax.set_ylim(-4.2, 4.2)
    ax.axis('off')

    fig.patch.set_facecolor('#F8FAFC')
    ax.set_facecolor('#F8FAFC')

    # Title
    ax.text(0, 3.8, "WHY WE DO THE SHORTEST-PATH ARC CHECK (q vs -q)", 
            fontsize=15, fontweight='bold', ha='center', va='center', color='#0F172A')
    ax.text(0, 3.4, "Quaternions have 2 mathematical representations for every 1 physical attitude:  q  and  -q", 
            fontsize=11, ha='center', va='center', color='#475569', style='italic')

    # Central Sphere / Circle representation
    circle = patches.Circle((0, 0.2), 2.2, edgecolor='#94A3B8', facecolor='#FFFFFF', linewidth=2.5, linestyle='-')
    ax.add_patch(circle)
    circle_inner = patches.Circle((0, 0.2), 2.2, edgecolor='#E2E8F0', facecolor='#F1F5F9', alpha=0.5)
    ax.add_patch(circle_inner)

    # Center origin
    ax.plot(0, 0.2, 'o', color='#64748B', markersize=6)
    ax.text(0, -0.1, "Rotation Center\nAxis n_hat", fontsize=8.5, color='#64748B', ha='center', va='top')

    # Start Point A (Current Attitude: Knife-Edge 90 deg)
    # Put Start at 12 o'clock (90 deg angle on circle)
    start_x, start_y = 0.0, 2.4
    ax.plot(start_x, start_y, 'o', color='#0284C7', markersize=12, zorder=5)
    ax.text(start_x, start_y + 0.35, "START ATTITUDE (q_meas)\nCurrent 90 deg Knife-Edge Bank", 
            fontsize=10.5, fontweight='bold', color='#0369A1', ha='center', va='bottom')

    # Target Point B (Target Attitude: 30 deg Pitch)
    # Error angle is approx 94 deg clockwise (at approx 4 o'clock)
    target_angle_rad = np.radians(-4) # around 4 o'clock
    target_x = 2.2 * np.cos(target_angle_rad)
    target_y = 0.2 + 2.2 * np.sin(target_angle_rad)
    ax.plot(target_x, target_y, 'o', color='#D97706', markersize=12, zorder=5)
    ax.text(target_x + 0.3, target_y, "TARGET ATTITUDE (q_sp)\nCommanded 30 deg Pitch", 
            fontsize=10.5, fontweight='bold', color='#B45309', ha='left', va='center')

    # 1. GREEN SHORT PATH (Direct 93.8 deg arc)
    # Arc from 90 deg down to -4 deg (clockwise)
    green_arc = patches.Arc((0, 0.2), 4.7, 4.7, angle=0, theta1=-4, theta2=90, 
                            color='#10B981', linewidth=5, zorder=4)
    ax.add_patch(green_arc)
    # Green arrow tip
    ax.annotate('', xy=(target_x + 0.15, target_y + 0.1), xytext=(target_x + 0.12, target_y + 0.25),
                arrowprops=dict(facecolor='#10B981', edgecolor='#10B981', width=3, headwidth=10, headlength=10))

    # Green Callout Box (Right)
    green_box = patches.FancyBboxPatch((2.6, 1.2), 3.6, 1.8,
                                       boxstyle="round,pad=0.15,rounding_size=0.2",
                                       linewidth=2, edgecolor='#10B981', facecolor='#ECFDF5')
    ax.add_patch(green_box)
    ax.text(4.4, 2.65, "[YES] SHORTEST ARC (q_err)", fontsize=11, fontweight='bold', color='#047857', ha='center')
    ax.text(4.4, 2.25, "Rotation Angle = 93.8 deg  (<= 180 deg)", fontsize=9.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(4.4, 1.85, "Condition:  q_w,err = +0.6830  (>= 0)", fontsize=9.5, color='#065F46', ha='center')
    ax.text(4.4, 1.45, "Action: Take direct turn! Fast & smooth.", fontsize=9, color='#047857', ha='center', style='italic')

    # 2. RED LONG PATH (Wasteful 266.2 deg arc)
    # Arc from 90 deg counter-clockwise all the way around to -4 deg (356 deg)
    red_arc = patches.Arc((0, 0.2), 4.1, 4.1, angle=0, theta1=90, theta2=356, 
                          color='#EF4444', linewidth=3.5, linestyle='--', zorder=4)
    ax.add_patch(red_arc)
    # Red arrow tip
    ax.annotate('', xy=(target_x - 0.05, target_y - 0.2), xytext=(target_x - 0.2, target_y - 0.15),
                arrowprops=dict(facecolor='#EF4444', edgecolor='#EF4444', width=2, headwidth=8, headlength=8))

    # Red Callout Box (Left)
    red_box = patches.FancyBboxPatch((-6.2, 1.2), 3.6, 1.8,
                                     boxstyle="round,pad=0.15,rounding_size=0.2",
                                     linewidth=2, edgecolor='#EF4444', facecolor='#FEF2F2')
    ax.add_patch(red_box)
    ax.text(-4.4, 2.65, "[NO] WASTEFUL ARC (-q_err)", fontsize=11, fontweight='bold', color='#B91C1C', ha='center')
    ax.text(-4.4, 2.25, "Rotation Angle = 266.2 deg  (> 180 deg)", fontsize=9.5, fontweight='bold', color='#0F172A', ha='center')
    ax.text(-4.4, 1.85, "Condition:  q_w,err < 0  (Negative scalar)", fontsize=9.5, color='#991B1B', ha='center')
    ax.text(-4.4, 1.45, "Problem: Pointless 270 deg barrel roll!", fontsize=9, color='#B91C1C', ha='center', style='italic')

    # ==================== BOTTOM GOLDEN RULE BOX ====================
    rule_box = patches.FancyBboxPatch((-5.5, -3.8), 11.0, 1.5,
                                      boxstyle="round,pad=0.15,rounding_size=0.25",
                                      linewidth=2, edgecolor='#6366F1', facecolor='#EEF2FF')
    ax.add_patch(rule_box)
    ax.text(0, -2.6, "THE GOLDEN SHORTEST-PATH RULE IN AUTOPILOT CODE:", 
            fontsize=11.5, fontweight='bold', color='#4338CA', ha='center')
    ax.text(0, -3.1, "if  q_w,err < 0:      q_err = -q_err    # Invert signs to force the <= 180 deg green path!", 
            fontsize=11, fontfamily='monospace', fontweight='bold', color='#1E1B4B', ha='center')
    ax.text(0, -3.55, "This 1-line check guarantees the drone ALWAYS takes the shortest, safest route to its target orientation.", 
            fontsize=9.5, color='#475569', ha='center', style='italic')

    plt.tight_layout()
    plt.savefig('figures/fig_intuitive_shortest_path.png', bbox_inches='tight', dpi=300)
    plt.close()
    print("Successfully generated figures/fig_intuitive_shortest_path.png")

create_shortest_path_diagram()
