# Draft LinkedIn Post for Project First Review

**Copy & paste the post below into LinkedIn, tag your team members, and replace the GitHub URL placeholder!**

---

🚀 Excited to share our 5th-Semester Drone Technologies & Flight Control Project at **Amrita Vishwa Vidyapeetham**: **Quaternion Attitude Control System of Highly Maneuverable Aircraft & Drones!**

Conventional autopilot controllers rely on Euler angles (Roll, Pitch, Yaw), which suffer from gimbal lock singularities at extreme pitch angles ($\pm 90^\circ$) and catastrophic elevator-rudder cross-coupling during steep bank turns ($80^\circ - 90^\circ$ knife-edge flight).

To solve this, we implemented a non-singular **Quaternion Proportional ($Q\_P$) Attitude Controller** coupled with a 3-axis inner-loop rate PID controller in a 6-DOF dynamic SITL simulation.

### 🌟 Key Highlights & Results:
- ❌ **Zero Gimbal Lock / Singularities:** Operates seamlessly across full 3D rotational space $SO(3)$.
- 🔄 **Shortest-Path SLERP Trajectory Tracking:** Guarantees minimum rotation arc ($\le 180^\circ$).
- 📈 **$10\times$ Yaw Error Reduction in Knife-Edge Flight ($90^\circ$ Bank):** Reduced yaw tracking RMSE from $30.50^\circ$ (Euler) down to $3.07^\circ$ (Quaternion).
- 📐 **Step-by-step Numerical Verification:** Verified through an explicit numerical toy example and 6-DOF dynamic flight simulations.

Special thanks to our course faculty **Prof. Sunil Kumar** for guidance throughout this research and implementation.

🔗 **GitHub Repository:** `https://github.com/your-username/Drones_S5_CD13_Quaternion_Attitude_Control_UAV`

👥 **Team Members:**
- Raghuram S (@tag_raghuram)
- [Tag Team Member 2]
- [Tag Team Member 3]

#DroneTechnology #FlightControl #Avionics #Quaternions #ControlSystems #UAV #AmritaUniversity #AerospaceEngineering #Robotics #ArduPilot #SITL #Simulation
