# What changed from the ROS 1 version

The project started as a ROS 1 (catkin) description package exported from SolidWorks. It is preserved in the git history under the tag [`ros1-legacy`](https://github.com/basheeraltawil/aibomech_agrobot/tree/ros1-legacy). This page lists what the ROS 2 version changed.

| ROS 1 package | ROS 2 stack |
|---|---|
| Single URDF from the SolidWorks exporter, plus hand-edited copies | Parametric xacro. One file serves every platform (`rail_trolley`, `pedestal`, `table`) and every hardware back-end (`mock`, `gz`, `real`) |
| Placeholder masses (0.15 kg per link), `velocity="0"` limits, 150 N·m effort | CAD masses and inertias plus the servo in each link (parallel-axis theorem), actuator-rated velocity and effort limits, damping and friction, and soft limits (`safety_controller`) |
| Full meshes used as collision geometry | Collision boxes derived from the meshes. The L-shaped base and the gripper jaws are split into several boxes, so the gripper has a real opening |
| Gazebo Classic with `gazebo_ros_control` and a `joint_states_to_gazebo.py` relay | Gazebo Fortress with `gz_ros2_control`; the same controllers run in simulation and on the robot |
| `rosserial` Arduino sketch sending raw angles | `ros2_control` hardware plugin with a checksummed serial protocol, per-joint calibration, a communication watchdog, e-stop handling and a board emulator |
| Arm fixed to the world | The arm sits on a lift column on a heating-pipe rail trolley (a 7th, external linear axis) with a harvest crate, an electrical cabinet, an e-stop and a signal tower |
| No sensors | RGB-D camera (RealSense D435 class) on a mast, with the same topic names in simulation and on the robot |
| No application | Four agricultural scenarios with perception, IK, collision checking, planning, KPI reports and e-stop handling |

The arm's kinematics and meshes are unchanged, so everything here applies to the real prototype. Joints were renamed to industrial style:

| ROS 1 | ROS 2 |
|---|---|
| `revolute_1` | `joint_1` |
| `revolute_2` | `joint_2` |
| `revolute_3` | `joint_3` |
| `revolute_4` | `joint_4` |
| `prismatic_1` | `gripper_jaw_joint` |
