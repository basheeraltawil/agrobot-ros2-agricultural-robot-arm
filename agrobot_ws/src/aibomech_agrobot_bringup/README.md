# aibomech_agrobot_bringup

Starts the robot without a simulator: `robot_state_publisher`, `ros2_control` with the arm, rail and gripper controllers, and RViz.

```bash
ros2 launch aibomech_agrobot_bringup robot.launch.py                      # mock hardware (no motors)
ros2 launch aibomech_agrobot_bringup robot.launch.py hardware:=real serial_port:=/dev/ttyACM0
```

| File | Content |
|---|---|
| `config/controllers.yaml` | `joint_state_broadcaster`, `arm_controller` and `rail_controller` (JointTrajectoryController), `gripper_controller` (GripperActionController) |
| `launch/robot.launch.py` | arguments: `hardware`, `platform`, `mount_height`, `serial_port`, `baud_rate`, `calibration_file`, `realsense`, `rviz` |
| `rviz/agrobot.rviz` | RViz layout |

The same controllers run in Gazebo (`aibomech_agrobot_gazebo`), so a task behaves the same in simulation and on the real robot. Commissioning steps: [docs/real_robot.md](../../../docs/real_robot.md).
