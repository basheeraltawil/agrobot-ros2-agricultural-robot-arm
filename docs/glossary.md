# Glossary

Terms used throughout the documentation and the code.

| Term | Meaning here |
|---|---|
| **TCP** (tool centre point) | The point between the gripper jaws where a crop's centre should be when it is gripped; frame `tcp`, z axis = approach direction |
| **Approach direction** | The direction the gripper moves in to reach a crop: horizontal into the row for strawberries, straight down for seedlings and weeds |
| **Pre-grasp / grasp / retreat** | The pose a few centimetres before the crop, the pose at the crop, and the pose the gripper pulls back to |
| **IK** (inverse kinematics) | Computing joint angles for a desired TCP position and approach direction |
| **Rail as external axis** | The trolley position is chosen per crop, like the 7th axis of an industrial cell, to put the crop where the arm can reach it well |
| **RRT-Connect** | A sampling-based planner that grows two trees of collision-free joint configurations until they meet |
| **ros2_control** | The ROS 2 framework that connects controllers (trajectory, gripper) to hardware plugins (mock, Gazebo, real) |
| **JTC** | `JointTrajectoryController`, follows time-stamped joint waypoints within tolerances |
| **xacro** | XML macros that generate the URDF robot description with parameters |
| **DetachableJoint** | A Gazebo plugin that creates and removes a fixed joint between two models; used to simulate gripping and cutting |
| **HSV** | Hue-saturation-value colour space; hue separates red fruit, green leaves and purple weeds well |
| **Survey** | Driving along the row and detecting all crops before acting on them |
| **KPI report** | `summary.json` and `results.csv` written after every run: detections, successes, cycle times |
