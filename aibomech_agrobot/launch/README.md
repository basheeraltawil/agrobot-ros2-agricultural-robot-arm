rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["joint1", "joint2", "joint3"], points: [{positions: [0.1, 0.57, 0.57], time_from_start: [1.0, 0.0]}]}' -1
