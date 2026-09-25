"""Task layer of the AgroBot: perception, planning and the agricultural scenarios.

Modules, from low to high level:
  kinematics.py       forward kinematics from the URDF and the inverse-kinematics solver
  collision.py        box collision model of the arm and the cell (separating-axis test)
  planner.py          RRT-Connect joint-space planner
  perception.py       RGB-D crop detection (colour thresholds + depth -> 3D points)
  robot_interface.py  blocking motion API on top of the ros2_control action servers
  task_base.py        shared scenario logic: survey, reach planning, grasp, recovery, reports
  estop.py            software emergency-stop node
  scenarios/          the four agricultural tasks
"""
