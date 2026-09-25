# aibomech_agrobot_tasks

The intelligence of the robot: perception, inverse kinematics, collision checking, motion planning and the four agricultural scenarios. It is plain Python on top of the standard `ros2_control` action servers, so it runs unchanged in simulation and on the real robot.

```bash
ros2 launch aibomech_agrobot_tasks scenario.launch.py scenario:=strawberry_harvest   # hardware:=sim|real|mock
ros2 run aibomech_agrobot_tasks estop on                                             # software e-stop (off to reset)
```

## Code map

Read the modules in this order, from the robot upwards:

| Module | What it does | Main idea |
|---|---|---|
| `kinematics.py` | forward kinematics from the URDF, IK solver | position first, then the approach direction in the null space (damped least squares) |
| `collision.py` | box model of the arm, the held crop and the cell | separating-axis test between oriented boxes |
| `planner.py` | joint-space path planner | RRT-Connect with path shortcutting |
| `perception.py` | crop detection | HSV colour thresholds + median depth → 3D point |
| `robot_interface.py` | blocking motion API | joint moves, straight-line tool moves, rail, gripper, simulated grasp |
| `task_base.py` | shared scenario logic | choose a rail position and grasp configuration, recovery, reports |
| `scenarios/*.py` | the four tasks | survey → plan → pick → place → report |

`config/<scenario>.yaml` holds every tunable number of a scenario (speeds, approach directions, colour ranges, crate position, etc.).

## Tests

`test/test_kinematics.py` checks the Jacobian against finite differences, IK on the task poses, collision detection and the planner; `test/test_configs.py` checks that every scenario configuration is a valid ROS parameter file. Run them with `colcon test --packages-select aibomech_agrobot_tasks`.

Further reading: [docs/architecture.md](../../../docs/architecture.md) (pipelines and diagrams), [docs/scenarios.md](../../../docs/scenarios.md), [docs/development.md](../../../docs/development.md) (adding your own task).
