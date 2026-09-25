# aibomech_agrobot_gazebo

Gazebo Fortress worlds for the agricultural scenarios, and the launch file that spawns the robot into them with `gz_ros2_control`.

| World | Used by |
|---|---|
| `strawberry_greenhouse` | `strawberry_harvest`, `plant_inspection` |
| `nursery_transplanting` | `seedling_transplant` |
| `weeding_bed` | `precision_weeding` |

```bash
ros2 launch aibomech_agrobot_gazebo sim.launch.py world:=weeding_bed     # gui:=false for headless
```

| File | Content |
|---|---|
| `tools/generate_worlds.py` | generates every world, its ground-truth object list and its ROS–Gazebo bridge configuration; edit crop layouts here, not in the `.sdf` files |
| `worlds/*.sdf` | generated worlds |
| `config/*_objects.yaml` | ground truth of every crop, used to score the tasks |
| `config/*_bridge.yaml` | `ros_gz_bridge` topics: camera, clock, grasp joints |
| `scripts/sim_manager.py` | releases the grasp joints and starts physics after the robot is spawned |

**How grasping is simulated.** Every graspable object has a `DetachableJoint` to the gripper and one to its plant or soil. The task attaches and detaches them over ROS topics, so a fruit stays on the plant until it is cut and stays in the gripper while it is carried. See [docs/scenarios.md](../../../docs/scenarios.md#how-grasping-is-simulated).
