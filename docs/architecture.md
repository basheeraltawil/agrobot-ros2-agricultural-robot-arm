# How the system is built

This section explains how the parts fit together, from the physical robot up to the task logic. Read it before changing anything; every scenario and the real robot reuse the same layers.

## Layered architecture

The system has four layers. Each layer only talks to the layer directly below it through standard ROS 2 interfaces. That is why a task that works in Gazebo runs unchanged on the real robot.

```mermaid
flowchart TB
    subgraph L4["4 · Task layer — aibomech_agrobot_tasks (Python)"]
        direction LR
        SC["Scenario nodes<br/>harvest · inspection · transplant · weeding"]
        TB["task_base<br/>survey · rail planning · reports"]
        PE["perception<br/>RGB-D crop detection"]
        KI["kinematics<br/>FK · Jacobian · IK"]
        CO["collision<br/>URDF boxes · SAT"]
        PL["planner<br/>RRT-Connect"]
        RI["robot_interface<br/>actions · lines · e-stop"]
        SC --> TB --> RI
        TB --> PE
        RI --> KI
        RI --> CO
        RI --> PL
    end
    subgraph L3["3 · Control layer — ros2_control"]
        direction LR
        CM["controller_manager 200 Hz"]
        AC["arm_controller<br/>JointTrajectoryController"]
        RC["rail_controller<br/>JointTrajectoryController"]
        GC["gripper_controller<br/>GripperActionController"]
        JS["joint_state_broadcaster"]
        CM --- AC & RC & GC & JS
    end
    subgraph L2["2 · Hardware abstraction — one of three back-ends"]
        direction LR
        MO["mock<br/>GenericSystem"]
        GZ["gz<br/>gz_ros2_control"]
        RE["real<br/>AgrobotSystemHardware"]
    end
    subgraph L1["1 · Physical layer"]
        direction LR
        SIM["Gazebo Fortress world<br/>crops · fixtures · RGB-D camera"]
        MCU["Motor-controller board<br/>servos · rail stepper · e-stop"]
        CAM["RealSense D435"]
    end
    RI -- "FollowJointTrajectory ×2<br/>GripperCommand" --> CM
    PE -- "images, depth" --- CAM
    PE -- "images, depth" --- SIM
    CM --> MO & GZ & RE
    GZ --> SIM
    RE -- "USB serial 115200 Bd" --> MCU
```

| Layer | What it is responsible for | What you change here |
|---|---|---|
| 4. Tasks | What to do: find crops, choose where to put the rail, plan collision-free motions, grasp, report | New agricultural tasks, detection thresholds, crop layouts |
| 3. Control | Following joint trajectories smoothly and within tolerances, opening and closing the gripper | Gains, tolerances, update rate (`controllers.yaml`) |
| 2. Hardware abstraction | Turning joint commands into actuator commands and reading joint states | New actuators or drives (a new `SystemInterface` plugin) |
| 1. Physical | The robot, the crops and the camera, real or simulated | Mechanics, wiring, world models |

## ROS 2 graph: nodes, topics and actions

The graph below is what runs during `scenario.launch.py`. In simulation, the camera topics come from `ros_gz_bridge`. On the real robot, the same topic names come from `realsense2_camera`.

```mermaid
flowchart LR
    subgraph gazebo["Gazebo Fortress"]
        W["world + robot model"]
        GZC["gz_ros2_control<br/>+ controller_manager"]
    end
    BR["ros_gz_bridge"]
    RSP["robot_state_publisher"]
    SM["sim_manager.py<br/>(one-shot)"]
    T["task node<br/>e.g. strawberry_harvest"]
    RV["RViz"]

    W -- "rgbd camera" --> BR
    BR -- "/camera/color/image_raw<br/>/camera/aligned_depth_to_color/image_raw<br/>/camera/color/camera_info" --> T
    BR -- "/clock" --> T
    BR -- "/agrobot/sim/model_poses" --> T
    T -- "/agrobot/sim/#lt;obj#gt;/attach_gripper<br/>…/detach_gripper · …/detach_anchor" --> BR
    SM -- "detach all grasp joints<br/>then unpause" --> BR
    BR --> W
    GZC -- "/joint_states" --> RSP & T
    RSP -- "/tf · /robot_description" --> T & RV
    T -- "/arm_controller/follow_joint_trajectory<br/>/rail_controller/follow_joint_trajectory<br/>/gripper_controller/gripper_cmd" --> GZC
    T -- "/agrobot/detections/image<br/>/agrobot/detections/markers" --> RV
    ES["estop CLI"] -- "/agrobot/estop" --> T
```

| Interface | Type | Direction | Purpose |
|---|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` (action) | task → controller | Arm motion (joints 1–4) |
| `/rail_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` (action) | task → controller | Trolley position along the row |
| `/gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` (action) | task → controller | Open or close the jaw |
| `/joint_states` | `sensor_msgs/JointState` | controller → all | Measured joint positions and velocities |
| `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | camera → task | RGB-D input for crop detection |
| `/agrobot/detections/image`, `/agrobot/detections/markers` | `Image`, `MarkerArray` | task → RViz | What the robot detected, for debugging |
| `/agrobot/estop` | `std_msgs/Bool` (latched) | operator → task | Software emergency stop |
| `/agrobot/sim/model_poses` | `tf2_msgs/TFMessage` | Gazebo → task | Ground truth, only used to score a simulated run |
| `/agrobot/sim/<obj>/attach_gripper`, `…/detach_gripper`, `…/detach_anchor` | `std_msgs/Empty` | task → Gazebo | Simulated grasping (no-ops on the real robot) |

## Frames (TF tree)

All positions in the task layer are expressed in `world` (crop map) or `arm_mount` (motion planning). The rail joint moves everything below `trolley`, so a crop's position in `arm_mount` changes when the trolley moves.

```mermaid
flowchart TD
    world --> rail
    rail -- "rail_joint (prismatic)" --> trolley
    trolley --> lift_column
    lift_column --> arm_mount
    lift_column --> crate
    arm_mount --> base_link
    arm_mount --> camera_mast --> camera_link --> camera_color_optical_frame
    base_link --> base
    base_link -- joint_1 --> link_1
    link_1 -- joint_2 --> link_2
    link_2 -- joint_3 --> link_3
    link_3 -- joint_4 --> link_4
    link_4 -- gripper_jaw_joint --> link_5
    link_4 --> flange --> tool0
    link_4 --> tcp
    link_4 --> tcp_center
```

## Simulation start-up

Grasping in Gazebo relies on `DetachableJoint`s, which Gazebo creates *attached*. The launch sequence therefore loads the world paused, releases all grasp joints, and only then starts physics and the controllers.

```mermaid
sequenceDiagram
    autonumber
    participant L as scenario.launch.py
    participant G as Gazebo (paused)
    participant C as ros_gz_sim create
    participant M as sim_manager.py
    participant S as controller spawners
    participant T as task node
    L->>G: load world (crops, fixtures, grasp joints)
    L->>C: spawn robot from /robot_description
    C-->>L: exit (robot exists)
    L->>M: start
    M->>G: detach_gripper for every crop
    M->>G: unpause (WorldControl)
    M-->>L: exit
    L->>S: joint_state_broadcaster, arm, gripper, rail controllers
    T->>T: wait for URDF, joint states, action servers, camera
    T->>G: run the scenario
```

## Inside the task layer

| Module | Responsibility | Key ideas |
|---|---|---|
| `perception.py` | Find crops in the RGB-D image and turn them into 3D points | HSV thresholds per class, blob size window, median depth, shift by object radius, transform with TF into `world`, crop region filter |
| `kinematics.py` | Forward and inverse kinematics from the URDF | Geometric Jacobian; IK solves position first, then uses the one redundant DOF to align the approach direction (null-space); multi-start; `track`/`refine` for local solutions |
| `collision.py` | Is a joint configuration collision-free? | The URDF's own collision boxes, separating-axis test, arm vs. carrier, arm vs. itself, arm and held crop vs. obstacle boxes (gutter, bench, tray, crops). Scene obstacles need 3 mm clearance, because touching them stalls the arm |
| `planner.py` | Collision-free joint path between two configurations | RRT-Connect with random shortcutting; direct path if already free |
| `robot_interface.py` | Execute motions safely | Planned joint moves (quintic segments), straight TCP lines (interpolated between known start and end configurations, so the wrist never flips; step-by-step tracking as fallback), rail and gripper actions, e-stop cancelling, simulated grasp topics, retrying goals while controllers start |
| `task_base.py` | Shared scenario logic | Rail as external axis (`plan_reach`), compliant approach (a stop on contact within 12 mm of the goal still grips, because crops are soft and never exactly where the camera saw them), crop obstacles, recovery, sim ground truth, reports |

### Perception pipeline

```mermaid
flowchart LR
    A["Colour image"] --> B["HSV conversion"]
    B --> C["Threshold per class<br/>(ripe, unripe, weed, crop …)"]
    C --> D["Morphological opening"]
    D --> E["Connected components<br/>area filter"]
    F["Aligned depth image"] --> G["Median depth<br/>inside blob"]
    E --> G
    G --> H["Deproject centroid<br/>(camera intrinsics)"]
    H --> I["+ object radius<br/>along viewing ray"]
    I --> J["TF → world frame"]
    J --> K{"inside<br/>detection_region?"}
    K -- yes --> L["Detection<br/>label · 3D centre · size"]
    K -- no --> X["discarded<br/>(e.g. fruit in the crate)"]
    L --> M["Fuse stations<br/>merge_detections"]
```

### Motion planning pipeline

For every crop the task layer answers: *where must the trolley stand, and how does the arm get there without hitting anything?*

```mermaid
flowchart TD
    S["Crop position (world)"] --> A["For each approach direction<br/>(horizontal, then tilted ±20°)"]
    A --> B["For each rail offset<br/>(0.16, 0.14, 0.18 … m)"]
    B --> C["IK for the grasp pose<br/>position exact, approach aligned,<br/>collision-free"]
    C -- fails --> B
    C --> D["Pre-grasp pose on the same arm branch"]
    D -- fails --> B
    D --> E["Straight approach line free?"]
    E -- no --> B
    E --> F["Straight retreat line free?"]
    F -- no --> B
    F --> G["RRT-Connect path from home<br/>to pre-grasp exists?"]
    G -- no --> B
    G --> H["Plan found:<br/>rail position + joint configurations"]
    B -- "all offsets tried" --> A
    A -- "all directions tried" --> U["Report 'unreachable'<br/>and continue with the next crop"]
```

## One harvest cycle, end to end

```mermaid
sequenceDiagram
    autonumber
    participant T as strawberry_harvest
    participant P as perception
    participant K as IK + collision + planner
    participant R as rail_controller
    participant A as arm_controller
    participant G as gripper_controller
    participant Z as Gazebo / real crop
    T->>K: plan_reach(fruit, approaches, rail offsets)
    K-->>T: rail x, pre-grasp q, grasp q
    T->>R: move trolley to x
    T->>P: look again at this station
    P-->>T: refined fruit position
    T->>A: planned joint path to pre-grasp
    T->>A: straight line to the fruit
    T->>G: close on 19 mm
    T->>Z: attach to gripper, detach from plant (sim only)
    T->>A: straight retreat (pulls the peduncle)
    T->>A: planned path over the crate
    T->>G: open
    T->>Z: release (sim only)
    T->>A: planned path home
```

## Real-robot data path

On the real robot the same controllers talk to `AgrobotSystemHardware`, which exchanges short ASCII frames with the motor-controller board.

```mermaid
sequenceDiagram
    participant CM as controller_manager (200 Hz)
    participant HW as AgrobotSystemHardware
    participant B as Motor-controller board
    participant M as Servos + rail stepper
    Note over HW,B: on_configure: open port, ping, wait for a state frame<br/>on_activate: refuse if e-stop pressed, start from measured position, $E,1
    loop every control cycle
        CM->>HW: write(): joint position commands
        HW->>B: $C,p0,…,p5*CS (throttled to 50 Hz)
        B->>M: clamped, slew-limited setpoints
        B-->>HW: $S,seq,status,positions,velocities*CS (50 Hz)
        HW-->>CM: read(): joint positions and velocities
    end
    Note over B: no command for 250 ms → hold position (watchdog)<br/>e-stop contact open → drives off, latched until $E,1
```

## Safety layers

```mermaid
flowchart LR
    E1["Hardware e-stop<br/>cuts actuator power"] --> E2["Firmware<br/>latch · watchdog · clamping"]
    E2 --> E3["Hardware plugin<br/>no activation on e-stop · no jumps · comm loss = error"]
    E3 --> E4["Controllers<br/>path and goal tolerances"]
    E4 --> E5["Task layer<br/>collision checks · software e-stop · per-crop recovery"]
```

1. **Motor-controller board.** A hardware e-stop input (normally-closed contact) cuts servo PWM and disables the stepper. It stays latched until the host re-enables it. A 250 ms command watchdog holds position if commands stop, and setpoints are clamped and slew-limited in firmware.
2. **Hardware plugin.** It refuses to activate while the e-stop is pressed, starts from the measured position (no jump), and reports a communication error after 10 missed frames.
3. **Controllers.** Path and goal tolerances abort a trajectory if the arm is blocked.
4. **Task layer.**
   - A software e-stop on `/agrobot/estop` (`ros2 run aibomech_agrobot_tasks estop on|off`) cancels motion.
   - Every joint move is collision-checked and planned, and linear moves are checked point by point.
   - A failed crop is reported and the robot recovers to home.
