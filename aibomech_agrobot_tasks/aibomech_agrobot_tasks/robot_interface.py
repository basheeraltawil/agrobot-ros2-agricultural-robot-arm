"""Motion interface to the AgroBot, identical for simulation and the real robot.

Talks only to the standard ros2_control action servers:
  /arm_controller/follow_joint_trajectory    control_msgs/FollowJointTrajectory
  /rail_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory
  /gripper_controller/gripper_cmd            control_msgs/GripperCommand

In simulation it also drives the Gazebo grasp joints (see aibomech_agrobot_gazebo).
"""
import threading
import time

import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .kinematics import Chain, IkSolver

ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


class EmergencyStop(RuntimeError):
    pass


class MotionError(RuntimeError):
    pass


def to_duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def transform_to_matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


class RobotInterface:
    """Blocking motion API. The node must be spun by an executor in another thread."""

    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        p = node.declare_parameter
        self.speed_scale = p('speed_scale', 0.5).value            # fraction of joint velocity limits
        self.cartesian_speed = p('cartesian_speed', 0.04).value   # m/s for approach/retreat
        self.sim_grasp = p('sim_grasp', True).value               # drive Gazebo grasp joints
        self.base_frame = p('base_frame', 'arm_mount').value
        self.world_frame = p('world_frame', 'world').value
        self.gripper_open_position = p('gripper_open', 0.010).value
        self.gripper_effort = p('gripper_effort', 8.0).value
        # Distance fixed jaw -> object centre at the TCP. The jaw position
        # that closes on an object of width w is w - jaw_gap_offset.
        self.jaw_gap_offset = p('jaw_gap_offset', 0.0174).value

        self._lock = threading.Lock()
        self._joint_positions = {}
        self._estop = False
        self._urdf = None
        self.arm = self.rail = self.ik = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/robot_description', self._on_description, latched)
        node.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        node.create_subscription(Bool, '/agrobot/estop', self._on_estop, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.arm_client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.rail_client = ActionClient(node, FollowJointTrajectory, '/rail_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
        self._sim_pubs = {}
        self._active_goal = None

    # ------------------------------------------------------------ plumbing --
    def _on_description(self, msg):
        with self._lock:
            self._urdf = msg.data

    def _on_joint_states(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _on_estop(self, msg):
        if msg.data and not self._estop:
            self.log.error('EMERGENCY STOP received - cancelling motion.')
            goal = self._active_goal
            if goal is not None:
                goal.cancel_goal_async()
        self._estop = msg.data

    def check_estop(self):
        if self._estop:
            raise EmergencyStop('emergency stop active')

    def wait_until_ready(self, timeout=120.0, need_rail=True):
        deadline = time.monotonic() + timeout
        self.log.info('Waiting for robot description, joint states and controllers...')
        clients = [self.arm_client, self.gripper_client] + ([self.rail_client] if need_rail else [])
        while time.monotonic() < deadline:
            with self._lock:
                ready = self._urdf is not None and all(j in self._joint_positions for j in ARM_JOINTS)
            if ready and all(c.server_is_ready() for c in clients):
                break
            time.sleep(0.2)
        else:
            raise MotionError('robot not ready (controllers or /robot_description missing)')
        self.arm = Chain(self._urdf, self.base_frame, 'tcp')
        self.ik = IkSolver(self.arm)
        self.has_rail = 'rail_joint' in self._joint_positions
        self.log.info('Robot ready.')

    # --------------------------------------------------------------- state --
    def joints(self, names=ARM_JOINTS):
        with self._lock:
            return np.array([self._joint_positions[n] for n in names])

    @property
    def rail_position(self):
        with self._lock:
            return self._joint_positions.get('rail_joint', 0.0)

    def lookup(self, target, source, timeout=2.0):
        """4x4 transform that maps points in `source` into `target`."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                return transform_to_matrix(self.tf_buffer.lookup_transform(target, source, Time()))
            except Exception as exc:  # tf2 raises several exception types
                if time.monotonic() > deadline:
                    raise MotionError(f'no transform {source} -> {target}: {exc}') from exc
                time.sleep(0.05)

    def world_to_base(self, point_world, rail=None):
        """Point in world -> arm base frame, optionally for a future rail position."""
        t = self.lookup(self.base_frame, self.world_frame)
        p = np.asarray(point_world, float).copy()
        if rail is not None:
            p[0] -= rail - self.rail_position
        return (t @ np.append(p, 1.0))[:3]

    def tcp_pose(self):
        return self.arm.fk(self.joints())

    # ------------------------------------------------------------- actions --
    def _send(self, client, goal, timeout):
        self.check_estop()
        send = client.send_goal_async(goal)
        handle = self._wait(send, 5.0)
        if not handle.accepted:
            raise MotionError('goal rejected')
        self._active_goal = handle
        try:
            result = self._wait(handle.get_result_async(), timeout)
        finally:
            self._active_goal = None
        self.check_estop()
        return result.result

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise MotionError('action timed out')
            time.sleep(0.01)
        return future.result()

    def _follow(self, client, names, points, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = points
        goal.goal_time_tolerance = to_duration(1.0)
        result = self._send(client, goal, duration + 10.0)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise MotionError(f'trajectory failed ({result.error_code}): {result.error_string}')

    def move_joints(self, q_target, speed_scale=None):
        """Joint-space move with a quintic time profile."""
        q0 = self.joints()
        q1 = np.clip(np.asarray(q_target, float), self.arm.lower, self.arm.upper)
        vmax = self.arm.max_velocity * (speed_scale or self.speed_scale)
        # Quintic peak velocity is 1.875 x the mean velocity.
        duration = max(0.6, float(np.max(np.abs(q1 - q0) / vmax)) * 1.875)
        points = []
        for s_t in np.linspace(0.1, 1.0, 10):
            s = 10 * s_t ** 3 - 15 * s_t ** 4 + 6 * s_t ** 5
            ds = (30 * s_t ** 2 - 60 * s_t ** 3 + 30 * s_t ** 4) / duration
            points.append(JointTrajectoryPoint(
                positions=list(q0 + s * (q1 - q0)), velocities=list(ds * (q1 - q0)),
                time_from_start=to_duration(s_t * duration)))
        self._follow(self.arm_client, ARM_JOINTS, points, duration)

    def solve(self, position_base, approach=None, q_seed=None, max_approach_error=np.pi):
        seed = self.joints() if q_seed is None else q_seed
        return self.ik.solve(position_base, approach, seed, max_approach_error)

    def move_linear(self, position_base, approach=None, speed=None, step=0.005):
        """Straight TCP line from the current pose to `position_base`."""
        q = self.joints()
        start = self.arm.fk(q)[:3, 3]
        goal = np.asarray(position_base, float)
        n = max(2, int(np.ceil(np.linalg.norm(goal - start) / step)))
        speed = speed or self.cartesian_speed
        vmax = self.arm.max_velocity * self.speed_scale
        points, t = [], 0.0
        for i in range(1, n + 1):
            target = start + (goal - start) * i / n
            res = self.ik.track(target, approach, q)
            if res.position_error > 0.003:
                raise MotionError(f'linear move leaves the workspace at {np.round(target, 3)}')
            dq = res.q - q
            if np.max(np.abs(dq)) > 0.35:
                raise MotionError('linear move needs a joint flip (singularity)')
            t += max(np.linalg.norm(goal - start) / n / speed, float(np.max(np.abs(dq) / vmax)))
            points.append(JointTrajectoryPoint(positions=list(res.q), time_from_start=to_duration(t)))
            q = res.q
        # Zero velocity at the end so the controller stops smoothly.
        points[-1].velocities = [0.0] * len(ARM_JOINTS)
        self._follow(self.arm_client, ARM_JOINTS, points, t)

    def move_rail(self, x, speed=0.15):
        if not self.has_rail:
            return
        x0 = self.rail_position
        duration = max(0.8, abs(x - x0) / speed * 1.5)
        points = []
        for s_t in np.linspace(0.1, 1.0, 10):
            s = 10 * s_t ** 3 - 15 * s_t ** 4 + 6 * s_t ** 5
            ds = (30 * s_t ** 2 - 60 * s_t ** 3 + 30 * s_t ** 4) / duration
            points.append(JointTrajectoryPoint(positions=[x0 + s * (x - x0)], velocities=[ds * (x - x0)],
                                               time_from_start=to_duration(s_t * duration)))
        self._follow(self.rail_client, ['rail_joint'], points, duration)

    def gripper(self, position, effort=None):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort or self.gripper_effort)
        self._send(self.gripper_client, goal, 10.0)

    def open_gripper(self):
        self.gripper(self.gripper_open_position)

    def close_on(self, width):
        """Close the jaw onto an object of the given width (m)."""
        self.gripper(max(-0.017, width - self.jaw_gap_offset - 0.001))

    # --------------------------------------------------- simulation helpers --
    def _sim(self, obj, action):
        if not self.sim_grasp or not obj:
            return
        topic = f'/agrobot/sim/{obj}/{action}'
        if topic not in self._sim_pubs:
            self._sim_pubs[topic] = self.node.create_publisher(Empty, topic, 10)
            deadline = time.monotonic() + 2.0
            while self._sim_pubs[topic].get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.05)
        for _ in range(2):
            self._sim_pubs[topic].publish(Empty())
            time.sleep(0.05)

    def attach(self, obj):
        """Simulation: fix the object to the gripper (the real gripper just holds it)."""
        self._sim(obj, 'attach_gripper')

    def release(self, obj):
        self._sim(obj, 'detach_gripper')

    def detach_from_plant(self, obj):
        """Simulation: cut the peduncle / pull the root out of the soil."""
        self._sim(obj, 'detach_anchor')
