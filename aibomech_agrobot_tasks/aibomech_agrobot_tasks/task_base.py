"""Common scaffolding of the agricultural task nodes."""
import csv
import json
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from .perception import CropDetector
from .planner import plan
from .robot_interface import EmergencyStop, MotionError, RobotInterface

# Folded beside the column, collision free even with a fruit in the gripper,
# and outside the camera's view of the crop row.
HOME = [-0.97, -2.0, -1.12, 1.46]
PRE_EXTRA_TILT = np.radians(20.0)


@dataclass
class ReachPlan:
    rail: float
    q_pre: np.ndarray
    q_grasp: np.ndarray
    approach_error: float


class AgrobotTask:
    """Owns the node, the robot interface, the camera and the report."""

    name = 'agrobot_task'
    uses_camera = True

    def __init__(self):
        self.node = Node(self.name)
        self.robot = RobotInterface(self.node)
        self.camera = CropDetector(self.node, self.robot) if self.uses_camera else None
        p = self.node.declare_parameter
        self.home = np.array(p('home_joints', HOME).value, float)
        # Stay clear of the hard stops of the rail.
        self.rail_limits = p('rail_limits', [0.03, 2.97]).value
        # Fixed obstacles of the scene as flat list [cx, cy, cz, sx, sy, sz, ...] (world frame)
        obst = p('obstacles', [0.0] * 6).value
        self.robot.world_obstacles = [(np.array(obst[i:i + 3]), np.array(obst[i + 3:i + 6]))
                                      for i in range(0, len(obst), 6) if any(obst[i + 3:i + 6])]
        self.report_root = os.path.expanduser(p('report_dir', '~/.ros/agrobot_reports').value)
        # Simulation only: ground truth of the world, used to score the run.
        truth_file = p('sim_objects_file', '').value
        self.truth = {}
        if truth_file:
            with open(truth_file) as fh:
                self.truth = {o['name']: o['type'] for o in yaml.safe_load(fh)['objects']}
        self.log = self.node.get_logger()
        self.rows = []
        self.summary = {'task': self.name}
        self._sim_poses = {}
        self._lock = threading.Lock()
        self.node.create_subscription(TFMessage, '/agrobot/sim/model_poses', self._on_sim_poses, 10)

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    # ------------------------------------------------------------ helpers --
    def param(self, name, default):
        return self.node.declare_parameter(name, default).value

    def step(self, phase, action, *args):
        """Runs one motion of a task cycle and names the phase if it fails."""
        self.log.debug(f'{phase}: {action.__name__}')
        try:
            return action(*args)
        except MotionError as exc:
            raise MotionError(f'{phase} failed: {exc}') from exc

    def go_home(self):
        self.robot.move_joints(self.home)

    def recover(self):
        """After a failed pick: let go of whatever is held and return home."""
        try:
            self.robot.open_gripper()
            self.robot.release()
            self.robot.move_joints(self.home)
        except MotionError as exc:
            self.log.error(f'recovery failed: {exc}')

    def plan_reach(self, target_world, approach, pre_distance, rail_offsets, max_approach_error,
                   pre_offset=None, retreat=None):
        """Chooses the rail position and joint solutions to reach a world point.

        The linear axis is used like an external axis of an industrial cell:
        each candidate offset puts the target at a different place in the arm's
        workspace, and the one with the best tool orientation wins.
        """
        approach = np.asarray(approach, float) / np.linalg.norm(approach)
        pre_vec = -approach * pre_distance if pre_offset is None else np.asarray(pre_offset, float)
        best = None
        candidates = rail_offsets if self.robot.has_rail else [None]
        try:
            best = self._plan_reach(target_world, approach, pre_vec, candidates, max_approach_error, retreat)
        finally:
            self.robot.planning_rail = None
            self.robot.update_obstacles()
        return best

    def _plan_reach(self, target_world, approach, pre_vec, candidates, max_approach_error, retreat):
        feasible = []
        for dx in candidates:
            rail = None
            if dx is not None:
                rail = float(np.clip(target_world[0] - dx, *self.rail_limits))
            p_grasp = self.robot.world_to_base(target_world, rail)
            self.robot.planning_rail = rail
            grasp = self.robot.solve(p_grasp, approach, self.home, max_approach_error)
            if not grasp.success:
                continue
            # The pre-grasp pose may tilt more; only the grasp itself must be well aligned.
            pre = self.robot.ik.solve(p_grasp + pre_vec, approach, grasp.q, max_approach_error + PRE_EXTRA_TILT)
            if not pre.success or not self.robot.line_free(pre.q, p_grasp, approach):
                continue
            if retreat is not None and not self.robot.line_free(grasp.q, p_grasp + retreat, approach):
                continue
            err = grasp.approach_error
            feasible.append((err, rail, pre.q, grasp.q))
        # Best tool orientation first, but only if the arm can actually get there
        # from home without collisions (some solutions lie behind the column).
        for err, rail, q_pre, q_grasp in sorted(feasible, key=lambda f: f[0]):
            self.robot.planning_rail = rail
            self.robot.update_obstacles()
            if plan(self.home, q_pre, self.robot.is_free, self.robot.arm.lower + 0.02,
                    self.robot.arm.upper - 0.02, max_iterations=600) is not None:
                return ReachPlan(rail if rail is not None else self.robot.rail_position, q_pre, q_grasp, err)
        return None

    def set_crop_obstacles(self, points, size, exclude=None, exclude_radius=0.02):
        """Treat detected crops (except the target) as obstacles."""
        self.robot.crop_obstacles = [
            (np.asarray(p, float), np.full(3, size)) for p in points
            if exclude is None or np.linalg.norm(np.asarray(p) - exclude) > exclude_radius]

    def base_point(self, world_point):
        return self.robot.world_to_base(world_point)

    # --------------------------------------------------- simulation truth --
    def _on_sim_poses(self, msg):
        with self._lock:
            for t in msg.transforms:
                tr = t.transform.translation
                self._sim_poses[t.child_frame_id] = np.array([tr.x, tr.y, tr.z])

    def sim_poses(self, prefix=''):
        with self._lock:
            return {k: v.copy() for k, v in self._sim_poses.items() if k.startswith(prefix)}

    def sim_object_near(self, world_point, prefix, max_distance=0.03):
        """Name of the simulated object closest to a detection (for the grasp joints)."""
        best, best_d = None, max_distance
        for name, pos in self.sim_poses(prefix).items():
            d = float(np.linalg.norm(pos - world_point))
            if d < best_d:
                best, best_d = name, d
        return best

    def in_crate(self, prefix):
        """Simulated objects whose centre is inside the crate on the trolley."""
        try:
            crate = self.robot.lookup(self.robot.world_frame, 'crate')[:3, 3]
        except MotionError:
            return []
        inside = []
        for name, p in self.sim_poses(prefix).items():
            d = p - crate
            if abs(d[0]) < 0.08 and abs(d[1]) < 0.06 and -0.01 < d[2] < 0.06:
                inside.append(name)
        return sorted(inside)

    # ------------------------------------------------------------- report --
    def write_report(self, images=None):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(self.report_root, f'{self.name}_{stamp}')
        os.makedirs(out, exist_ok=True)
        if self.rows:
            with open(os.path.join(out, 'results.csv'), 'w', newline='') as fh:
                writer = csv.DictWriter(fh, fieldnames=list(self.rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.rows)
        with open(os.path.join(out, 'summary.json'), 'w') as fh:
            json.dump(self.summary, fh, indent=2, default=float)
        for name, img in (images or {}).items():
            cv2.imwrite(os.path.join(out, name), img)
        self.log.info(f'Report written to {out}')
        for k, v in self.summary.items():
            self.log.info(f'  {k}: {v}')
        return out

    # ---------------------------------------------------------------- run --
    def execute(self):
        raise NotImplementedError

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


def run_task(task_cls):
    rclpy.init()
    task = task_cls()
    code = 0
    t0 = time.monotonic()
    try:
        task.robot.wait_until_ready()
        if task.camera and not task.camera.wait_for_camera():
            raise MotionError('no camera images on ' + task.camera.color_topic)
        task.execute()
    except EmergencyStop:
        task.log.error('Task stopped by the emergency stop. Reset the e-stop and restart the task.')
        code = 2
    except (MotionError, RuntimeError) as exc:
        task.log.error(f'Task aborted: {exc}')
        task.summary['aborted'] = str(exc)
        code = 1
    except KeyboardInterrupt:
        code = 130
    finally:
        task.summary['total_time_s'] = round(time.monotonic() - t0, 1)
        if code in (1, 2):
            task.write_report()
        task.shutdown()
        rclpy.try_shutdown()
    return code
