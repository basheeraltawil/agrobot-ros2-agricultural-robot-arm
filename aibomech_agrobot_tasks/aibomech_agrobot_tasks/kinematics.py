"""Kinematics of a serial chain read from the URDF.

No ROS dependency, so it can be unit tested and used offline.

The AgroBot arm has 4 axes, so a TCP position (3 DOF) leaves one redundant
degree of freedom. The IK solver uses it to bring the gripper approach axis
(tcp z) as close as possible to a requested direction: position is the
primary task, approach direction is optimised in the null space.
"""
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def cross(a, b):
    """3-vector cross product (np.cross is slow for single small vectors)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


@dataclass
class Segment:
    name: str
    type: str            # fixed | revolute | prismatic | continuous
    origin: np.ndarray   # 4x4 parent -> joint frame
    axis: np.ndarray
    lower: float = -np.inf
    upper: float = np.inf
    velocity: float = np.inf

    @property
    def movable(self):
        return self.type != 'fixed'

    def motion(self, q):
        t = np.eye(4)
        if self.type in ('revolute', 'continuous'):
            t[:3, :3] = axis_angle(self.axis, q)
        elif self.type == 'prismatic':
            t[:3, 3] = self.axis * q
        return t


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


class Chain:
    """Kinematic chain between two links of a URDF."""

    def __init__(self, urdf_xml, base_link, tip_link, prefix=''):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall('joint'):
            by_child[j.find('child').get('link')] = j
        base_link, tip_link = prefix + base_link, prefix + tip_link

        path = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f'{tip_link} is not below {base_link} in the URDF')
            j = by_child[link]
            path.append(j)
            link = j.find('parent').get('link')
        path.reverse()

        self.segments = []
        for j in path:
            origin = j.find('origin')
            xyz = _floats(origin.get('xyz') if origin is not None else None, (0, 0, 0))
            rpy = _floats(origin.get('rpy') if origin is not None else None, (0, 0, 0))
            axis_el = j.find('axis')
            axis = np.array(_floats(axis_el.get('xyz') if axis_el is not None else None, (1, 0, 0)))
            seg = Segment(j.get('name'), j.get('type'), transform(xyz, rpy), axis / np.linalg.norm(axis))
            limit = j.find('limit')
            if limit is not None and seg.type != 'continuous':
                seg.lower = float(limit.get('lower', -np.inf))
                seg.upper = float(limit.get('upper', np.inf))
            if limit is not None:
                seg.velocity = float(limit.get('velocity', np.inf))
            self.segments.append(seg)

        self.movable = [s for s in self.segments if s.movable]
        self.joint_names = [s.name for s in self.movable]
        self.lower = np.array([s.lower for s in self.movable])
        self.upper = np.array([s.upper for s in self.movable])
        self.max_velocity = np.array([s.velocity for s in self.movable])

    @property
    def dof(self):
        return len(self.movable)

    def fk(self, q):
        """Pose of the tip in the base frame."""
        return self._frames(q)[0]

    def _frames(self, q):
        """Tip pose and, for every movable joint, (joint frame before motion, segment)."""
        t = np.eye(4)
        joints = []
        i = 0
        for seg in self.segments:
            t = t @ seg.origin
            if seg.movable:
                joints.append((t.copy(), seg))
                t = t @ seg.motion(q[i])
                i += 1
        return t, joints

    def jacobian(self, q):
        """Geometric Jacobian (6 x dof) of the tip, expressed in the base frame."""
        tip, joints = self._frames(q)
        p_tip = tip[:3, 3]
        jac = np.zeros((6, self.dof))
        for i, (frame, seg) in enumerate(joints):
            axis = frame[:3, :3] @ seg.axis
            if seg.type == 'prismatic':
                jac[:3, i] = axis
            else:
                jac[:3, i] = cross(axis, p_tip - frame[:3, 3])
                jac[3:, i] = axis
        return tip, jac

    def clamp(self, q, margin=0.0):
        return np.clip(q, self.lower + margin, self.upper - margin)


@dataclass
class IkResult:
    q: np.ndarray
    position_error: float     # m
    approach_error: float     # rad, angle between tcp z and requested approach
    success: bool
    valid: bool = True        # passed the validity check (collision free)


class IkSolver:
    """Position IK with approach direction optimised in the null space."""

    def __init__(self, chain, position_tolerance=0.002, limit_margin=0.02,
                 max_iterations=200, restarts=12, seed=7, valid=None):
        self.chain = chain
        # Optional check q -> bool (e.g. collision free); invalid solutions lose.
        self.valid = valid
        self.position_tolerance = position_tolerance
        self.limit_margin = limit_margin
        self.max_iterations = max_iterations
        self.restarts = restarts
        self.rng = np.random.default_rng(seed)

    def solve(self, position, approach=None, q_seed=None, max_approach_error=np.pi):
        """Find joint angles that put the tip at `position` (base frame).

        approach: desired direction of the tip z axis, or None for position only.
        Returns the best solution; success means position within tolerance and
        approach within max_approach_error.
        """
        position = np.asarray(position, float)
        if approach is not None:
            approach = np.asarray(approach, float) / np.linalg.norm(approach)
        lo = self.chain.lower + self.limit_margin
        hi = self.chain.upper - self.limit_margin
        seeds = []
        if q_seed is not None:
            seeds.append(np.clip(np.asarray(q_seed, float), lo, hi))
        seeds += [self.rng.uniform(lo, hi) for _ in range(self.restarts)]

        best = None
        for q0 in seeds:
            res = self._descend(q0, position, approach, lo, hi)
            if best is None or self._score(res, q_seed) < self._score(best, q_seed):
                best = res
            if res.position_error < self.position_tolerance and res.valid and (
                    approach is None or res.approach_error < 0.05) and q_seed is not None:
                break
        best.success = bool(best.position_error < self.position_tolerance
                            and best.approach_error <= max_approach_error
                            and best.valid)
        return best

    def track(self, position, approach, q_seed):
        """Local solution next to q_seed, for following a path without jumps."""
        lo = self.chain.lower + self.limit_margin
        hi = self.chain.upper - self.limit_margin
        res = self._descend(np.clip(np.asarray(q_seed, float), lo, hi),
                            np.asarray(position, float),
                            None if approach is None else np.asarray(approach, float) / np.linalg.norm(approach),
                            lo, hi)
        res.success = bool(res.position_error < self.position_tolerance and res.valid)
        return res

    def _score(self, res, q_seed):
        # Lexicographic: any solution inside the position tolerance beats any
        # solution outside it; then smallest approach error; then least motion.
        score = res.approach_error
        if not res.valid:
            score += 50.0
        if res.position_error > self.position_tolerance:
            score += 100.0 + res.position_error * 1000.0
        if q_seed is not None:
            score += 0.02 * np.linalg.norm(res.q - q_seed)
        return score

    def _descend(self, q, target, approach, lo, hi):
        # Stage 1 reaches the position. Stage 2 turns the tool towards the
        # approach direction with null-space motion and is followed by a
        # position-only polish. Position always wins over approach.
        q_pos = self._iterate(q, target, None, lo, hi)
        candidates = [q_pos]
        if approach is not None:
            q_app = self._iterate(q_pos, target, approach, lo, hi)
            candidates.append(self._iterate(q_app, target, None, lo, hi))
        results = [self._evaluate(c, target, approach) for c in candidates]
        if self.valid is not None:
            for r in results:
                if r.position_error < self.position_tolerance:
                    r.valid = bool(self.valid(r.q))
        return min(results, key=lambda r: self._score(r, None))

    def _evaluate(self, q, target, approach):
        tip = self.chain.fk(q)
        pos_err = float(np.linalg.norm(target - tip[:3, 3]))
        ang_err = 0.0
        if approach is not None:
            ang_err = float(np.arccos(np.clip(tip[:3, 2] @ approach, -1.0, 1.0)))
        return IkResult(q, pos_err, ang_err, False)

    def _iterate(self, q, target, approach, lo, hi):
        eye3 = np.eye(3)
        for _ in range(self.max_iterations):
            tip, jac = self.chain.jacobian(q)
            e_p = target - tip[:3, 3]
            jp = jac[:3]
            jp_pinv = jp.T @ np.linalg.inv(jp @ jp.T + 1e-4 * eye3)
            dq = jp_pinv @ e_p
            if approach is not None:
                e_a = cross(tip[:3, 2], approach)   # rotation turning tcp z onto approach
                null = np.eye(self.chain.dof) - jp_pinv @ jp
                ja_n = jac[3:] @ null
                dq += 0.5 * null @ (ja_n.T @ np.linalg.solve(ja_n @ ja_n.T + 1e-3 * eye3, e_a))
            step = np.linalg.norm(dq)
            if step > 0.2:
                dq *= 0.2 / step
            q = np.clip(q + dq, lo, hi)
            if step < 1e-4:
                break
        return q
