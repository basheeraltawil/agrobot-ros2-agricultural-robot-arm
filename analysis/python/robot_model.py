"""Rigid-body model of the AgroBot arm, built directly from its URDF.

This is the numeric counterpart of the Mathematica derivation in
analysis/mathematica/: the same formulas, applied to the real robot
(3D joint axes, CAD masses including the servos, gravity included).

    For every link i with mass m_i, centre of mass p_ci(q) and inertia I_i:

    J_vi = d p_ci / d q                     linear Jacobian of the COM   (3 x n)
    J_wi = [z_1 ... z_i 0 ... 0]            angular Jacobian (joint axes) (3 x n)

    M(q)  = sum_i  m_i J_viᵀ J_vi + J_wiᵀ R_i I_i R_iᵀ J_wi        inertia matrix
    c_ijk = 1/2 (dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k)             Christoffel symbols
    C_kj  = sum_i c_ijk q̇_i                                         Coriolis/centrifugal
    G(q)  = -sum_i J_viᵀ m_i g                                      gravity torques

    τ = M(q) q̈ + C(q, q̇) q̇ + G(q)                                 joint torques

Units are SI: m, kg, s, rad, N·m.

Usage:
    model = ArmModel.from_urdf('analysis/data/agrobot.urdf')
    tau = model.inverse_dynamics(q, qd, qdd)
"""
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])
ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


def rpy_matrix(roll, pitch, yaw):
    """Rotation matrix of URDF roll-pitch-yaw angles (fixed axes x, y, z)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rotation(axis, angle):
    """Rodrigues' formula: rotation by `angle` about the unit vector `axis`."""
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def homogeneous(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def _vec(element, attr, default):
    if element is None or element.get(attr) is None:
        return np.array(default, float)
    return np.array([float(v) for v in element.get(attr).split()])


@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))       # in the link frame
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))  # about the COM, link axes


@dataclass
class Joint:
    name: str
    kind: str                 # revolute | prismatic | fixed
    parent: str
    child: str
    origin: np.ndarray        # 4x4 parent -> joint frame
    axis: np.ndarray          # unit vector in the joint frame
    lower: float = -np.inf
    upper: float = np.inf
    effort: float = np.inf
    velocity: float = np.inf


class ArmModel:
    """Serial chain from `base` through the arm joints; later links ride along."""

    def __init__(self, links, joints, base, joint_names):
        self.links = links
        self.joints = joints            # child link name -> Joint
        self.base = base
        self.joint_names = joint_names
        by_name = {j.name: j for j in joints.values()}
        self.arm_joints = [by_name[n] for n in joint_names]
        # Every link whose pose depends on the arm joints, in chain order.
        self.moving_links = self._links_below(self.arm_joints[0].child)

    @classmethod
    def from_urdf(cls, path, base='arm_mount', joint_names=ARM_JOINTS):
        root = ET.parse(path).getroot()
        links = {}
        for el in root.findall('link'):
            link = Link(el.get('name'))
            inertial = el.find('inertial')
            if inertial is not None:
                link.mass = float(inertial.find('mass').get('value'))
                link.com = _vec(inertial.find('origin'), 'xyz', (0, 0, 0))
                i = inertial.find('inertia')
                v = {k: float(i.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
                link.inertia = np.array([[v['ixx'], v['ixy'], v['ixz']],
                                         [v['ixy'], v['iyy'], v['iyz']],
                                         [v['ixz'], v['iyz'], v['izz']]])
            links[link.name] = link
        joints = {}
        for el in root.findall('joint'):
            origin = el.find('origin')
            axis = _vec(el.find('axis'), 'xyz', (1, 0, 0))
            joint = Joint(el.get('name'), el.get('type'), el.find('parent').get('link'),
                          el.find('child').get('link'),
                          homogeneous(rpy_matrix(*_vec(origin, 'rpy', (0, 0, 0))), _vec(origin, 'xyz', (0, 0, 0))),
                          axis / np.linalg.norm(axis))
            limit = el.find('limit')
            if limit is not None:
                joint.lower = float(limit.get('lower', -np.inf))
                joint.upper = float(limit.get('upper', np.inf))
                joint.effort = float(limit.get('effort', np.inf))
                joint.velocity = float(limit.get('velocity', np.inf))
            joints[joint.child] = joint
        return cls(links, joints, base, list(joint_names))

    # ------------------------------------------------------------ geometry --
    def _links_below(self, link):
        out = [link]
        for child, joint in self.joints.items():
            if joint.parent == link:
                out += self._links_below(child)
        return out

    @property
    def lower(self):
        return np.array([j.lower for j in self.arm_joints])

    @property
    def upper(self):
        return np.array([j.upper for j in self.arm_joints])

    def link_poses(self, q):
        """Pose (4x4, base frame) of every moving link, plus the joint frames.

        Returns (poses, joint_frames) where joint_frames[k] is the frame of
        arm joint k before its own motion (its axis is fixed in that frame).
        """
        values = dict(zip(self.joint_names, q))
        poses, joint_frames = {}, {}

        def pose_of(link):
            if link == self.base:
                return np.eye(4)
            if link in poses:
                return poses[link]
            joint = self.joints[link]
            frame = pose_of(joint.parent) @ joint.origin
            motion = np.eye(4)
            if joint.name in values and joint.kind in ('revolute', 'continuous'):
                motion[:3, :3] = axis_rotation(joint.axis, values[joint.name])
            elif joint.name in values and joint.kind == 'prismatic':
                motion[:3, 3] = joint.axis * values[joint.name]
            if joint.name in values:
                joint_frames[joint.name] = frame
            poses[link] = frame @ motion
            return poses[link]

        for link in self.moving_links:
            pose_of(link)
        return poses, joint_frames

    def com_jacobians(self, q):
        """For every moving link: (world COM, J_v 3xn, J_w 3xn, rotation R)."""
        poses, frames = self.link_poses(q)
        n = len(self.arm_joints)
        result = {}
        for name in self.moving_links:
            link = self.links[name]
            if link.mass <= 0:
                continue
            pose = poses[name]
            p_c = pose[:3, :3] @ link.com + pose[:3, 3]
            jv, jw = np.zeros((3, n)), np.zeros((3, n))
            for k, joint in enumerate(self.arm_joints):
                if not self._is_ancestor(joint.child, name):
                    continue            # joint k does not move this link
                frame = frames[joint.name]
                z = frame[:3, :3] @ joint.axis           # joint axis in the base frame
                if joint.kind == 'prismatic':
                    jv[:, k] = z
                else:
                    jv[:, k] = np.cross(z, p_c - frame[:3, 3])
                    jw[:, k] = z
            result[name] = (p_c, jv, jw, pose[:3, :3])
        return result

    def _is_ancestor(self, ancestor, link):
        while True:
            if link == ancestor:
                return True
            if link not in self.joints:
                return False
            link = self.joints[link].parent

    # ------------------------------------------------------------ dynamics --
    def mass_matrix(self, q):
        """M(q) = Σ m J_vᵀ J_v + J_wᵀ R I Rᵀ J_w."""
        n = len(self.arm_joints)
        m_q = np.zeros((n, n))
        for name, (_, jv, jw, rot) in self.com_jacobians(q).items():
            link = self.links[name]
            m_q += link.mass * jv.T @ jv + jw.T @ rot @ link.inertia @ rot.T @ jw
        return m_q

    def gravity_torque(self, q, gravity=GRAVITY):
        """G(q) = -Σ J_vᵀ m g: the torque each joint needs to hold the arm still."""
        g_q = np.zeros(len(self.arm_joints))
        for name, (_, jv, _, _) in self.com_jacobians(q).items():
            g_q -= jv.T @ (self.links[name].mass * gravity)
        return g_q

    def coriolis_matrix(self, q, qd, eps=1e-6):
        """C(q, q̇) from Christoffel symbols; dM/dq by central differences."""
        n = len(q)
        dm = np.zeros((n, n, n))                 # dm[:, :, i] = dM/dq_i
        for i in range(n):
            step = np.zeros(n)
            step[i] = eps
            dm[:, :, i] = (self.mass_matrix(q + step) - self.mass_matrix(q - step)) / (2 * eps)
        c = np.zeros((n, n))
        for k in range(n):
            for j in range(n):
                c[k, j] = sum(0.5 * (dm[k, j, i] + dm[k, i, j] - dm[i, j, k]) * qd[i] for i in range(n))
        return c

    def inverse_dynamics(self, q, qd, qdd):
        """τ = M q̈ + C q̇ + G."""
        q, qd, qdd = map(np.asarray, (q, qd, qdd))
        return self.mass_matrix(q) @ qdd + self.coriolis_matrix(q, qd) @ qd + self.gravity_torque(q)

    def tcp_position(self, q, tcp='tcp'):
        poses, _ = self.link_poses(q)
        return poses[tcp][:3, 3]
