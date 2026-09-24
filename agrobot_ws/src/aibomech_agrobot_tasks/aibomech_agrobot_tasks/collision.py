"""Box-based collision checking of the arm against itself, its carrier and known obstacles.

Uses the same collision boxes as the URDF (and therefore as Gazebo), so a
motion that passes here does not jam in simulation. Oriented boxes are tested
with the separating axis theorem. Deliberately small: crops and scene fixtures
are handed in as boxes by the task nodes.
"""
import itertools
import xml.etree.ElementTree as ET

import numpy as np

from .kinematics import axis_angle, cross, transform


def _floats(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, float)


class Box:
    __slots__ = ('centre', 'rot', 'half')

    def __init__(self, centre, rot, half):
        self.centre, self.rot, self.half = centre, rot, half


def boxes_overlap(a, b, margin=0.0):
    """Separating-axis test for two oriented boxes."""
    t = b.centre - a.centre
    if np.dot(t, t) > (np.linalg.norm(a.half) + np.linalg.norm(b.half) + margin) ** 2:
        return False   # bounding spheres apart
    axes = [a.rot[:, i] for i in range(3)] + [b.rot[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            c = cross(a.rot[:, i], b.rot[:, j])
            if np.dot(c, c) > 1e-10:
                axes.append(c / np.linalg.norm(c))
    for axis in axes:
        ra = np.sum(a.half * np.abs(a.rot.T @ axis))
        rb = np.sum(b.half * np.abs(b.rot.T @ axis))
        if abs(np.dot(t, axis)) > ra + rb + margin:
            return False
    return True


class CollisionModel:
    """Tree FK over the whole URDF plus the collision boxes of every link."""

    def __init__(self, urdf_xml, base_frame, moving_links, ignore_pairs=(), margin=0.002,
                 obstacle_clearance=0.003):
        root = ET.fromstring(urdf_xml)
        # Robot parts may touch their neighbours by up to `margin` (the boxes
        # are conservative); scene obstacles must stay `obstacle_clearance` away,
        # because touching them stalls the arm in simulation and on the robot.
        self.margin = margin
        self.obstacle_clearance = obstacle_clearance
        self.joints = {}
        self.children = {}
        for j in root.findall('joint'):
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            o = j.find('origin')
            origin = transform(_floats(o.get('xyz') if o is not None else None, (0, 0, 0)),
                               _floats(o.get('rpy') if o is not None else None, (0, 0, 0)))
            a = j.find('axis')
            axis = _floats(a.get('xyz') if a is not None else None, (1, 0, 0))
            self.joints[child] = (j.get('name'), j.get('type'), parent, origin, axis)
            self.children.setdefault(parent, []).append(child)
        self.boxes = {}
        for link in root.findall('link'):
            for c in link.findall('collision'):
                b = c.find('geometry/box')
                if b is None:
                    continue
                o = c.find('origin')
                local = transform(_floats(o.get('xyz') if o is not None else None, (0, 0, 0)),
                                  _floats(o.get('rpy') if o is not None else None, (0, 0, 0)))
                self.boxes.setdefault(link.get('name'), []).append((local, _floats(b.get('size'), (0, 0, 0)) / 2))
        self.base_frame = base_frame
        self.moving = [l for l in moving_links if l in self.boxes]
        # Everything rigidly attached to the base frame's carrier counts as static.
        self.static = [l for l in self.boxes if l not in moving_links and self._rigid_to_base(l)]
        adjacent = {(self.joints[l][2], l) for l in self.moving if l in self.joints}
        ignore = {tuple(sorted(p)) for p in list(adjacent) + list(ignore_pairs)}
        self.pairs = [(a, b) for a, b in itertools.product(self.moving, self.static)
                      if tuple(sorted((a, b))) not in ignore]
        self.pairs += [(a, b) for a, b in itertools.combinations(self.moving, 2)
                       if tuple(sorted((a, b))) not in ignore]
        self.obstacles = []
        self.obstacle_clearances = []

    def _rigid_to_base(self, link):
        """True if no movable joint separates `link` from the base frame's carrier."""
        def path_to_root(l):
            out = []
            while l in self.joints:
                out.append(l)
                l = self.joints[l][2]
            return out
        a, b = path_to_root(link), path_to_root(self.base_frame)
        common = set(a) & set(b)
        for l in a + b:
            if l not in common and self.joints[l][1] not in ('fixed',):
                return False
        return True

    def set_obstacles(self, boxes_base):
        """Obstacles as (centre[3], size[3]) or (centre[3], size[3], clearance), axis-aligned
        in the base frame. Without an explicit clearance, obstacle_clearance applies."""
        self.obstacles = []
        self.obstacle_clearances = []
        for box in boxes_base:
            c, s = box[0], box[1]
            self.obstacles.append(Box(np.asarray(c, float), np.eye(3), np.asarray(s, float) / 2))
            self.obstacle_clearances.append(box[2] if len(box) > 2 else self.obstacle_clearance)

    def link_poses(self, joint_values):
        """Poses of all links relative to the base frame."""
        poses = {}
        roots = [l for l in self.children if l not in self.joints]

        def visit(link, t):
            poses[link] = t
            for child in self.children.get(link, []):
                name, jtype, _, origin, axis = self.joints[child]
                m = np.eye(4)
                q = joint_values.get(name, 0.0)
                if jtype in ('revolute', 'continuous'):
                    m[:3, :3] = axis_angle(axis / np.linalg.norm(axis), q)
                elif jtype == 'prismatic':
                    m[:3, 3] = axis / np.linalg.norm(axis) * q
                visit(child, t @ origin @ m)
        for r in roots:
            visit(r, np.eye(4))
        base_inv = np.linalg.inv(poses[self.base_frame])
        return {k: base_inv @ v for k, v in poses.items()}

    def _world_boxes(self, link, pose):
        return [Box((pose @ local)[:3, 3], (pose @ local)[:3, :3], half) for local, half in self.boxes[link]]

    def collisions(self, joint_values, held_radius=0.0, tcp_link='tcp', held_half=None, held_offset=None):
        """List of colliding (link, other) pairs for a joint configuration.

        A held crop is a box at the TCP: a cube of half-size held_radius, or
        held_half (half extents in the tcp frame) shifted by held_offset.
        """
        if held_half is not None:
            held_radius = float(np.max(held_half))
        poses = self.link_poses(joint_values)
        boxes = {l: self._world_boxes(l, poses[l]) for l in set(itertools.chain(*self.pairs))}
        hits = []
        for a, b in self.pairs:
            if any(boxes_overlap(x, y, -self.margin) for x in boxes[a] for y in boxes[b]):
                hits.append((a, b))
        moving_boxes = [(l, bx) for l in self.moving for bx in boxes[l]]
        if held_radius > 0 and tcp_link in poses:
            t = poses[tcp_link]
            half = np.full(3, held_radius) if held_half is None else np.asarray(held_half, float)
            offset = np.zeros(3) if held_offset is None else np.asarray(held_offset, float)
            moving_boxes.append(('held_object', Box(t[:3, 3] + t[:3, :3] @ offset, t[:3, :3], half)))
            for l in self.static:
                for bx in self._world_boxes(l, poses[l]):
                    if boxes_overlap(moving_boxes[-1][1], bx, -self.margin):
                        hits.append(('held_object', l))
        for name, bx in moving_boxes:
            for k, ob in enumerate(self.obstacles):
                if boxes_overlap(bx, ob, self.obstacle_clearances[k]):
                    hits.append((name, f'obstacle_{k}'))
        return hits
