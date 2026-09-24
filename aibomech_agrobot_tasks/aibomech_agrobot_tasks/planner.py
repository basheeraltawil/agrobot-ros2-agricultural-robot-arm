"""Joint-space RRT-Connect with shortcut smoothing.

A compact stand-in for MoveIt/OMPL that the task nodes can run without a
move_group: the arm has only four joints, so a few hundred collision checks
find a path around the column, crate and crop structures.
"""
import numpy as np


def edge_free(q0, q1, valid, resolution):
    n = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / resolution)))
    return all(valid(q0 + (q1 - q0) * k / n) for k in range(1, n + 1))


def _extend(tree, parents, target, step, valid, resolution):
    dists = [np.linalg.norm(q - target) for q in tree]
    i = int(np.argmin(dists))
    q_near = tree[i]
    d = dists[i]
    q_new = target if d <= step else q_near + (target - q_near) * step / d
    if not edge_free(q_near, q_new, valid, resolution):
        return None
    tree.append(q_new)
    parents.append(i)
    return len(tree) - 1


def _connect(tree, parents, target, step, valid, resolution):
    while True:
        idx = _extend(tree, parents, target, step, valid, resolution)
        if idx is None:
            return None
        if np.linalg.norm(tree[idx] - target) < 1e-9:
            return idx


def _branch(tree, parents, idx):
    out = []
    while idx is not None and idx >= 0:
        out.append(tree[idx])
        idx = parents[idx]
    return out


def plan(q_start, q_goal, valid, lower, upper, step=0.25, resolution=0.04,
         max_iterations=1500, seed=0):
    """Collision-free list of waypoints from q_start to q_goal, or None."""
    q_start, q_goal = np.asarray(q_start, float), np.asarray(q_goal, float)
    if edge_free(q_start, q_goal, valid, resolution):
        return [q_start, q_goal]
    rng = np.random.default_rng(seed)
    a, pa = [q_start], [-1]
    b, pb = [q_goal], [-1]
    for it in range(max_iterations):
        sample = q_goal if it % 10 == 0 else rng.uniform(lower, upper)
        idx = _extend(a, pa, sample, step, valid, resolution)
        if idx is not None:
            j = _connect(b, pb, a[idx], step, valid, resolution)
            if j is not None:
                path = _branch(a, pa, idx)[::-1] + _branch(b, pb, j)[1:]
                if a[0] is not q_start:     # trees were swapped an odd number of times
                    path = path[::-1]
                return shortcut(path, valid, resolution, rng)
        a, pa, b, pb = b, pb, a, pa
    return None


def shortcut(path, valid, resolution, rng, attempts=60):
    path = list(path)
    for _ in range(attempts):
        if len(path) <= 2:
            break
        i, j = sorted(rng.choice(len(path), 2, replace=False))
        if j - i < 2:
            continue
        if edge_free(path[i], path[j], valid, resolution):
            path = path[:i + 1] + path[j:]
    return path
