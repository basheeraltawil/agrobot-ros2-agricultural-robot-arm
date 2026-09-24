"""Offline tests of kinematics, collision checking and planning (no ROS needed).

test/data/agrobot.urdf is the expanded default xacro. Regenerate it with
  xacro ../aibomech_agrobot_description/urdf/agrobot.urdf.xacro > test/data/agrobot.urdf
"""
import os

import numpy as np
import pytest

from aibomech_agrobot_tasks.collision import CollisionModel
from aibomech_agrobot_tasks.kinematics import Chain, IkSolver
from aibomech_agrobot_tasks.planner import edge_free, plan
from aibomech_agrobot_tasks.task_base import HOME

URDF = open(os.path.join(os.path.dirname(__file__), 'data', 'agrobot.urdf')).read()
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
LINKS = ['link_1', 'link_2', 'link_3', 'link_4', 'link_5']


@pytest.fixture(scope='module')
def chain():
    return Chain(URDF, 'arm_mount', 'tcp')


@pytest.fixture(scope='module')
def collision():
    return CollisionModel(URDF, 'arm_mount', LINKS)


def test_jacobian_matches_finite_differences(chain):
    q = np.array([0.3, -0.4, 0.2, 0.5])
    tip, jac = chain.jacobian(q)
    eps = 1e-6
    numeric = np.array([(chain.fk(q + eps * np.eye(4)[i])[:3, 3] - tip[:3, 3]) / eps for i in range(4)]).T
    assert np.allclose(numeric, jac[:3], atol=1e-5)


@pytest.mark.parametrize('target,approach,max_deg', [
    ((0.18, 0.19, 0.03), (0, 1, 0), 10),      # strawberry, horizontal approach
    ((0.0, 0.29, -0.058), (0, 0, -1), 10),    # soil block, top-down
])
def test_ik_reaches_task_poses(chain, target, approach, max_deg):
    res = IkSolver(chain).solve(target, approach, np.array(HOME))
    assert res.position_error < 0.002
    assert np.degrees(res.approach_error) < max_deg


def test_home_is_collision_free_even_when_holding(collision):
    assert collision.collisions(dict(zip(JOINTS, HOME)), held_radius=0.012) == []


def test_obstacle_is_detected(collision, chain):
    q = np.zeros(4)
    tcp = chain.fk(q)[:3, 3]
    collision.set_obstacles([(tcp, (0.05, 0.05, 0.05))])
    try:
        assert collision.collisions(dict(zip(JOINTS, q)))
    finally:
        collision.set_obstacles([])


def test_planner_returns_collision_free_path(collision, chain):
    free = lambda q: not collision.collisions(dict(zip(JOINTS, q)))  # noqa: E731
    goal = IkSolver(chain, valid=free).solve((0.18, 0.13, 0.03), (0, 1, 0), np.array(HOME)).q
    path = plan(np.array(HOME), goal, free, chain.lower + 0.02, chain.upper - 0.02)
    assert path is not None
    assert np.allclose(path[0], HOME) and np.allclose(path[-1], goal)
    for a, b in zip(path, path[1:]):
        assert edge_free(a, b, free, 0.04)
