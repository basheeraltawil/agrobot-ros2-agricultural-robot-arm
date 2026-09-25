#!/usr/bin/env python3
"""Actuator sizing: joint torques of the real arm versus the servo ratings.

Uses robot_model.py (URDF masses and inertias, including the servos) and the
full equation of motion

    τ = M(q) q̈ + C(q, q̇) q̇ + G(q)

1. Static load: the gravity torque G(q) over the whole joint space (Monte
   Carlo), with and without a 50 g payload in the gripper (a large fruit or a
   soil block with its seedling).
2. Dynamic load: a fast joint move with a quintic time profile at the
   velocity limits of joint_limits.yaml, evaluated along the path.
3. Safety factor = rated effort / required torque, per joint.

Output: analysis/figures/torque_sizing.png and a table on stdout.

    python3 04_torque_sizing.py
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from robot_model import ArmModel  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, '..', 'data', 'agrobot.urdf')
FIGURES = os.path.join(HERE, '..', 'figures')
PAYLOAD = 0.05     # kg in the gripper
SAMPLES = 3000
LX16A = 15 * 9.81 / 100   # N·m, the 15 kg·cm servo of the prototype


def with_payload(model, mass):
    """Add a point mass at the TCP by lumping it into link_4 (the gripper body)."""
    link = model.links['link_4']
    tcp = model.joints['tcp'].origin[:3, 3]           # TCP position in link_4
    total = link.mass + mass
    com = (link.mass * link.com + mass * tcp) / total
    d_old, d_new = link.com - com, tcp - com
    parallel = lambda m, d: m * (d @ d * np.eye(3) - np.outer(d, d))  # noqa: E731
    link.inertia = link.inertia + parallel(link.mass, d_old) + parallel(mass, d_new)
    link.mass, link.com = total, com
    return model


def quintic_move(q0, q1, vmax):
    """Joint trajectory with zero start/end velocity and acceleration.

    s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ reaches 1.875 × the mean velocity at τ = 0.5,
    so the duration T = 1.875 · max|Δq| / v_max keeps every joint within its limit.
    """
    dq = q1 - q0
    duration = 1.875 * np.max(np.abs(dq) / vmax)
    for t in np.linspace(0, duration, 200):
        x = t / duration
        s = 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5
        sd = (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / duration
        sdd = (60 * x - 180 * x ** 2 + 120 * x ** 3) / duration ** 2
        yield q0 + s * dq, sd * dq, sdd * dq



def main():
    os.makedirs(FIGURES, exist_ok=True)
    empty = ArmModel.from_urdf(URDF)
    loaded = with_payload(ArmModel.from_urdf(URDF), PAYLOAD)
    rated = np.array([j.effort for j in empty.arm_joints])
    vmax = np.array([j.velocity for j in empty.arm_joints])
    rng = np.random.default_rng(1)
    qs = rng.uniform(empty.lower, empty.upper, (SAMPLES, 4))

    g_empty = np.max([np.abs(empty.gravity_torque(q)) for q in qs], axis=0)
    g_loaded = np.max([np.abs(loaded.gravity_torque(q)) for q in qs], axis=0)

    # Fast move between two far-apart poses (home to an extended reach pose).
    home = np.array([-0.97, -2.0, -1.12, 1.46])
    reach = np.array([1.0, 0.8, 0.9, -0.8])
    dynamic = np.max([np.abs(loaded.inverse_dynamics(q, qd, qdd))
                      for q, qd, qdd in quintic_move(home, reach, vmax)], axis=0)

    required = np.maximum(g_loaded, dynamic)
    print(f'Torques in N·m (payload {PAYLOAD * 1000:.0f} g, {SAMPLES} random poses)\n')
    print('| joint | gravity, empty | gravity, with payload | fast move (M q̈ + C q̇ + G) | rated | safety factor |')
    print('|---|---|---|---|---|---|')
    for k, name in enumerate(empty.joint_names):
        print(f'| {name} | {g_empty[k]:.3f} | {g_loaded[k]:.3f} | {dynamic[k]:.3f} | {rated[k]:.1f} | '
              f'{rated[k] / max(required[k], 1e-6):.0f}× |')
    print('\nJoints 1 and 2 turn about vertical axes, so gravity does not load them;')
    print('their torque comes only from accelerating the arm.')
    # The 3D-printed prototype of the paper used LX-16A bus servos (15 kg·cm) on
    # every joint and showed vibration at joints 1 and 3.
    print(f'\nPrototype servo LX-16A ({LX16A:.2f} N·m) safety factors: '
          + ', '.join(f'{n} {LX16A / max(r, 1e-6):.1f}×' for n, r in zip(empty.joint_names, required)))

    fig, ax = plt.subplots(figsize=(8, 4.2))
    x = np.arange(4)
    ax.bar(x - 0.27, g_empty, 0.27, label='gravity, empty gripper')
    ax.bar(x, g_loaded, 0.27, label=f'gravity, {PAYLOAD * 1000:.0f} g payload')
    ax.bar(x + 0.27, dynamic, 0.27, label='fast move, with payload')
    ax.scatter(x, rated, marker='_', s=900, color='k', label='servo rating', zorder=3)
    ax.set_yscale('log')
    ax.set_xticks(x, empty.joint_names)
    ax.set_ylabel('torque [N·m] (log scale)')
    ax.set_title('Required joint torque vs. actuator rating')
    ax.legend(fontsize=8, loc='lower left')
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURES, 'torque_sizing.png'), dpi=120)
    print('\nwrote figures/torque_sizing.png')


if __name__ == '__main__':
    main()
