#!/usr/bin/env python3
"""Workspace analysis of the real AgroBot arm (URDF model).

1. Reachable workspace: sample joint angles uniformly inside the joint limits,
   compute the TCP position p = FK(q) for each sample (Monte Carlo method, the
   numeric version of notebook 18), and plot top and side views.
2. Dexterity maps: for a grid of TCP targets, solve IK with a required tool
   approach direction and colour each target by the best achievable
   alignment error. This shows *where* the 4-axis arm can grip from above
   (seedlings, weeds) or horizontally (hanging fruit), and is the reason the
   tasks move the rail to put every crop into the arm's sweet spot.

Outputs: analysis/figures/workspace_views.png, analysis/figures/approach_maps.png
         and a short summary on stdout.

    python3 02_workspace.py            (about 2 minutes)
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'agrobot_ws', 'src', 'aibomech_agrobot_tasks'))
from aibomech_agrobot_tasks.kinematics import Chain, IkSolver  # noqa: E402

URDF = os.path.join(HERE, '..', 'data', 'agrobot.urdf')
FIGURES = os.path.join(HERE, '..', 'figures')
SAMPLES = 40000


def reachable_points(chain, samples, seed=0):
    rng = np.random.default_rng(seed)
    q = rng.uniform(chain.lower, chain.upper, (samples, chain.dof))
    return np.array([chain.fk(qi)[:3, 3] for qi in q])


def approach_map(ik, approach, z, xs, ys):
    """Alignment error (deg) of the best IK solution at height z; NaN = unreachable."""
    err = np.full((len(ys), len(xs)), np.nan)
    for i, y in enumerate(ys):
        for j, x in enumerate(xs):
            res = ik.solve((x, y, z), approach)
            if res.position_error < 0.002:
                err[i, j] = np.degrees(res.approach_error)
    return err


def main():
    os.makedirs(FIGURES, exist_ok=True)
    chain = Chain(open(URDF).read(), 'arm_mount', 'tcp')
    pts = reachable_points(chain, SAMPLES)
    radius = np.hypot(pts[:, 0], pts[:, 1])
    print(f'Reach from the J1 axis: max {radius.max():.3f} m, 95th percentile {np.percentile(radius, 95):.3f} m')
    print(f'TCP height range relative to the mounting plate: {pts[:, 2].min():.3f} .. {pts[:, 2].max():.3f} m')

    fig, (top, side) = plt.subplots(1, 2, figsize=(12, 5))
    top.hexbin(pts[:, 0], pts[:, 1], gridsize=45, cmap='Greens', mincnt=1)
    top.set(title='Top view: reachable TCP positions', xlabel='x along the row [m]',
            ylabel='y towards the crops [m]', aspect='equal')
    top.plot(0, 0, 'k+', ms=12)
    top.annotate('J1 axis', (0, 0), (0.03, -0.06))
    side.hexbin(pts[:, 1], pts[:, 2], gridsize=45, cmap='Greens', mincnt=1)
    side.set(title='Side view (y–z)', xlabel='y towards the crops [m]',
             ylabel='z above the mounting plate [m]', aspect='equal')
    side.axhline(0, color='k', lw=0.8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURES, 'workspace_views.png'), dpi=120)
    print('wrote figures/workspace_views.png')

    ik = IkSolver(chain, restarts=8)
    xs = np.arange(-0.25, 0.26, 0.05)
    ys = np.arange(0.10, 0.36, 0.05)
    maps = [('Top-down grasp (seedlings, weeds)\nz = −0.06 m', (0, 0, -1), -0.06),
            ('Horizontal grasp into the row (fruit)\nz = +0.03 m', (0, 1, 0), 0.03)]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for ax, (title, approach, z) in zip(axes, maps):
        err = approach_map(ik, approach, z, xs, ys)
        im = ax.imshow(err, origin='lower', cmap='RdYlGn_r', vmin=0, vmax=45,
                       extent=(xs[0] - 0.025, xs[-1] + 0.025, ys[0] - 0.025, ys[-1] + 0.025))
        for i, y in enumerate(ys):
            for j, x in enumerate(xs):
                ax.text(x, y, '×' if np.isnan(err[i, j]) else f'{err[i, j]:.0f}', ha='center', va='center', fontsize=7)
        ax.set(title=title, xlabel='x along the row [m]', ylabel='y towards the crops [m]')
        reachable = err[~np.isnan(err)]
        good = np.mean(reachable < 10) if reachable.size else 0.0
        print(f'{title.splitlines()[0]}: {100 * good:.0f}% of reachable grid cells within 10° of the wanted approach')
    fig.colorbar(im, ax=axes, label='approach error [deg]  (× = unreachable)', shrink=0.9)
    fig.savefig(os.path.join(FIGURES, 'approach_maps.png'), dpi=120, bbox_inches='tight')
    print('wrote figures/approach_maps.png')


if __name__ == '__main__':
    main()
