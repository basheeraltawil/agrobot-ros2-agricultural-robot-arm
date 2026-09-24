"""Transplanting seedlings from a plug tray into pots on a potting bench.

Tray and pots are fixtures with taught positions (first cell + pitch), as in
nursery automation. The camera only checks that a cell holds a seedling
before the robot reaches into it.

For every cell: top-down pick of the soil plug, lift, drive the rail to the
target pot, top-down place, release, lift.
"""
import math
import time

import numpy as np

from ..perception import classes_from_params
from ..robot_interface import MotionError
from ..task_base import PRE_EXTRA_TILT, AgrobotTask, run_task

DOWN = np.array([0.0, 0.0, -1.0])


def grid(first, pitch, cols, rows, z):
    return [np.array([first[0] + i * pitch[0], first[1] + j * pitch[1], z])
            for i in range(cols) for j in range(rows)]


class SeedlingTransplant(AgrobotTask):
    name = 'seedling_transplant'

    def __init__(self):
        super().__init__()
        self.cells = grid(self.param('tray.first_cell', [0.45, 0.29]), self.param('tray.pitch', [0.06, 0.06]),
                          self.param('tray.cols', 4), self.param('tray.rows', 2), self.param('tray.grasp_z', 0.792))
        self.pots = grid(self.param('pots.first_pot', [1.10, 0.29]), self.param('pots.pitch', [0.10, 0.075]),
                         self.param('pots.cols', 4), self.param('pots.rows', 2), self.param('pots.place_z', 0.828))
        self.classes = classes_from_params(self.node, 'detection', ['seedling'])
        self.check_occupancy = self.param('check_occupancy', True)
        self.plug_width = self.param('plug_width', 0.018)
        self.pre_distance = self.param('pregrasp_distance', 0.05)
        self.lift = np.array([0.0, 0.0, self.param('lift_height', 0.06)])
        self.max_err = math.radians(self.param('max_approach_error_deg', 35.0))
        self.rail_offsets = self.param('rail_offsets', [0.0, 0.05, -0.05, 0.1, -0.1])
        self.max_plants = self.param('max_plants', 0)

    def occupied(self, cell):
        dets, _ = self.camera.detect(self.classes)
        return any(np.linalg.norm(d.position[:2] - cell[:2]) < 0.03 for d in dets)

    def reach(self, target):
        plan = self.plan_reach(target, DOWN, self.pre_distance, self.rail_offsets, self.max_err, retreat=self.lift)
        if plan is None:
            return None
        self.robot.move_rail(plan.rail)
        p = self.robot.world_to_base(target)
        grasp = self.robot.solve(p, DOWN, plan.q_grasp, self.max_err)
        pre = self.robot.ik.solve(p + [0, 0, self.pre_distance], DOWN, grasp.q, self.max_err + PRE_EXTRA_TILT)
        return (p, pre.q) if grasp.success and pre.success else None

    def transplant(self, cell, pot):
        step = self.step
        reach = self.reach(cell)
        if reach is None:
            return 'pick_unreachable'
        if self.check_occupancy and not self.occupied(cell):
            return 'empty_cell'
        obj = self.sim_object_near(cell, 'seedling_', 0.04) if self.robot.sim_grasp else None
        p, q_pre = reach
        step('pick', self.robot.open_gripper)
        step('pick', self.robot.move_joints, q_pre)
        step('pick', self.robot.move_linear, p, DOWN)
        step('pick', self.robot.close_on, self.plug_width)
        self.robot.attach(obj)
        step('pick', self.robot.move_linear, p + self.lift, DOWN)

        reach = self.reach(pot)
        if reach is None:
            step('abort', self.robot.move_linear, p, DOWN)   # put it back
            step('abort', self.robot.open_gripper)
            self.robot.release(obj)
            return 'place_unreachable'
        p, q_pre = reach
        step('place', self.robot.move_joints, q_pre)
        step('place', self.robot.move_linear, p, DOWN)
        step('place', self.robot.open_gripper)
        self.robot.release(obj)
        time.sleep(0.3)
        step('place', self.robot.move_linear, p + self.lift, DOWN)
        return 'transplanted'

    def execute(self):
        self.robot.open_gripper()
        self.go_home()
        pairs = list(zip(self.cells, self.pots))
        if self.max_plants:
            pairs = pairs[:self.max_plants]
        for k, (cell, pot) in enumerate(pairs):
            t0 = time.monotonic()
            try:
                outcome = self.transplant(cell, pot)
            except MotionError as exc:
                self.log.warning(f'seedling {k}: {exc}')
                outcome = 'motion_error'
                self.recover()
            dt = time.monotonic() - t0
            self.log.info(f'seedling {k}: {outcome} in {dt:.1f} s')
            self.rows.append({'seedling': k, 'cell_x': round(cell[0], 3), 'cell_y': round(cell[1], 3),
                              'pot_x': round(pot[0], 3), 'pot_y': round(pot[1], 3),
                              'outcome': outcome, 'cycle_time_s': round(dt, 1)})
        self.go_home()
        done = [r for r in self.rows if r['outcome'] == 'transplanted']
        self.summary['attempted'] = len(self.rows)
        self.summary['transplanted'] = len(done)
        if done:
            self.summary['mean_cycle_time_s'] = round(float(np.mean([r['cycle_time_s'] for r in done])), 1)
        if self.robot.sim_grasp:
            potted = 0
            for pos in self.sim_poses('seedling_').values():
                if any(np.linalg.norm(pos[:2] - pot[:2]) < 0.025 and pos[2] > pot[2] - 0.06 for pot in self.pots):
                    potted += 1
            self.summary['sim_seedlings_in_pots'] = potted
        self.write_report()


def main():
    raise SystemExit(run_task(SeedlingTransplant))


if __name__ == '__main__':
    main()
