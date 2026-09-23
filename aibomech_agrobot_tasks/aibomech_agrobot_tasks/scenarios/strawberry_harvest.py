"""Selective harvesting of ripe strawberries from a table-top gutter.

1. Survey: the trolley drives along the row, the RGB-D camera detects ripe
   (red) and unripe fruit at every station; detections are fused in the world
   frame.
2. For every ripe fruit: choose the rail position that gives the best
   horizontal approach, drive there, look again to refine the fruit position,
   approach, grip, detach the fruit from its peduncle, retreat and drop it in
   the crate on the trolley.
3. Report: detections, attempts, fruit in crate (simulation), cycle times.
"""
import math
import time

import numpy as np

from ..perception import classes_from_params, merge_detections
from ..robot_interface import MotionError
from ..task_base import AgrobotTask, run_task


class StrawberryHarvest(AgrobotTask):
    name = 'strawberry_harvest'

    def __init__(self):
        super().__init__()
        self.stations = self.param('survey_stations', [0.35, 0.75, 1.15, 1.55, 1.95, 2.35])
        self.classes = classes_from_params(self.node, 'detection', ['ripe', 'unripe'])
        self.approach = np.array(self.param('approach', [0.0, 1.0, 0.0]), float)
        self.pre_distance = self.param('pregrasp_distance', 0.06)
        self.retreat = np.array(self.param('retreat_offset', [0.0, -0.06, -0.015]), float)
        self.max_err = math.radians(self.param('max_approach_error_deg', 35.0))
        self.rail_offsets = self.param('rail_offsets', [0.18, -0.18, 0.15, -0.15, 0.21, -0.21])
        self.fruit_width = self.param('fruit_width', 0.019)
        self.drop_height = self.param('drop_height', 0.09)
        self.refine_radius = self.param('refine_radius', 0.03)
        self.max_fruit = self.param('max_fruit', 0)

    def survey(self):
        found = []
        images = {}
        for i, x in enumerate(self.stations):
            self.robot.move_rail(x)
            dets, img = self.camera.detect(self.classes)
            found += dets
            images[f'survey_{i}_{x:.2f}.png'] = img
            self.log.info(f'station {x:.2f} m: {sum(d.label == "ripe" for d in dets)} ripe, '
                          f'{sum(d.label == "unripe" for d in dets)} unripe')
        fruit = merge_detections(found)
        ripe = sorted((f for f in fruit if f['label'] == 'ripe'), key=lambda f: f['position'][0])
        self.summary['detected_ripe'] = len(ripe)
        self.summary['detected_unripe'] = sum(f['label'] == 'unripe' for f in fruit)
        return ripe, images

    def refine(self, expected):
        dets, _ = self.camera.detect(self.classes[:1])
        close = [d for d in dets if np.linalg.norm(d.position - expected) < self.refine_radius]
        if not close:
            return None
        return min(close, key=lambda d: np.linalg.norm(d.position - expected)).position

    def drop_pose(self):
        crate = self.robot.lookup(self.robot.base_frame, 'crate')[:3, 3]
        res = self.robot.solve(crate + [0.0, 0.0, self.drop_height], np.array([0.0, 0.0, -1.0]), self.home)
        if not res.success:
            raise MotionError('crate is out of reach')
        return res.q

    def pick(self, fruit):
        target = fruit['position']
        plan = self.plan_reach(target, self.approach, self.pre_distance, self.rail_offsets, self.max_err)
        if plan is None:
            return 'unreachable'
        self.robot.move_rail(plan.rail)
        refined = self.refine(target)
        if refined is None:
            return 'lost'
        target = refined
        obj = self.sim_object_near(target, 'fruit_') if self.robot.sim_grasp else None

        p_grasp = self.robot.world_to_base(target)
        grasp = self.robot.solve(p_grasp, self.approach, plan.q_grasp, self.max_err)
        pre = self.robot.ik.solve(p_grasp - self.approach * self.pre_distance, self.approach, grasp.q, self.max_err)
        if not (grasp.success and pre.success):
            return 'unreachable'

        self.robot.open_gripper()
        self.robot.move_joints(pre.q)
        self.robot.move_linear(p_grasp, self.approach)
        self.robot.close_on(self.fruit_width)
        self.robot.attach(obj)
        # Real robot: the pull-and-twist below breaks the peduncle.
        self.robot.detach_from_plant(obj)
        self.robot.move_linear(p_grasp + self.retreat, self.approach)
        self.robot.move_joints(self.home)
        self.robot.move_joints(self.drop_pose())
        self.robot.open_gripper()
        self.robot.release(obj)
        time.sleep(0.5)
        self.robot.move_joints(self.home)
        return 'picked'

    def execute(self):
        self.robot.open_gripper()
        self.go_home()
        ripe, images = self.survey()
        if self.max_fruit:
            ripe = ripe[:self.max_fruit]
        for k, fruit in enumerate(ripe):
            t0 = time.monotonic()
            try:
                outcome = self.pick(fruit)
            except MotionError as exc:
                self.log.warning(f'fruit {k}: {exc}')
                outcome = 'motion_error'
                self.recover()
            dt = time.monotonic() - t0
            p = fruit['position']
            self.log.info(f'fruit {k} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}): {outcome} in {dt:.1f} s')
            self.rows.append({'fruit': k, 'x': round(p[0], 4), 'y': round(p[1], 4), 'z': round(p[2], 4),
                              'outcome': outcome, 'cycle_time_s': round(dt, 1)})
        self.robot.move_rail(self.stations[0])
        picked = [r for r in self.rows if r['outcome'] == 'picked']
        self.summary['attempted'] = len(self.rows)
        self.summary['picked'] = len(picked)
        if picked:
            self.summary['mean_cycle_time_s'] = round(float(np.mean([r['cycle_time_s'] for r in picked])), 1)
        if self.robot.sim_grasp:
            in_crate = self.in_crate('fruit_')
            self.summary['sim_fruit_in_crate'] = len(in_crate)
            self.summary['sim_unripe_in_crate'] = sum(
                1 for n in in_crate if self.truth.get(n) == 'strawberry_unripe')
            self.summary['sim_ripe_total'] = sum(1 for t in self.truth.values() if t == 'strawberry_ripe')
        self.write_report(images)


def main():
    raise SystemExit(run_task(StrawberryHarvest))


if __name__ == '__main__':
    main()
