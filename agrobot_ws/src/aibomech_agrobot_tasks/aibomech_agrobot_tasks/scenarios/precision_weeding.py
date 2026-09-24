"""Precision weeding in a raised lettuce bed.

1. Survey the bed along the rail: detect weeds (purple broadleaf) and crop
   plants (lettuce), fuse detections in the world frame.
2. Weeds closer to a crop plant than the protection radius are left alone
   (a real system would use a finer tool there) and reported.
3. Every other weed is gripped at the stem base, pulled out vertically with
   its root and dropped into the bin on the trolley. Lettuce plants are
   obstacles for the motion planner.
"""
import math
import time

import numpy as np

from ..perception import classes_from_params, merge_detections
from ..robot_interface import MotionError
from ..task_base import AgrobotTask, run_task

DOWN = np.array([0.0, 0.0, -1.0])


class PrecisionWeeding(AgrobotTask):
    name = 'precision_weeding'

    def __init__(self):
        super().__init__()
        self.stations = self.param('survey_stations', [0.35, 0.75, 1.15, 1.55, 1.95, 2.35])
        self.classes = classes_from_params(self.node, 'detection', ['weed', 'crop'])
        self.protect_radius = self.param('crop_protection_radius', 0.06)
        self.grasp_offset = self.param('grasp_height_above_detection', 0.03)
        self.stem_width = self.param('stem_width', 0.006)
        self.pre_distance = self.param('pregrasp_distance', 0.05)
        self.pull = np.array([0.0, 0.0, self.param('pull_height', 0.08)])
        self.max_err = math.radians(self.param('max_approach_error_deg', 35.0))
        self.rail_offsets = self.param('rail_offsets', [0.0, 0.05, -0.05, 0.1, -0.1])
        self.crop_size = self.param('crop_obstacle_size', 0.08)
        self.drop_height = self.param('drop_height', 0.07)
        self.max_weeds = self.param('max_weeds', 0)

    def survey(self):
        found, images = [], {}
        for i, x in enumerate(self.stations):
            self.robot.move_rail(x)
            dets, img = self.camera.detect(self.classes)
            found += dets
            images[f'survey_{i}_{x:.2f}.png'] = img
            self.log.info(f'station {x:.2f} m: {sum(d.label == "weed" for d in dets)} weeds, '
                          f'{sum(d.label == "crop" for d in dets)} crop plants')
        merged = merge_detections(found, radius=0.03)
        weeds = sorted((m for m in merged if m['label'] == 'weed'), key=lambda m: m['position'][0])
        crops = [m['position'] for m in merged if m['label'] == 'crop']
        self.summary['detected_weeds'] = len(weeds)
        self.summary['detected_crops'] = len(crops)
        return weeds, crops, images

    def drop_pose(self):
        crate = self.robot.lookup(self.robot.base_frame, 'crate')[:3, 3]
        res = self.robot.solve(crate + [0.0, 0.0, self.drop_height], DOWN)
        if not res.success:
            raise MotionError('weed bin is out of reach')
        return res.q

    def remove(self, weed):
        step = self.step
        target = weed['position'] + [0.0, 0.0, self.grasp_offset]
        plan = self.plan_reach(target, DOWN, self.pre_distance, self.rail_offsets, self.max_err, retreat=self.pull)
        if plan is None:
            return 'unreachable'
        self.robot.move_rail(plan.rail)
        obj = self.sim_object_near(weed['position'], 'weed_', 0.04) if self.robot.sim_grasp else None
        p = self.robot.world_to_base(target)
        step('approach', self.robot.open_gripper)
        step('approach', self.robot.move_joints, plan.q_pre)
        step('approach', self.robot.move_linear, p, DOWN)
        step('grip', self.robot.close_on, self.stem_width)
        self.robot.attach(obj)
        self.robot.detach_from_plant(obj)          # the root comes out with the pull
        step('pull', self.robot.move_linear, p + self.pull, DOWN, 0.03)
        step('dispose', self.robot.move_joints, self.drop_pose())
        step('dispose', self.robot.open_gripper)
        self.robot.release(obj)
        time.sleep(0.4)
        step('return', self.robot.move_joints, self.home)
        return 'removed'

    def execute(self):
        self.robot.open_gripper()
        self.go_home()
        weeds, crops, images = self.survey()
        self.robot.crop_obstacles = [(c + [0, 0, 0.03], np.array([self.crop_size] * 2 + [0.06])) for c in crops]
        if self.max_weeds:
            weeds = weeds[:self.max_weeds]
        for k, weed in enumerate(weeds):
            t0 = time.monotonic()
            p = weed['position']
            nearest = min((np.linalg.norm(p[:2] - c[:2]) for c in crops), default=np.inf)
            if nearest < self.protect_radius:
                outcome = 'protected_zone'
            else:
                try:
                    outcome = self.remove(weed)
                except MotionError as exc:
                    self.log.warning(f'weed {k}: {exc}')
                    outcome = 'motion_error'
                    self.recover()
            dt = time.monotonic() - t0
            self.log.info(f'weed {k} at ({p[0]:.3f}, {p[1]:.3f}): {outcome} in {dt:.1f} s')
            self.rows.append({'weed': k, 'x': round(p[0], 4), 'y': round(p[1], 4), 'z': round(p[2], 4),
                              'distance_to_crop_m': round(float(nearest), 3), 'outcome': outcome,
                              'cycle_time_s': round(dt, 1)})
        self.robot.move_rail(self.stations[0])
        removed = [r for r in self.rows if r['outcome'] == 'removed']
        self.summary['attempted'] = len(self.rows)
        self.summary['removed'] = len(removed)
        self.summary['left_in_protection_zone'] = sum(r['outcome'] == 'protected_zone' for r in self.rows)
        if removed:
            self.summary['mean_cycle_time_s'] = round(float(np.mean([r['cycle_time_s'] for r in removed])), 1)
        if self.robot.sim_grasp:
            self.summary['sim_weeds_in_bin'] = len(self.in_crate('weed_'))
            self.summary['sim_weeds_total'] = sum(1 for t in self.truth.values() if t == 'weed')
        self.write_report(images)


def main():
    raise SystemExit(run_task(PrecisionWeeding))


if __name__ == '__main__':
    main()
