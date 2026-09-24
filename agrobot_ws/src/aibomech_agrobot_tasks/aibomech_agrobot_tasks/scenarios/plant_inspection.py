"""Plant inspection / phenotyping along a strawberry row.

The trolley stops in front of every plant, the RGB-D camera measures:
  * ripe and unripe fruit count, ripeness ratio
  * fruit size (equivalent diameter from blob area and depth)
  * canopy cover: share of leaf pixels in the plant's image window
and writes a CSV per plant, annotated images and a yield estimate.
The arm stays folded; this is the non-contact task a robot would run daily.
"""
import cv2
import numpy as np

from ..perception import classes_from_params, segment
from ..task_base import AgrobotTask, run_task


class PlantInspection(AgrobotTask):
    name = 'plant_inspection'

    def __init__(self):
        super().__init__()
        self.plants = self.param('plant_positions', [0.40, 0.80, 1.20, 1.60, 2.00, 2.40])
        self.window = self.param('plant_window', 0.20)          # m along the row per plant
        self.fruit_mass = self.param('mean_fruit_mass_kg', 0.015)
        self.classes = classes_from_params(self.node, 'detection', ['ripe', 'unripe'])
        self.leaf = classes_from_params(self.node, 'detection', ['leaf'])[0]

    def execute(self):
        self.go_home()
        images = {}
        for k, x in enumerate(self.plants):
            self.robot.move_rail(x)
            dets, img = self.camera.detect(self.classes)
            mine = [d for d in dets if abs(d.position[0] - x) < self.window / 2]
            ripe = [d for d in mine if d.label == 'ripe']
            unripe = [d for d in mine if d.label == 'unripe']
            leaf_mask = segment(img, self.leaf)
            h, w = leaf_mask.shape
            roi = leaf_mask[:, int(w * 0.25):int(w * 0.75)]
            cover = float(np.count_nonzero(roi)) / roi.size
            sizes = [d.extent * 1000 for d in mine]
            cv2.putText(img, f'plant {k}: {len(ripe)} ripe, {len(unripe)} unripe, canopy {cover:.0%}',
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            images[f'plant_{k}.png'] = img
            row = {'plant': k, 'rail_x': x, 'ripe': len(ripe), 'unripe': len(unripe),
                   'ripeness_ratio': round(len(ripe) / max(1, len(mine)), 2),
                   'mean_fruit_diameter_mm': round(float(np.mean(sizes)), 1) if sizes else 0.0,
                   'canopy_cover': round(cover, 3)}
            self.rows.append(row)
            self.log.info(f'plant {k} at {x:.2f} m: {row}')
        self.robot.move_rail(self.plants[0])
        ripe_total = sum(r['ripe'] for r in self.rows)
        self.summary['plants'] = len(self.rows)
        self.summary['ripe_fruit'] = ripe_total
        self.summary['unripe_fruit'] = sum(r['unripe'] for r in self.rows)
        self.summary['harvest_ready_kg'] = round(ripe_total * self.fruit_mass, 3)
        if self.truth:
            self.summary['sim_true_ripe'] = sum(1 for t in self.truth.values() if t == 'strawberry_ripe')
            self.summary['sim_true_unripe'] = sum(1 for t in self.truth.values() if t == 'strawberry_unripe')
        self.write_report(images)


def main():
    raise SystemExit(run_task(PlantInspection))


if __name__ == '__main__':
    main()
