#!/usr/bin/env python3
"""Short design calculations behind the robot cell.

1. Mass budget of the arm (CAD parts + servos, from arm_physical.yaml).
2. Gripper: jaw gap as a function of the jaw joint position.
3. Camera: area covered and resolution on the crop row.
4. Rail: travel time between survey stations.

    python3 05_design_calculations.py
"""
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DESCRIPTION = os.path.join(HERE, '..', '..', 'agrobot_ws', 'src', 'aibomech_agrobot_description', 'config')

# Values taken from the robot description and task configuration.
JAW_GAP_OFFSET = 0.0174            # m, gap when the jaw joint is at 0 (fixed jaw face to moving jaw face)
JAW_RANGE = (-0.017, 0.010)        # m, jaw joint limits
CAMERA_HFOV = math.radians(69.0)   # RealSense D435 colour, 640 x 480 used
CAMERA_PIXELS = (640, 480)
CAMERA_TO_FRUIT = 0.66             # m, camera (mast) to the fruit zone, from the URDF poses
FRUIT_DIAMETER = 0.019             # m
RAIL_SPEED, RAIL_STATION_PITCH = 0.15, 0.40   # m/s (task setting), m between survey stations


def main():
    phys = yaml.safe_load(open(os.path.join(DESCRIPTION, 'arm_physical.yaml')))
    print('1. Mass budget (CAD parts ×1.10 for fasteners and cables, plus one servo per joint)\n')
    print('| link | mass [kg] |')
    print('|---|---|')
    for link, p in phys.items():
        print(f'| {link} | {p["mass"]:.3f} |')
    moving = sum(p['mass'] for k, p in phys.items() if k != 'base_link')
    print(f'| **moving arm (link_1…link_5)** | **{moving:.3f}** |')

    print('\n2. Gripper: gap = jaw position + offset')
    for q in (JAW_RANGE[1], 0.0026, 0.0):
        print(f'   jaw {q * 1000:+5.1f} mm  →  gap {(q + JAW_GAP_OFFSET) * 1000:4.1f} mm')
    print(f'   fully open: {1000 * (JAW_RANGE[1] + JAW_GAP_OFFSET):.1f} mm, so objects up to about '
          f'{1000 * (JAW_RANGE[1] + JAW_GAP_OFFSET - 0.004):.0f} mm fit with 2 mm clearance per side')

    width = 2 * CAMERA_TO_FRUIT * math.tan(CAMERA_HFOV / 2)
    mm_per_px = 1000 * width / CAMERA_PIXELS[0]
    print(f'\n3. Camera at {CAMERA_TO_FRUIT:.2f} m: covers {width:.2f} m of the row, '
          f'{mm_per_px:.1f} mm per pixel,')
    print(f'   a {FRUIT_DIAMETER * 1000:.0f} mm fruit is about {FRUIT_DIAMETER * 1000 / mm_per_px:.0f} px across '
          f'(blob area ≈ {math.pi / 4 * (FRUIT_DIAMETER * 1000 / mm_per_px) ** 2:.0f} px, detection minimum is 25 px)')
    print(f'   with {RAIL_STATION_PITCH:.2f} m between stations the views overlap by '
          f'{100 * (1 - RAIL_STATION_PITCH / width):.0f}%')

    # Quintic profile: T = 1.875 · distance / v_peak
    t_station = 1.875 * RAIL_STATION_PITCH / RAIL_SPEED
    print(f'\n4. Rail: {RAIL_STATION_PITCH:.2f} m between stations at {RAIL_SPEED} m/s peak → '
          f'{t_station:.1f} s per move, a 3 m row surveyed in about '
          f'{t_station * 3 / RAIL_STATION_PITCH / 60:.1f} min')


if __name__ == '__main__':
    main()
