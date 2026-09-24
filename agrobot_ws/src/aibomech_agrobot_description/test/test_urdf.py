"""The xacro must expand to a valid URDF for every supported configuration."""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

XACRO = os.path.join(get_package_share_directory('aibomech_agrobot_description'),
                     'urdf', 'agrobot.urdf.xacro')
ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'gripper_jaw_joint']


def expand(*args):
    out = subprocess.run(['xacro', XACRO, 'controllers_file:=/tmp/controllers.yaml', *args],
                         check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


@pytest.mark.parametrize('hardware', ['mock', 'gz', 'real'])
@pytest.mark.parametrize('platform', ['rail_trolley', 'pedestal', 'table'])
def test_expands(hardware, platform):
    root = expand(f'hardware:={hardware}', f'platform:={platform}')
    joints = {j.get('name'): j for j in root.findall('joint')}
    for name in ARM_JOINTS:
        limit = joints[name].find('limit')
        assert float(limit.get('velocity')) > 0.0, f'{name} has no velocity limit'
        assert float(limit.get('effort')) > 0.0, f'{name} has no effort limit'
    assert ('rail_joint' in joints) == (platform == 'rail_trolley')

    controlled = [j.get('name') for j in root.find('ros2_control').findall('joint')]
    assert controlled[:5] == ARM_JOINTS
    assert ('rail_joint' in controlled) == (platform == 'rail_trolley')


def test_links_have_positive_inertia():
    root = expand()
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        assert float(inertial.find('mass').get('value')) > 0.0
        inertia = inertial.find('inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            assert float(inertia.get(axis)) > 0.0, f'{link.get("name")} {axis}'


def test_standard_frames_present():
    names = {link.get('name') for link in expand().findall('link')}
    assert {'base', 'flange', 'tool0', 'tcp', 'camera_color_optical_frame', 'crate'} <= names
