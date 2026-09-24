"""Every task configuration must load as ROS 2 parameters for its node."""
import glob
import os

import pytest
import yaml

CONFIGS = sorted(glob.glob(os.path.join(os.path.dirname(__file__), '..', 'config', '*.yaml')))


@pytest.mark.parametrize('path', CONFIGS, ids=[os.path.basename(p) for p in CONFIGS])
def test_config_is_a_ros_parameter_file(path):
    data = yaml.safe_load(open(path))
    node = os.path.splitext(os.path.basename(path))[0]
    assert list(data) == [node], f'top-level key must be the node name "{node}"'
    params = data[node]['ros__parameters']
    assert isinstance(params, dict) and params, 'ros__parameters must be a non-empty mapping'
