"""Run an agricultural scenario end to end.

  ros2 launch aibomech_agrobot_tasks scenario.launch.py scenario:=strawberry_harvest
  ros2 launch aibomech_agrobot_tasks scenario.launch.py scenario:=precision_weeding gui:=false
  ros2 launch aibomech_agrobot_tasks scenario.launch.py scenario:=strawberry_harvest hardware:=real

Scenarios: strawberry_harvest | plant_inspection | seedling_transplant | precision_weeding
hardware:  sim (Gazebo, default) | real (robot + RealSense) | mock (no physics, no camera)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = {
    'strawberry_harvest': 'strawberry_greenhouse',
    'plant_inspection': 'strawberry_greenhouse',
    'seedling_transplant': 'nursery_transplanting',
    'precision_weeding': 'weeding_bed',
}


def launch_setup(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    hardware = LaunchConfiguration('hardware').perform(context)
    if scenario not in WORLDS:
        raise RuntimeError(f'unknown scenario {scenario}, choose one of {sorted(WORLDS)}')
    world = WORLDS[scenario]
    tasks = get_package_share_directory('aibomech_agrobot_tasks')
    gazebo = get_package_share_directory('aibomech_agrobot_gazebo')
    sim = hardware == 'sim'

    if sim:
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'sim.launch.py')),
            launch_arguments={'world': world, 'gui': LaunchConfiguration('gui'),
                              'rviz': LaunchConfiguration('rviz')}.items())
    else:
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('aibomech_agrobot_bringup'), 'launch', 'robot.launch.py')),
            launch_arguments={'hardware': hardware, 'rviz': LaunchConfiguration('rviz'),
                              'platform': LaunchConfiguration('platform'),
                              'mount_height': LaunchConfiguration('mount_height'),
                              'serial_port': LaunchConfiguration('serial_port'),
                              'realsense': 'true' if hardware == 'real' else 'false'}.items())

    overrides = {'use_sim_time': sim, 'sim_grasp': sim}
    if sim:
        overrides['sim_objects_file'] = os.path.join(gazebo, 'config', f'{world}_objects.yaml')
    task = Node(package='aibomech_agrobot_tasks', executable=scenario, output='screen',
                parameters=[os.path.join(tasks, 'config', f'{scenario}.yaml'), overrides])
    return [robot, task]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='strawberry_harvest',
                              description=' | '.join(WORLDS)),
        DeclareLaunchArgument('hardware', default_value='sim', description='sim | real | mock'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('platform', default_value='rail_trolley',
                              description='real/mock only: rail_trolley | pedestal | table'),
        DeclareLaunchArgument('mount_height', default_value='0.85', description='real/mock only'),
        OpaqueFunction(function=launch_setup),
    ])
