"""AgroBot in Gazebo Sim (Fortress) with ros2_control.

  ros2 launch aibomech_agrobot_gazebo sim.launch.py world:=strawberry_greenhouse
  ros2 launch aibomech_agrobot_gazebo sim.launch.py world:=weeding_bed gui:=false

Worlds: strawberry_greenhouse | nursery_transplanting | weeding_bed
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gazebo = get_package_share_directory('aibomech_agrobot_gazebo')
    description = get_package_share_directory('aibomech_agrobot_description')
    bringup = get_package_share_directory('aibomech_agrobot_bringup')
    controllers = os.path.join(bringup, 'config', 'controllers.yaml')
    use_sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', os.path.join(description, 'urdf', 'agrobot.urdf.xacro'),
        ' hardware:=gz platform:=rail_trolley',
        ' mount_height:=', LaunchConfiguration('mount_height'),
        ' controllers_file:=', controllers,
    ]), value_type=str)

    # Start paused (no -r): sim_manager unpauses once the grasp joints are released.
    gz_args = os.path.join(gazebo, 'worlds', f'{world}.sdf')
    if not gui:
        gz_args = '-s --headless-rendering ' + gz_args
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'gz_version': '6'}.items())

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'agrobot', '-allow_renaming', 'false'])

    sim_manager = Node(package='aibomech_agrobot_gazebo', executable='sim_manager.py', output='screen',
                       parameters=[{'world': world,
                                    'objects_file': os.path.join(gazebo, 'config', f'{world}_objects.yaml')}])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60'])

    return [
        gz,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}, use_sim_time]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gazebo, 'config', f'{world}_bridge.yaml')},
                         use_sim_time]),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[sim_manager])),
        RegisterEventHandler(OnProcessExit(target_action=sim_manager, on_exit=[
            spawner('joint_state_broadcaster'), spawner('arm_controller'),
            spawner('gripper_controller'), spawner('rail_controller')])),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(bringup, 'rviz', 'agrobot.rviz')],
             parameters=[use_sim_time], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share_root = os.path.dirname(get_package_share_directory('aibomech_agrobot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='strawberry_greenhouse',
                              description='strawberry_greenhouse | nursery_transplanting | weeding_bed'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo client window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mount_height', default_value='0.85',
                              description='Must match the height the worlds were generated for'),
        # Lets Gazebo resolve package://aibomech_agrobot_description/meshes/...
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', share_root),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_root),
        OpaqueFunction(function=launch_setup),
    ])
