"""Bring up the robot with ros2_control on mock or real hardware.

  ros2 launch aibomech_agrobot_bringup robot.launch.py hardware:=mock
  ros2 launch aibomech_agrobot_bringup robot.launch.py hardware:=real serial_port:=/dev/ttyACM0 platform:=table

For Gazebo use aibomech_agrobot_gazebo/sim.launch.py instead.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = FindPackageShare('aibomech_agrobot_description')
    bringup = FindPackageShare('aibomech_agrobot_bringup')
    cfg = LaunchConfiguration

    args = [
        DeclareLaunchArgument('hardware', default_value='mock', description='mock | real'),
        DeclareLaunchArgument('platform', default_value='rail_trolley',
                              description='rail_trolley | pedestal | table'),
        DeclareLaunchArgument('mount_height', default_value='0.85'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('calibration_file', default_value=PathJoinSubstitution(
            [description, 'config', 'hardware_calibration.yaml'])),
        DeclareLaunchArgument('controllers_file', default_value=PathJoinSubstitution(
            [bringup, 'config', 'controllers.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('realsense', default_value='false',
                              description='Start the realsense2_camera driver (real hardware)'),
    ]

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([description, 'urdf', 'agrobot.urdf.xacro']),
        ' hardware:=', cfg('hardware'),
        ' platform:=', cfg('platform'),
        ' mount_height:=', cfg('mount_height'),
        ' use_camera:=', cfg('use_camera'),
        ' serial_port:=', cfg('serial_port'),
        ' baud_rate:=', cfg('baud_rate'),
        ' calibration_file:=', cfg('calibration_file'),
    ]), value_type=str)

    use_rail = PythonExpression(["'", cfg('platform'), "' == 'rail_trolley'"])

    def spawner(name, condition=None):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'],
                    condition=condition, output='screen')

    return LaunchDescription(args + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[cfg('controllers_file')],
             remappings=[('~/robot_description', '/robot_description')],
             output='screen'),
        spawner('joint_state_broadcaster'),
        spawner('arm_controller'),
        spawner('gripper_controller'),
        spawner('rail_controller', IfCondition(use_rail)),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'agrobot.rviz'])],
             condition=IfCondition(cfg('rviz'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={'camera_namespace': '', 'camera_name': 'camera', 'align_depth.enable': 'true',
                              'pointcloud.enable': 'true', 'publish_tf': 'false'}.items(),
            condition=IfCondition(cfg('realsense'))),
    ])
