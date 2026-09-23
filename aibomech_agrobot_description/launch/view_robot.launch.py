"""Show the robot model in RViz with joint sliders (no controllers)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('aibomech_agrobot_description')
    args = [
        DeclareLaunchArgument('platform', default_value='rail_trolley',
                              description='rail_trolley | pedestal | table'),
        DeclareLaunchArgument('mount_height', default_value='0.85'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Joint slider GUI'),
    ]
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', PathJoinSubstitution([pkg, 'urdf', 'agrobot.urdf.xacro']),
        ' platform:=', LaunchConfiguration('platform'),
        ' mount_height:=', LaunchConfiguration('mount_height'),
        ' use_camera:=', LaunchConfiguration('use_camera'),
    ]), value_type=str)

    return LaunchDescription(args + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'view_robot.rviz'])]),
    ])
