"""Open RViz with the robot model and the full TF tree (no Gazebo needed).

    ros2 launch robocon27_simulation display.launch.py

Every link frame is drawn with its name and axes (red = X forward, green = Y left, blue = Z up).
Use the joint_state_publisher_gui sliders to spin the wheels / due units and watch their frames turn.
    gui:=false   -> plain joint_state_publisher (no sliders)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('robocon27_simulation')
    robot_file = os.path.join(package_share, 'urdf', 'due_wheel.urdf')
    rviz_config = os.path.join(package_share, 'rviz', 'display.rviz')

    with open(robot_file, 'r') as f:
        robot_description = f.read()

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show joint_state_publisher_gui sliders')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([gui_arg, robot_state_publisher, jsp_gui, jsp, rviz])
