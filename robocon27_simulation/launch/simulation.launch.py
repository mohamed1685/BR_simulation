import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    # Installed package directories
    package_share = get_package_share_directory(
        'robocon27_simulation'
    )
    gazebo_resource_path = os.path.dirname(package_share)

    gazebo_share = get_package_share_directory(
        'ros_gz_sim'
    )

    # Installed world and robot description
    world_file = os.path.join(
        package_share,
        'worlds',
        'R27.sdf'
    )

    robot_file = os.path.join(
        package_share,
        'urdf',
        'due_wheel.urdf'
    )
    set_gazebo_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=(
        gazebo_resource_path
        + ':'
        + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ),
)
    
    # 1. Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # 2. Bridge ROS 2 wheel commands to Gazebo Transport
    wheel_topics = [
        '/due_wheel/front_due_joint/cmd_vel',
        '/due_wheel/back_due_joint/cmd_vel',
        '/due_wheel/back_normal_joint/cmd_vel',
        '/due_wheel/front_normal_joint/cmd_vel',
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheel_command_bridge',
        arguments=[
            f'{topic}@std_msgs/msg/Float64]gz.msgs.Double'
            for topic in wheel_topics
        ],
        output='screen',
    )

    # 3. Spawn the robot after Gazebo has had time to initialize
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_due_wheel',
        arguments=[
            '-world', 'robocon27',
            '-file', robot_file,
            '-name', 'due_wheel',
            '-x', '-1.5',
            '-y', '1.5',
            '-z', '-0.955',
        ],
        output='screen',
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        gazebo,
        bridge,
        TimerAction(
            period=5.0,
            actions=[spawn_robot],
        ),
    ])