import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---- EDIT HERE: where the robot's centre should be, and which way it faces ----
# base_link's origin is the geometric centre of the base, so the robot turns about its own centre.
# Yaw 0 = robot front (+X of base_link) points along world +X.
ROBOT_X = -1.0
ROBOT_Y = 1.0
ROBOT_YAW = 1.57         # radians, e.g. 1.57 = 90 deg
# base_link origin is 0.3335 m above the floor (rollers touch the floor); +1 cm so it drops in gently.
ROBOT_Z = 0.3435
# -------------------------------------------------------------------------------


def generate_launch_description():
    package_share = get_package_share_directory('robocon27_simulation')
    gazebo_resource_path = os.path.dirname(package_share)
    gazebo_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(package_share, 'worlds', 'R27.sdf')
    robot_file = os.path.join(package_share, 'urdf', 'due_wheel.urdf')
    rviz_config = os.path.join(package_share, 'rviz', 'sim.rviz')

    with open(robot_file, 'r') as f:
        robot_description = f.read()

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Also open RViz (TF tree + robot model) next to Gazebo')

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(gazebo_resource_path + ':'
               + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
    )

    # 1. Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. ROS <-> Gazebo bridge
    #    '[' = Gazebo -> ROS,  ']' = ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # base motion: the MecanumDrive plugin listens to this Twist
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # feedback from the MecanumDrive / JointStatePublisher plugins
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # the two due units are plain joint-velocity controllers (rad/s)
            '/due_wheel/front_due_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/due_wheel/back_due_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # 3. TF tree: base_link -> wheels / due units from /joint_states
    #    (odom -> base_link comes from the MecanumDrive plugin through /tf)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    # 4. Spawn the robot once Gazebo is up
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_due_wheel',
        arguments=[
            '-world', 'robocon27',
            '-file', robot_file,
            '-name', 'due_wheel',
            '-x', str(ROBOT_X),
            '-y', str(ROBOT_Y),
            '-z', str(ROBOT_Z),
            '-Y', str(ROBOT_YAW),
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        set_gazebo_resource_path,
        gazebo,
        bridge,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        rviz,
    ])
