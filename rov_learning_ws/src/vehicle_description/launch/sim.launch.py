#!/usr/bin/env python3
"""
Simulation Master Launch File
-----------------------------
Starts:
  1. Gazebo Harmonic with the custom learning_world.sdf
  2. robot_state_publisher with the processed Xacro vehicle model
  3. ros_gz_sim entity spawner to drop the vehicle into Gazebo
  4. ros_gz_bridge for /vehicle/cmd_vel, /vehicle/odom, /clock, and /tf
  5. vehicle_command_monitor from vehicle_control
  6. (Optional) RViz2 with pre-configured displays

Part of the ROS 2 Jazzy Remote-Controlled Vehicle Learning Project.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_vehicle_description = get_package_share_directory('vehicle_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    default_world_path = os.path.join(pkg_vehicle_description, 'worlds', 'learning_world.sdf')
    xacro_file = os.path.join(pkg_vehicle_description, 'urdf', 'vehicle.urdf.xacro')
    rviz_config_file = os.path.join(pkg_vehicle_description, 'rviz', 'vehicle.rviz')

    # Parse Xacro
    doc = xacro.process_file(xacro_file)
    robot_description_content = doc.toxml()

    # Launch Configurations
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for 3D state visualization'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world model file to load'
    )

    # 1. Robot State Publisher (publishes /robot_description and coordinate transforms)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Harmonic Simulation (direct process execution for fast startup and stability)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
        output='screen'
    )

    # 3. Spawn the custom vehicle into Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'learning_vehicle',
            '-allow_renaming', 'true',
            '-z', '0.12',
            '-x', '0.0',
            '-y', '0.0'
        ]
    )

    # 4. ROS-Gazebo Bridge
    # Connects ROS 2 topics to Gazebo Harmonic Transport topics
    bridge_params = [
        # Velocity command: ROS -> GZ
        '/vehicle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        # Odometry: GZ -> ROS
        '/vehicle/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # Simulation clock: GZ -> ROS
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # Joint states: GZ -> ROS
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        # Transform frames: GZ -> ROS
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_params,
        parameters=[{'use_sim_time': True}]
    )

    # 5. Vehicle Command Monitor (Watchdog)
    node_command_monitor = Node(
        package='vehicle_control',
        executable='vehicle_command_monitor',
        name='vehicle_command_monitor',
        output='screen',
        parameters=[{
            'timeout_seconds': 1.0,
            'check_rate_hz': 5.0,
            'use_sim_time': True
        }]
    )

    # 6. Optional RViz2 Node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        world_arg,
        node_robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        ros_gz_bridge,
        node_command_monitor,
        node_rviz
    ])
