#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directory
    pkg_blimp = get_package_share_directory('blimp_description')
    
    # Launch arguments
    uav_name = LaunchConfiguration('uav_name')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    
    # Robot description from xacro
    robot_description = Command([
        'xacro ', model,
        ' uav_name:=', uav_name,  # ← FIXED: Changed from namespace to uav_name
        ' enable_meshes:=', LaunchConfiguration('enable_meshes'),
        ' enable_wind:=', LaunchConfiguration('enable_wind'),
        ' enable_physics:=', LaunchConfiguration('enable_physics'),
        ' enable_sensors:=', LaunchConfiguration('enable_sensors'),
        ' enable_logging:=', LaunchConfiguration('enable_logging'),
        ' enable_ground_truth:=', LaunchConfiguration('enable_ground_truth'),
        ' enable_mavlink_interface:=', LaunchConfiguration('enable_mavlink_interface'),
    ])
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', uav_name,
            '-x', LaunchConfiguration('X'),
            '-y', LaunchConfiguration('Y'),
            '-z', LaunchConfiguration('Z'),
        ],
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('model',
            default_value=os.path.join(pkg_blimp, 'urdf', 'blimp_base.xacro')),
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='false'),
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Set robot description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        
        # Spawn the robot
        spawn_entity,
    ])