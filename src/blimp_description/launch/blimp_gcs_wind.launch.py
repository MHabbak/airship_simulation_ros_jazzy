#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # EXACT ROS1 equivalent: include blimp_gcs.launch with enable_wind=true
    blimp_gcs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_blimp_description, 'launch', 'blimp_gcs.launch.py')
        ]),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'roboID': LaunchConfiguration('roboID'),
            'is_input_joystick': LaunchConfiguration('is_input_joystick'),
            'enable_meshes': LaunchConfiguration('enable_meshes'),
            'enable_wind': LaunchConfiguration('enable_wind'),  # KEY DIFFERENCE: wind enabled by default
            'enable_physics': LaunchConfiguration('enable_physics'),
            'enable_sensors': LaunchConfiguration('enable_sensors'),
            'enable_logging': LaunchConfiguration('enable_logging'),
            'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
            'enable_mavlink_interface': LaunchConfiguration('enable_mavlink_interface'),
            'world_name': LaunchConfiguration('world_name'),
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'log_file': LaunchConfiguration('log_file'),
            'wait_to_record_bag': LaunchConfiguration('wait_to_record_bag'),
            'verbose': LaunchConfiguration('verbose'),
            'X': LaunchConfiguration('X'),
            'Y': LaunchConfiguration('Y'),
            'Z': LaunchConfiguration('Z'),
        }.items()
    )
    
    return LaunchDescription([
        # Declare arguments (EXACT ROS1 equivalents with KEY DIFFERENCE: enable_wind=true)
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),    # ROS1 original: true
        DeclareLaunchArgument('enable_wind', default_value='true'),      # KEY DIFFERENCE: true (vs false in gcs)
        DeclareLaunchArgument('enable_physics', default_value='true'),   # ROS1 original: true
        DeclareLaunchArgument('enable_sensors', default_value='true'),   # ROS1 original: true
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'), # ROS1 original: true
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'), # ROS1 original: false
        DeclareLaunchArgument('world_name', default_value='basic'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),  # ROS1 original: 1.0
        
        # Include blimp_gcs with wind enabled (EXACT ROS1 wrapper pattern)
        blimp_gcs_launch,
    ])