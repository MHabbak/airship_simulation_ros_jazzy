#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # This is essentially a wrapper that calls blimp_gcs_launch with wind enabled
    blimp_gcs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_blimp_description, 'launch', 'blimp_gcs_launch.py')
        ),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'roboID': LaunchConfiguration('roboID'),
            'is_input_joystick': LaunchConfiguration('is_input_joystick'),
            'enable_meshes': LaunchConfiguration('enable_meshes'),
            'enable_wind': 'true',  # Force wind to be enabled
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
            'rvizconfig': LaunchConfiguration('rvizconfig'),
            'X': LaunchConfiguration('X'),
            'Y': LaunchConfiguration('Y'),
            'Z': LaunchConfiguration('Z'),
        }.items()
    )
    
    return LaunchDescription([
        # Declare all arguments with same defaults as blimp_gcs
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='true'),  # Default to true for wind version
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='basic.world'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('rvizconfig', 
            default_value=os.path.join(pkg_blimp_description, 'rviz', 'blimp.rviz')),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Include blimp_gcs launch with wind enabled
        blimp_gcs,
    ])