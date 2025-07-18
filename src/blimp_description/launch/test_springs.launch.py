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
    
    # Reuse blimp_with_env launch with spring test defaults
    blimp_with_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_blimp_description, 'launch', 'blimp_with_env.launch.py')
        ),
        launch_arguments={
            # Core arguments (forward user choices)
            'uav_name': LaunchConfiguration('uav_name'),
            'roboID': LaunchConfiguration('roboID'),
            'namespace': LaunchConfiguration('namespace'),
            'is_input_joystick': LaunchConfiguration('is_input_joystick'),
            
            # Enable flags (forward user choices)
            'enable_meshes': LaunchConfiguration('enable_meshes'),
            'enable_wind': LaunchConfiguration('enable_wind'),  # Default true for springs test
            'enable_physics': LaunchConfiguration('enable_physics'),
            'enable_sensors': LaunchConfiguration('enable_sensors'),
            'enable_logging': LaunchConfiguration('enable_logging'),
            'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
            'enable_mavlink_interface': LaunchConfiguration('enable_mavlink_interface'),
            
            # Gazebo arguments (forward user choices)
            'world_name': LaunchConfiguration('world_name'),  # Default spring_damper_test.world
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'verbose': LaunchConfiguration('verbose'),
            
            # Logging arguments
            'log_file': LaunchConfiguration('log_file'),
            'wait_to_record_bag': LaunchConfiguration('wait_to_record_bag'),
            
            # RViz config
            'rvizconfig': LaunchConfiguration('rvizconfig'),
            
            # Position arguments
            'X': LaunchConfiguration('X'),
            'Y': LaunchConfiguration('Y'),
            'Z': LaunchConfiguration('Z'),
        }.items()
    )
    
    return LaunchDescription([
        # Declare arguments with spring test defaults (matching original ROS1)
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        
        # Enable flags with spring test defaults
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='true'),  # KEY DIFFERENCE: wind enabled by default
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        
        # Gazebo arguments
        DeclareLaunchArgument('world_name', default_value='spring_damper_test.world'),  # KEY DIFFERENCE: spring test world
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        
        # Logging arguments
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        
        # RViz config (same as original)
        DeclareLaunchArgument('rvizconfig', 
            default_value=os.path.join(pkg_blimp_description, 'rviz', 'blimp.rviz')),
        
        # Position arguments (same as original)
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Include main launch with all arguments forwarded
        blimp_with_env,
    ])
