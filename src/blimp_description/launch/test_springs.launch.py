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
    
    # Just reuse blimp_with_env launch with spring test world
    # FIXED: Correct filename reference
    blimp_with_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_blimp_description, 'launch', 'blimp_with_env.launch.py')
        ),
        launch_arguments={
            'world_name': 'spring_damper_test.world',
            'enable_wind': 'true',
            'enable_physics': 'true',
            'uav_name': LaunchConfiguration('uav_name'),
            'X': LaunchConfiguration('X'),
            'Y': LaunchConfiguration('Y'),
            'Z': LaunchConfiguration('Z'),
        }.items()
    )
    
    return LaunchDescription([
        # Declare arguments with spring test defaults
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('world_name', default_value='spring_damper_test.world'),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Include main launch
        blimp_with_env,
    ])