#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    world_name = LaunchConfiguration('world_name')
    
    # Just reuse blimp_with_env launch with spring test world
    blimp_with_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_blimp_description, 'launch', 'blimp_with_env_launch.py')
        ),
        launch_arguments={
            'world_name': 'spring_damper_test.world',
            'enable_wind': 'true',
            'enable_physics': 'true',
        }.items()
    )
    
    return LaunchDescription([
        # Declare arguments with spring test defaults
        DeclareLaunchArgument('world_name', default_value='spring_damper_test.world'),
        
        # Include main launch
        blimp_with_env,
    ])