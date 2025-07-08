#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    target_type = LaunchConfiguration('target_type')
    
    # Target node - choose between interactive or moving
    target_node = Node(
        package='blimp_description',
        executable='target_interactive.py',  # or 'target_moving.py'
        name='target',
        output='screen',
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('target_type', default_value='interactive',
                            description='Type of target: interactive or moving'),
        
        # Launch nodes
        target_node,
    ])