#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    namespace = LaunchConfiguration('namespace', default='')
    
    # Teleop keyboard node
    teleop_node = Node(
        package='blimp_description',
        executable='teleokeyboard.py',
        name='teleokeyboard',
        namespace=namespace,
        output='screen',
        prefix='xterm -e',  # Launch in new terminal window for keyboard input
        remappings=[
            ('cmd_vel', 'cmd_vel'),  # Can be remapped if needed
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('namespace', default_value='',
                            description='Namespace for the teleop node'),
        
        # Launch node
        teleop_node,
    ])