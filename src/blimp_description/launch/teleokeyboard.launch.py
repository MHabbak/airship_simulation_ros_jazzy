#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments (minimal, matching original simplicity)
    namespace = LaunchConfiguration('namespace')
    
    # Teleop keyboard node (matches original ROS1 behavior)
    teleop_node = Node(
        package='blimp_description',
        executable='teleokeyboard.py',
        name='teleokeyboard',
        namespace=namespace,
        output='screen',
        prefix='xterm -e',  # Launch in new terminal window for keyboard input
        # Original script publishes to 'blimp/teleokeyboardcmd' - no remapping needed by default
    )
    
    return LaunchDescription([
        # Declare arguments (keep minimal like original ROS1)
        DeclareLaunchArgument('namespace', default_value='',
                            description='Optional namespace for the teleop node'),
        
        # Launch node
        teleop_node,
    ])
