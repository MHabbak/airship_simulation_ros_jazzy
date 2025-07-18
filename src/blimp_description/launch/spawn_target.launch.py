#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    target_type = LaunchConfiguration('target_type')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # Target node - choose between interactive or moving based on argument
    target_node = Node(
        package='blimp_description',
        executable=PythonExpression([
            '"target_interactive.py" if "', target_type, '" == "interactive" else "target_moving.py"'
        ]),
        name='target',
        output='screen',
    )
    
    # Optional RViz node (was commented in original ROS1)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz)
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('target_type', default_value='interactive',
                            description='Type of target: interactive or moving'),
        DeclareLaunchArgument('enable_rviz', default_value='false',
                            description='Launch RViz for visualization'),
        
        # Launch nodes
        target_node,
        rviz_node,
    ])
