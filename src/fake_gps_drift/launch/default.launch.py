#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_context import LaunchContext

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='num_robots',
            default_value='1',
            description='Number of robot instances to launch'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) time'
        ),
        OpaqueFunction(function=launch_robot_nodes)
    ])

def launch_robot_nodes(context: LaunchContext, *args, **kwargs):
    try:
        num_robots = int(LaunchConfiguration('num_robots').perform(context))
        if num_robots < 1:
            raise ValueError("num_robots must be >= 1")
    except (KeyError, ValueError) as e:
        print(f"Error: {str(e)} - Using default value 1")
        num_robots = 1
    
    # Convert string to boolean
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ['true', '1', 't', 'y', 'yes']
    
    nodes = []
    
    for robot_id in range(1, num_robots + 1):
        namespace = f'machine_{robot_id}'
        
        group = GroupAction([
            PushRosNamespace(namespace=namespace),
            Node(
                package='fake_gps_drift',
                executable='fake_gps_drift_node',
                name='fake_gps_drift',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,  # Pass as boolean
                    'robot_id': robot_id
                }],
                # REMOVED explicit namespace parameter
                emulate_tty=True
            )
        ])
        
        nodes.append(group)
    
    return nodes    