#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
        
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace')
    
    # Paths to configuration files
    controller_config = os.path.join(pkg_blimp_description, 'resource', 'controller_blimp.yaml')
    blimp_config = os.path.join(pkg_blimp_description, 'resource', 'blimp.yaml')
    rviz_config = os.path.join(pkg_blimp_description, 'rviz', 'blimp.rviz')
        
    # Group for namespaced nodes  
    blimp_group = GroupAction([
        PushRosNamespace(namespace),
        
        # Include spawn_uav launch (handles robot_state_publisher and Gazebo)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_blimp_description, 'launch', 'spawn_uav.launch.py')
            ),
            launch_arguments={
                'uav_name': uav_name,
                'namespace': namespace,
                'model': os.path.join(pkg_blimp_description, 'urdf', 'blimp_base.xacro'),
                'enable_meshes': LaunchConfiguration('enable_meshes'),
                'enable_wind': LaunchConfiguration('enable_wind'),
                'enable_physics': LaunchConfiguration('enable_physics'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'enable_logging': LaunchConfiguration('enable_logging'),
                'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
                'enable_mavlink_interface': LaunchConfiguration('enable_mavlink_interface'),
                'is_input_joystick': LaunchConfiguration('is_input_joystick'),
                'X': LaunchConfiguration('X'),
                'Y': LaunchConfiguration('Y'),
                'Z': LaunchConfiguration('Z'),
            }.items()
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
    
    # Controller spawner - Wait for everything to be ready
    delayed_spawner = TimerAction(
        period=8.0,  # Wait 8 seconds for robot and controller manager to be ready
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='controller_spawner',
                namespace=namespace,
                arguments=[
                    'joint_state_broadcaster',
                    'stick_joint_position_controller',
                    'botfin_joint_position_controller', 
                    'topfin_joint_position_controller',
                    'leftfin_joint_position_controller',
                    'rightfin_joint_position_controller',
                ],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]
    )
    
    # Optional: RViz
    rviz_node = TimerAction(
        period=12.0,  # Start RViz last
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                namespace=namespace,
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_rviz'))
            )
        ]
    )
    
    return LaunchDescription([
        # Declare arguments (same as ROS1 version)
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='false'),  # Start simple
        DeclareLaunchArgument('enable_wind', default_value='false'),    # Start simple
        DeclareLaunchArgument('enable_physics', default_value='false'), # Start simple
        DeclareLaunchArgument('enable_sensors', default_value='false'), # Start simple
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='false'), # Start simple
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),    # Start simple
        DeclareLaunchArgument('world_name', default_value='basic'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Launch sequence with proper timing
        blimp_group,                # Basic robot setup
        delayed_spawner,           # Controller spawning after robot is ready
        rviz_node,                 # RViz last (optional)
    ])