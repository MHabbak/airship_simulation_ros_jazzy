#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directory
    pkg_blimp = get_package_share_directory('blimp_description')
    
    # Launch arguments
    uav_name = LaunchConfiguration('uav_name')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')

    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' enable_meshes:=', LaunchConfiguration('enable_meshes'),
            ' enable_wind:=', LaunchConfiguration('enable_wind'),
            ' enable_physics:=', LaunchConfiguration('enable_physics'),
            ' enable_sensors:=', LaunchConfiguration('enable_sensors'),
            ' enable_logging:=', LaunchConfiguration('enable_logging'),
            ' enable_ground_truth:=', LaunchConfiguration('enable_ground_truth'),
            ' enable_mavlink_interface:=', LaunchConfiguration('enable_mavlink_interface'),
            ' enable_custom_plugins:=', LaunchConfiguration('enable_custom_plugins'),
            ' log_file:=', LaunchConfiguration('log_file'),
            ' wait_to_record_bag:=', LaunchConfiguration('wait_to_record_bag'),
            ' uav_name:=', uav_name,
            ' namespace:=', namespace,
            ' is_input_joystick:=', LaunchConfiguration('is_input_joystick'),
        ]),
        value_type=str
    )

    # FIXED: Robot state publisher WITHOUT explicit namespace
    # (inherits /blimp/ namespace from PushRosNamespace in parent)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # REMOVED: namespace=namespace  # This was causing double namespace
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )

    # FIXED: Spawn entity WITHOUT explicit namespace
    # (inherits /blimp/ namespace from PushRosNamespace in parent)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        # REMOVED: namespace=namespace  # This was causing double namespace
        output='screen',
        arguments=[
            '-topic', 'robot_description',  # Will use /blimp/robot_description
            '-name', 'blimp',  # Simple name instead of LaunchConfiguration
            '-x', LaunchConfiguration('X'),
            '-y', LaunchConfiguration('Y'),
            '-z', LaunchConfiguration('Z'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Event handler: spawn entity only after robot_state_publisher starts
    spawn_entity_event = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_entity]
        )
    )
    
    return LaunchDescription([
        # Launch arguments (same as before)
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('model',
            default_value=os.path.join(pkg_blimp, 'urdf', 'blimp_base.xacro')),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='0.3'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='false'),
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        DeclareLaunchArgument('enable_custom_plugins', default_value='true'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        
        # Launch nodes in proper order
        robot_state_publisher,
        spawn_entity_event,
    ])