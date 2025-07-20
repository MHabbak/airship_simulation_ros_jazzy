#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directory
    pkg_blimp = get_package_share_directory('blimp_description')
    
    # Launch arguments
    namespace = LaunchConfiguration('namespace')

    robot_description = ParameterValue(
        Command([
            'xacro ', os.path.join(pkg_blimp, 'urdf', 'blimp_base.xacro'),
            ' enable_meshes:=false',
            ' enable_wind:=false',
            ' enable_physics:=false',
            ' enable_sensors:=false',
            ' enable_logging:=false',
            ' enable_ground_truth:=false',
            ' enable_mavlink_interface:=false',
            ' uav_name:=blimp',
            ' namespace:=blimp',
            ' is_input_joystick:=false',
        ]),
        value_type=str
    )

    # Start Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': 'empty.sdf -v 4'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Spawn the robot entity with a simple timer delay
    spawn_entity = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to be ready
        actions=[
            LogInfo(msg='Spawning blimp robot...'),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_entity',
                namespace=namespace,
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'blimp',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.5',
                ],
                parameters=[{'use_sim_time': True}],
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='blimp'),
        
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])