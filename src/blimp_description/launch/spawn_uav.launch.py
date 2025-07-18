#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            ' log_file:=', LaunchConfiguration('log_file'),
            ' wait_to_record_bag:=', LaunchConfiguration('wait_to_record_bag'),
            ' uav_name:=', uav_name,
            ' namespace:=', namespace,
            ' is_input_joystick:=', LaunchConfiguration('is_input_joystick'),
        ]),
        value_type=str
    )

    # Start Gazebo using the proper ROS2 ros_gz_sim launch integration
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': 'empty.sdf -v 4'  # Start empty world with verbose output
        }.items()
    )

    # Robot state publisher - FIXED: Proper topic remapping
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        # FIXED: Proper topic remapping to ensure controller manager can find it
        remappings=[
            ('/robot_description', f'/{namespace}/robot_description'),
        ]
    )
    
    # Spawn entity - FIXED: Removed problematic respawn and on_exit
    spawn_entity = TimerAction(
        period=3.0,  # Wait for Gazebo to be ready
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_entity',
                namespace=namespace,
                output='screen',
                arguments=[
                    '-topic', f'/{namespace}/robot_description',
                    '-name', namespace,
                    '-x', LaunchConfiguration('X'),
                    '-y', LaunchConfiguration('Y'),
                    '-z', LaunchConfiguration('Z'),
                ],
                # FIXED: Single spawn attempt (remove respawn for now)
                # respawn=True,
                # respawn_delay=2.0,
                parameters=[{'use_sim_time': True}],
                # REMOVED: on_exit parameter that was causing launch exception
            )
        ]
    )
    
    return LaunchDescription([
        # Declare arguments - COMPLETE SET from original ROS1
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('model',
            default_value=os.path.join(pkg_blimp, 'urdf', 'blimp_base.xacro')),
        # Position arguments
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='0.3'),
        # All enable flags
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='false'),
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        
        # Launch sequence (proper ROS2 Gazebo Harmonic integration)
        gazebo,                     # Start Gazebo using ros_gz_sim launch
        robot_state_publisher,     # Start robot_state_publisher immediately  
        spawn_entity,               # Spawn robot (fixed - no more launch exception)
    ])