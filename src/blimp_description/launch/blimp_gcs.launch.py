#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, 
    ExecuteProcess, RegisterEventHandler, LogInfo, OpaqueFunction, SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Set up Gazebo resource paths
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':' if EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='') else '',
            pkg_blimp_description
        ]
    )
        
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    
    # Generate robot_description
    robot_description = ParameterValue(
        Command([
            'xacro ', os.path.join(pkg_blimp_description, 'urdf', 'blimp_base.xacro'),
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
    
    # Use empty.sdf
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['empty.sdf -v 4']  # Same as blimp_only
        }.items()
    )

    # PX4 SITL process 
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_none'
        ],
        output='screen',
        shell=True,
    )
        
    # Group for namespaced nodes
    blimp_group = GroupAction([
        PushRosNamespace(namespace),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            # NO explicit namespace - inherits from PushRosNamespace above
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),
        
        # Spawn entity
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_entity',
            # NO explicit namespace - inherits from PushRosNamespace above
            output='screen',
            arguments=[
                '-topic', 'robot_description',  # Will be /blimp/robot_description
                '-name', namespace,
                '-x', LaunchConfiguration('X'),
                '-y', LaunchConfiguration('Y'),
                '-z', LaunchConfiguration('Z'),
            ],
            parameters=[{'use_sim_time': True}],
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            # NO explicit namespace - inherits from PushRosNamespace above
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # Fake GPS Drift Node
        Node(
            package='fake_gps_drift',
            executable='fake_gps_drift_node',
            name='fake_gps_drift_node',
            # NO explicit namespace - inherits from PushRosNamespace above
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # MAVROS Node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14540@127.0.0.1:14557',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'use_sim_time': True
            }],
            remappings=[
                ('/mavros/imu/data', 'tail/imu'),
                ('/mavros/global_position/local', 'tail/position'),
                ('/mavros/global_position/rel_alt', 'ground_speed'),
                ('/mavros/vfr_hud', 'tail/indicatedAirspeed'),
            ]
        ),
        
        # PX4 Blimp Control Node
        Node(
            package='blimp_description',
            executable='px4_blimp_control.py',
            name='px4_blimp_control',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
    
    return LaunchDescription([
        # Set Gazebo resource path first
        gazebo_resource_path,
        
        # Declare arguments with SAME defaults
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='false'),     # CHANGED: enable wind for gcs
        DeclareLaunchArgument('enable_physics', default_value='true'),   # CHANGED: enable physics for gcs
        DeclareLaunchArgument('enable_sensors', default_value='true'),   # CHANGED: enable sensors for gcs
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'), # CHANGED: enable for gcs
        DeclareLaunchArgument('enable_mavlink_interface', default_value='true'), # CHANGED: enable for PX4
        DeclareLaunchArgument('enable_custom_plugins', default_value='true'), # CHANGED: enable for gcs
        DeclareLaunchArgument('enable_rviz', default_value='false'),
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
        
        # Launch Gazebo with custom world
        gazebo,
        
        # Launch PX4 SITL
        px4_sitl,
        
        # Start the blimp group
        blimp_group,
    ])