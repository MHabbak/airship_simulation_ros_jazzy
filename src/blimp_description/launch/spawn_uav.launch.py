#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    
    is_input_joystick = LaunchConfiguration('is_input_joystick')
    enable_meshes = LaunchConfiguration('enable_meshes')
    enable_wind = LaunchConfiguration('enable_wind')
    enable_physics = LaunchConfiguration('enable_physics')
    enable_sensors = LaunchConfiguration('enable_sensors')
    enable_logging = LaunchConfiguration('enable_logging')
    enable_ground_truth = LaunchConfiguration('enable_ground_truth')
    enable_mavlink_interface = LaunchConfiguration('enable_mavlink_interface')
    log_file = LaunchConfiguration('log_file')
    wait_to_record_bag = LaunchConfiguration('wait_to_record_bag')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_blimp_description, 'urdf', 'blimp_base.xacro')
    
    # Robot description from xacro
    robot_description = Command([
        'xacro ', xacro_file,
        ' uav_name:=', uav_name,
        ' namespace:=', namespace,
        ' is_input_joystick:=', is_input_joystick,
        ' enable_meshes:=', enable_meshes,
        ' enable_wind:=', enable_wind,
        ' enable_physics:=', enable_physics,
        ' enable_sensors:=', enable_sensors,
        ' enable_logging:=', enable_logging,
        ' enable_ground_truth:=', enable_ground_truth,
        ' enable_mavlink_interface:=', enable_mavlink_interface,
        ' log_file:=', log_file,
        ' wait_to_record_bag:=', wait_to_record_bag,
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Spawn entity in Gazebo using ros_gz_sim for Gazebo Harmonic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-name', uav_name,
            '-topic', [namespace, '/robot_description'],
            '-x', x,
            '-y', y,
            '-z', z,
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('model', default_value=xacro_file),
        DeclareLaunchArgument('tf_prefix', default_value=''),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
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
        
        # Nodes
        robot_state_publisher,
        spawn_entity,
    ])