#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rvizconfig = LaunchConfiguration('rvizconfig')
    uav_name = LaunchConfiguration('uav_name')
    
    # Paths
    xacro_file = os.path.join(pkg_blimp_description, 'urdf', 'blimp_base.xacro')
    rviz_config_file = os.path.join(pkg_blimp_description, 'rviz', 'display.rviz')
    
    # Robot description from xacro - WITH ALL REQUIRED ARGS
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' uav_name:=', uav_name,
            ' namespace:=', uav_name,
            ' enable_meshes:=true',
            ' enable_wind:=false',
            ' enable_physics:=false',
            ' enable_sensors:=false',
            ' enable_logging:=false',
            ' enable_ground_truth:=false',
            ' enable_mavlink_interface:=false',
        ]),
        value_type=str
    )
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('rvizconfig', default_value=rviz_config_file),
        
        # Launch nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])