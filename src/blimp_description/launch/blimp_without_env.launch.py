#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParametersFromFile

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace', default=uav_name)
    rvizconfig = LaunchConfiguration('rvizconfig')
    
    # Paths - FIXED: Use resource/ not config/
    rviz_config_file = PathJoinSubstitution([pkg_blimp_description, 'rviz', 'blimp_mpc.rviz'])
    controller_config = os.path.join(pkg_blimp_description, 'resource', 'controller_blimp.yaml')
    blimp_config = os.path.join(pkg_blimp_description, 'resource', 'blimp.yaml')
    
    # Group for namespaced nodes
    blimp_group = GroupAction([
        PushRosNamespace(namespace),
        
        # Load parameters from resource folder
        SetParametersFromFile(controller_config),
        SetParametersFromFile(blimp_config),
        
        # Include spawn_uav launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_blimp_description, 'launch', 'spawn_uav.launch.py')
            ),
            launch_arguments={
                'uav_name': uav_name,
                'namespace': namespace,
                'model': os.path.join(pkg_blimp_description, 'urdf', 'blimp_base.xacro'),
                'enable_meshes': 'false',
                'enable_wind': 'false',
                'enable_physics': LaunchConfiguration('enable_physics'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'enable_logging': LaunchConfiguration('enable_logging'),
                'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
                'enable_mavlink_interface': 'true',
                'X': LaunchConfiguration('X'),
                'Y': LaunchConfiguration('Y'),
                'Z': LaunchConfiguration('Z'),
            }.items()
        ),
        
        # Controller manager and spawner
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[controller_config]
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                'stick_joint_position_controller',
                'botfin_joint_position_controller',
                'topfin_joint_position_controller',
                'leftfin_joint_position_controller',
                'rightfin_joint_position_controller',
            ],
            output='screen',
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),
        
        # Blimp environment node (if it exists)
        Node(
            package='blimp_description',
            executable='blimp_env.py',
            name='blimp_env',
            output='screen',
        ),
    ])
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': True}],
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='false'),
        DeclareLaunchArgument('enable_wind', default_value='false'),
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='basic'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value=rviz_config_file),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='10.0'),
        
        # Launch nodes
        blimp_group,
        rviz,
    ])