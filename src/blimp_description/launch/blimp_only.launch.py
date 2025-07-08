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
    pkg_blimp = get_package_share_directory('blimp_description')
    
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace', default=uav_name)
    rvizconfig = LaunchConfiguration('rvizconfig')
    
    # Paths
    rviz_config_file = PathJoinSubstitution([pkg_blimp, 'rviz', 'blimp.rviz'])
    
    # Group for namespaced nodes
    blimp_group = GroupAction([
        PushRosNamespace(namespace),
        
        # Load parameters 
        SetParametersFromFile(os.path.join(pkg_blimp, 'config', 'controller_blimp.yaml')),
        SetParametersFromFile(os.path.join(pkg_blimp, 'config', 'blimp.yaml')),
        
        # Include spawn_uav launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_blimp, 'launch', 'spawn_uav_launch.py')
            ),
            launch_arguments={
                'uav_name': uav_name,
                'namespace': namespace,
                'enable_meshes': LaunchConfiguration('enable_meshes'),
                'enable_wind': LaunchConfiguration('enable_wind'),
                'enable_physics': LaunchConfiguration('enable_physics'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'enable_logging': LaunchConfiguration('enable_logging'),
                'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
                'enable_mavlink_interface': LaunchConfiguration('enable_mavlink_interface'),
                'is_input_joystick': LaunchConfiguration('is_input_joystick'),
                'x': LaunchConfiguration('X'),
                'y': LaunchConfiguration('Y'),
                'z': LaunchConfiguration('Z'),
            }.items()
        ),
        
        # Controller spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'revolute_joint_state_controller',
                'stick_joint_position_controller',
                'botfin_joint_position_controller',
                'topfin_joint_position_controller',
                'leftfin_joint_position_controller',
                'rightfin_joint_position_controller',
            ],
            output='screen',
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),
    ])
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': True}],
    )
    
    # Blimp control node
    blimp_ctrl = Node(
        package='blimp_description',
        executable='gcs_blimp.py',
        name='blimp_ctrl',
        namespace=namespace,
        output='screen',
        arguments=[uav_name],  # passes the UAV name directly
    )
    
    # Put everything into the LaunchDescription
    ld = LaunchDescription()

    # Declare all arguments
    argument_defaults = [
        ('uav_name', 'blimp'),
        ('namespace', ''),
        ('is_input_joystick', 'false'),
        ('enable_meshes', 'true'),
        ('enable_wind', 'true'),
        ('enable_physics', 'true'),
        ('enable_sensors', 'true'),
        ('enable_logging', 'false'),
        ('enable_ground_truth', 'true'),
        ('enable_mavlink_interface', 'false'),
        ('rvizconfig', rviz_config_file.perform(None)),
        ('X', '0.0'),
        ('Y', '0.0'),
        ('Z', '1.0'),
    ]
    for name, default in argument_defaults:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    # Launch everything
    ld.add_action(blimp_group)
    ld.add_action(rviz_node)
    ld.add_action(blimp_ctrl)

    return ld
