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
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace', default=uav_name)
    world_name = LaunchConfiguration('world_name')
    rvizconfig = LaunchConfiguration('rvizconfig')
    
    # Paths
    world_file = PathJoinSubstitution([pkg_blimp, 'worlds', world_name])
    rviz_config_file = PathJoinSubstitution([pkg_blimp, 'rviz', 'blimp.rviz'])

    # Gazebo launch inclusion
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Blimp group for namespaced processes
    blimp_group = GroupAction([
        PushRosNamespace(namespace),

        SetParametersFromFile(os.path.join(pkg_blimp, 'config', 'controller_blimp.yaml')),
        SetParametersFromFile(os.path.join(pkg_blimp, 'config', 'blimp.yaml')),

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
                'x': LaunchConfiguration('X'),
                'y': LaunchConfiguration('Y'),
                'z': LaunchConfiguration('Z'),
            }.items()
        ),

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

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='blimp_description',
            executable='gcs_blimp.py',
            name='blimp_ctrl',
            output='screen',
            # pass namespace value directly as argument
            arguments=[namespace],
        ),
    ])

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': True}],
    )

    # Build Description
    ld = LaunchDescription()

    # Declare Arguments
    for arg_name, default in [
        ('uav_name', 'blimp'),
        ('namespace', ''),           # optional override
        ('enable_meshes', 'true'),
        ('enable_wind', 'false'),
        ('enable_physics', 'true'),
        ('enable_sensors', 'true'),
        ('enable_logging', 'false'),
        ('enable_ground_truth', 'true'),
        ('enable_mavlink_interface', 'false'),
        ('world_name', 'basic.world'),
        ('X', '0.0'),
        ('Y', '0.0'),
        ('Z', '1.0'),
        ('rvizconfig', rviz_config_file.perform(None)),
    ]:
        ld.add_action(DeclareLaunchArgument(arg_name, default_value=default))

    # Compose actions
    ld.add_action(gazebo)
    ld.add_action(blimp_group)
    ld.add_action(rviz)

    return ld
