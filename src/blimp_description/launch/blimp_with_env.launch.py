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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace', default=uav_name)
    world_name = LaunchConfiguration('world_name')
    
    # Paths - FIXED: Use resource/ not config/
    world_file = PathJoinSubstitution([pkg_blimp_description, 'worlds', world_name])
    controller_config = os.path.join(pkg_blimp_description, 'resource', 'controller_blimp.yaml')
    blimp_config = os.path.join(pkg_blimp_description, 'resource', 'blimp.yaml')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Group for namespaced blimp nodes
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
                'enable_meshes': LaunchConfiguration('enable_meshes'),
                'enable_wind': LaunchConfiguration('enable_wind'),
                'enable_physics': LaunchConfiguration('enable_physics'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'enable_logging': LaunchConfiguration('enable_logging'),
                'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
                'enable_mavlink_interface': LaunchConfiguration('enable_mavlink_interface'),
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
    ])
    
    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            f'/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            f'/{namespace}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/{namespace}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            f'/{namespace}/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            f'/{namespace}/wind_speed@geometry_msgs/msg/Vector3@gz.msgs.Vector3d',
            f'/{namespace}/wind_force@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),
        DeclareLaunchArgument('enable_wind', default_value='false'),
        DeclareLaunchArgument('enable_physics', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='basic.world'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Launch components
        gazebo,
        blimp_group,
        bridge,
    ])