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
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def load_yaml_params(file_path):
    """Load YAML parameter file and return as dict"""
    with open(file_path, 'r') as file:
        return yaml.safe_load(file) or {}

def generate_launch_description():
    # Package Directories
    pkg_blimp_description = get_package_share_directory('blimp_description')
    
    # ADDED: Set up Gazebo resource paths (ROS1 equivalent: GAZEBO_MODEL_PATH env vars)
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':' if EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='') else '',
            pkg_blimp_description,
        ]
    )
    
    # Load controller and blimp parameters (ROS1 equivalent: <rosparam command="load">)
    controller_params_file = os.path.join(pkg_blimp_description, 'resource', 'controller_blimp.yaml')
    blimp_params_file = os.path.join(pkg_blimp_description, 'resource', 'blimp.yaml')
    
    controller_params = load_yaml_params(controller_params_file)
    blimp_params = load_yaml_params(blimp_params_file)
    
    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace')
    
    # Generate robot_description (equivalent to spawn_uav.launch include)
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
            ' log_file:=', LaunchConfiguration('log_file'),
            ' wait_to_record_bag:=', LaunchConfiguration('wait_to_record_bag'),
            ' uav_name:=', uav_name,
            ' namespace:=', namespace,
            ' is_input_joystick:=', LaunchConfiguration('is_input_joystick'),
        ]),
        value_type=str
    )

    # PX4 SITL process (NEW - replaces LibrePilot SITL)
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_none'
        ],
        output='screen',
        shell=True,
    )
        
    # Group for namespaced nodes (EXACT ROS1 equivalent: <group ns="$(arg uav_name)">)
    blimp_group = GroupAction([
        PushRosNamespace(namespace),
        
        # Robot state publisher (EXACT ROS1 equivalent)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                **blimp_params  # Load blimp.yaml parameters
            }],
            output='screen'
        ),
        
        # Joint state publisher (EXACT ROS1 equivalent)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # Spawn entity in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', namespace,
                '-x', LaunchConfiguration('X'),
                '-y', LaunchConfiguration('Y'),
                '-z', LaunchConfiguration('Z'),
            ],
            parameters=[{'use_sim_time': True}],
        ),
        
        # Controller spawner (EXACT ROS1 equivalent: controller_manager spawner)
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=[
                'revolute_joint_state_controller',
                'stick_joint_position_controller', 
                'botfin_joint_position_controller',
                'topfin_joint_position_controller',
                'leftfin_joint_position_controller',
                'rightfin_joint_position_controller',
                '--controller-manager', '/blimp/controller_manager'
            ],
            parameters=[{
                'use_sim_time': True,
                **controller_params  # Load controller_blimp.yaml parameters
            }]
        ),
        
        # MAVROS Node (REPLACES LibrePilot gcs2ros/ros2gcs)
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
                # Map MAVROS topics to blimp-specific topics (same as ROS1 gcshitl)
                ('/mavros/imu/data', 'tail/imu'),
                ('/mavros/global_position/local', 'tail/position'),  # Same as gcshitl (not pose_with_drift)
                ('/mavros/global_position/rel_alt', 'ground_speed'),
                ('/mavros/vfr_hud', 'tail/indicatedAirspeed'),
            ]
        ),
        
        # PX4 Blimp Control Node (REPLACES gcs_blimp.py)
        Node(
            package='blimp_description',
            executable='px4_blimp_control.py',
            name='px4_blimp_control',
            output='screen',
            parameters=[{'use_sim_time': True}],
            # NOTE: NO remapping for gcshitl version (same as original ROS1)
        ),
        
        # Sensor Bridge Node (REPLACES gcs2ros.py and ros2gcs.py functionality)
        Node(
            package='blimp_description',
            executable='mavros_sensor_bridge.py',
            name='mavros_sensor_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Fake GPS Drift Node (KEEP - unchanged from ROS1)
        Node(
            package='fake_gps_drift',
            executable='fake_gps_drift_node',
            name='fake_gps_drift_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
    
    return LaunchDescription([
        # ADDED: Set Gazebo resource path first
        gazebo_resource_path,
        
        # Declare arguments (EXACT ROS1 equivalents with same defaults)
        DeclareLaunchArgument('uav_name', default_value='blimp'),
        DeclareLaunchArgument('roboID', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='blimp'),
        DeclareLaunchArgument('is_input_joystick', default_value='false'),
        DeclareLaunchArgument('enable_meshes', default_value='true'),    # ROS1 original: true
        DeclareLaunchArgument('enable_wind', default_value='true'),      # ROS1 original: true
        DeclareLaunchArgument('enable_physics', default_value='true'),   # ROS1 original: true
        DeclareLaunchArgument('enable_sensors', default_value='true'),   # ROS1 original: true
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'), # ROS1 original: true
        DeclareLaunchArgument('enable_mavlink_interface', default_value='true'), # CHANGED: true for PX4
        DeclareLaunchArgument('world_name', default_value='basic'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),  # ROS1 original: 1.0
        
        # Launch PX4 SITL first
        px4_sitl,
        
        # NO Gazebo launch - gcshitl assumes external environment
        
        # Start the blimp group with PX4 integration
        blimp_group,
    ])