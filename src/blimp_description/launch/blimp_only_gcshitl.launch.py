#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParametersFromFile
from launch.conditions import IfCondition

def generate_launch_description():
    # Package directory
    pkg_blimp = get_package_share_directory('blimp_description')

    # Launch arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace', default=uav_name)
    rvizconfig = LaunchConfiguration('rvizconfig')

    # Paths to resource files (not config!)
    controller_config = os.path.join(pkg_blimp, 'resource', 'controller_blimp.yaml')
    blimp_config = os.path.join(pkg_blimp, 'resource', 'blimp.yaml')
    rviz_config_file = PathJoinSubstitution([pkg_blimp, 'rviz', 'blimp.rviz'])

    # Group with namespace for blimp components
    blimp_group = GroupAction([
        PushRosNamespace(namespace),

        # Load parameters from resource folder
        SetParametersFromFile(controller_config),
        SetParametersFromFile(blimp_config),

        # Spawn UAV
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_blimp, 'launch', 'spawn_uav.launch.py')
            ),
            launch_arguments={
                'uav_name': uav_name,
                'namespace': namespace,
                'model': os.path.join(pkg_blimp, 'urdf', 'blimp_base.xacro'),
                'enable_meshes': LaunchConfiguration('enable_meshes'),
                'enable_wind': LaunchConfiguration('enable_wind'),
                'enable_physics': LaunchConfiguration('enable_physics'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'enable_logging': LaunchConfiguration('enable_logging'),
                'enable_ground_truth': LaunchConfiguration('enable_ground_truth'),
                'enable_mavlink_interface': LaunchConfiguration('enable_mavlink_interface'),
                'log_file': LaunchConfiguration('log_file'),
                'wait_to_record_bag': LaunchConfiguration('wait_to_record_bag'),
                'X': LaunchConfiguration('X'),
                'Y': LaunchConfiguration('Y'),
                'Z': LaunchConfiguration('Z'),
            }.items()
        ),

        # ROS2 Control Manager (replaces the old controller spawner)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[controller_config]
        ),

        # Controller spawner (spawn individual controllers)
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

        # Robot State Publisher (MISSING - ADDED BACK)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),

        # Joint state publisher 
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),

        # Blimp Control Node (gcs_blimp.py) (MISSING - ADDED BACK)
        Node(
            package='blimp_description',
            executable='gcs_blimp.py',
            name='blimp_ctrl',
            output='screen',
            arguments=[f'/{namespace}'],
        ),

        # Fake GPS Drift Node (MISSING - ADDED BACK)
        Node(
            package='fake_gps_drift',
            executable='fake_gps_drift_node',
            name='fake_gps_drift_node',
            output='screen',
        ),

        # GCS to ROS Bridge (MISSING - ADDED BACK)
        Node(
            package='roshitl',
            executable='gcs2ros.py',
            name='gcs2ros',
            output='screen',
        ),

        # ROS to GCS Bridge with remapping (MISSING - ADDED BACK)
        Node(
            package='roshitl',
            executable='ros2gcs.py',
            name='ros2gcs',
            output='screen',
            remappings=[
                ('GCSIMU', 'tail/imu'),
                ('GCSPOS', 'tail/position'),
                ('GCSVEL', 'ground_speed'),
                ('GCSAIRSPEED', 'tail/indicatedAirspeed'),
            ],
        ),
    ])

    # RViz2 (was commented out in original ROS1)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # ROS-Gazebo bridge for topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # Basic robot topics
            f'/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            f'/{namespace}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/{namespace}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            f'/{namespace}/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            f'/{namespace}/wind_speed@geometry_msgs/msg/Vector3@gz.msgs.Vector3d',
            f'/{namespace}/wind_force@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
            # Control topics
            f'/{namespace}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
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
        DeclareLaunchArgument('enable_rviz', default_value='false'),  # Was commented out in ROS1
        DeclareLaunchArgument('log_file', default_value='blimp'),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('rvizconfig', 
            default_value=os.path.join(pkg_blimp, 'rviz', 'blimp.rviz')),
        DeclareLaunchArgument('X', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('Z', default_value='1.0'),
        
        # Launch components
        blimp_group,
        rviz_node,
        bridge,
    ])
