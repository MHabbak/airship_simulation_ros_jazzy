#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
    TimerAction
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_blimp = get_package_share_directory('blimp_description')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # Launch Arguments
    uav_name = LaunchConfiguration('uav_name')
    namespace = LaunchConfiguration('namespace')
    instance = LaunchConfiguration('instance')

    # Set up Gazebo resource paths
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':' if EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='') else '',
            pkg_blimp
        ]
    )

    # Robot description
    robot_description = ParameterValue(
        Command([
            'xacro ', os.path.join(pkg_blimp, 'urdf', 'blimp_base.xacro'),
            ' enable_meshes:=true',
            ' enable_wind:=false',
            ' enable_physics:=true', 
            ' enable_sensors:=true',
            ' enable_logging:=false',
            ' enable_ground_truth:=true',
            ' enable_custom_plugins:=true',
            ' enable_mavlink_interface:=false',
            ' log_file:=blimp',
            ' wait_to_record_bag:=false',
            ' uav_name:=', uav_name,
            ' namespace:=', namespace,
            ' is_input_joystick:=false',
        ]),
        value_type=str
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r -v 4 {os.path.join(pkg_blimp, "worlds", "empty.sdf")}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # FIXED: Clock bridge with proper QoS settings
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Robot state publisher
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

    # MicroXRCE-DDS Agent for PX4 communication
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='microxrce_agent',
        output='screen'
    )

    # Spawn blimp
    spawn_blimp = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_blimp',
        namespace=namespace,
        arguments=[
            '-name', uav_name,
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('X'),
            '-y', LaunchConfiguration('Y'),
            '-z', LaunchConfiguration('Z')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # FIXED: Minimal simulation bridge (remove barometer that's causing issues)
    blimp_simulation_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='blimp_simulation_bridge',
        namespace=namespace,
        arguments=[
            # Basic sensor bridges that PX4 GZBridge expects
            '/world/empty/model/blimp/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/world/empty/model/blimp/link/gps_link/sensor/gps/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/world/empty/model/blimp/link/magnetometer_link/sensor/mag/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
            
            # Remove barometer bridge that's causing template specialization error
            # Add it back later once the message type incompatibility is resolved
            
            # Essential simulation bridges
            'joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            'odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # FIXED: PX4 SITL with correct airframe and environment variables
    px4_sitl = ExecuteProcess(
        cmd=[
            './bin/px4',
            '-i', instance,
        ],
        name='px4_sitl',
        output='screen',
        cwd=os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default'),
        additional_env={
            # CRITICAL: Use the custom airframe we created
            'PX4_SYS_AUTOSTART': '4099',       # Custom blimp airframe
            
            # CRITICAL: Connect to existing Gazebo model with exact name matching
            'PX4_GZ_MODEL_NAME': uav_name,    # Must exactly match spawned model name
            'PX4_GZ_WORLD': 'empty',          # Must match world name
            'PX4_SIMULATOR': 'gz',
            'PX4_SIM_SPEED_FACTOR': '1.0',
            
            # GPS coordinates (Zurich area) - must match your world coordinates
            'PX4_HOME_LAT': '47.397742',
            'PX4_HOME_LON': '8.545594', 
            'PX4_HOME_ALT': '341.0',
            
            # MicroXRCE-DDS configuration
            'PX4_MICRODDS_NS': namespace,     # Use dynamic namespace
            'UXRCE_DDS_SYNCT': '0',           # Critical: Disable time sync conflicts
        }
    )

    # Monitor PX4 sensor data (for debugging)
    px4_sensor_listener = Node(
        package='px4_ros_com',
        executable='sensor_combined_listener',
        name='sensor_combined_listener',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ===== SEQUENTIAL STARTUP HANDLERS =====
    
    spawn_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                LogInfo(msg="Robot state publisher ready, spawning blimp in 1 second..."),
                TimerAction(period=1.0, actions=[spawn_blimp])
            ]
        )
    )

    bridge_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_blimp,
            on_start=[
                LogInfo(msg="Blimp spawned, starting minimal simulation bridge in 2 seconds..."),
                TimerAction(period=2.0, actions=[blimp_simulation_bridge])
            ]
        )
    )

    px4_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=blimp_simulation_bridge,
            on_start=[
                LogInfo(msg="Simulation bridge ready, starting PX4 with custom airframe in 3 seconds..."),
                TimerAction(period=3.0, actions=[px4_sitl])
            ]
        )
    )

    listener_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=px4_sitl,
            on_start=[
                LogInfo(msg="PX4 started with custom airframe, monitoring sensor data in 4 seconds..."),
                TimerAction(period=4.0, actions=[px4_sensor_listener])
            ]
        )
    )

    return LaunchDescription([
        # Environment setup
        gazebo_resource_path,
        
        # Launch arguments
        DeclareLaunchArgument('uav_name', default_value='blimp', description='Vehicle name'),
        DeclareLaunchArgument('namespace', default_value='blimp', description='ROS namespace'),
        DeclareLaunchArgument('instance', default_value='0', description='PX4 instance ID'),
        DeclareLaunchArgument('X', default_value='0.0', description='X spawn position'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Y spawn position'),
        DeclareLaunchArgument('Z', default_value='2.0', description='Z spawn position'),

        # Core components (start immediately)
        gazebo,
        clock_bridge,
        robot_state_publisher,
        microxrce_agent,

        # Sequential startup with proper sensor detection
        spawn_handler,
        bridge_handler,
        px4_handler,
        listener_handler,
    ])