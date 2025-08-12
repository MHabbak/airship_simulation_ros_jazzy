#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    LogInfo,
    TimerAction
)
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
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # 1. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r -v 4 {os.path.join(pkg_blimp, "worlds", "empty.sdf")}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 2. Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 4. MicroXRCE-DDS Agent
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='microxrce_agent',
        output='screen'
    )

    # 5. Spawn blimp with delay
    spawn_blimp = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="Spawning blimp at ground level..."),
            Node(
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
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # 6. Start PX4 with delay
    start_px4 = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="Starting PX4 with corrected airframe..."),
            ExecuteProcess(
                cmd=[
                    './bin/px4',
                    '-i', instance,
                ],
                name='px4_sitl',
                output='screen',
                cwd=os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default'),
                env={
                    **os.environ,
                    'PX4_SYS_AUTOSTART': '4099',
                    'PX4_GZ_MODEL_NAME': uav_name,
                    'PX4_GZ_WORLD': 'empty',
                    'PX4_SIMULATOR': 'gz',
                    'PX4_SIM_SPEED_FACTOR': '1.0',
                    'PX4_HOME_LAT': '47.397742',
                    'PX4_HOME_LON': '8.545594',
                    'PX4_HOME_ALT': '341.0',
                    'PX4_MICRODDS_NS': namespace,
                    'UXRCE_DDS_SYNCT': '0',
                }
            )
        ]
    )

    # 7. Optional: ROS sensor monitoring
    start_ros_monitoring = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg="Starting sensor topic monitoring..."),
            Node(
                package='ros_gz_bridge', 
                executable='parameter_bridge',
                name='ros_sensor_monitor',
                arguments=[
                    '/world/empty/model/blimp/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/world/empty/model/blimp/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                    '/world/empty/model/blimp/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                ],
                remappings=[
                    ('/world/empty/model/blimp/link/base_link/sensor/imu_sensor/imu', '/debug/gazebo_imu'),
                    ('/world/empty/model/blimp/link/base_link/sensor/navsat_sensor/navsat', '/debug/gazebo_gps'),
                    ('/world/empty/model/blimp/link/base_link/sensor/air_pressure_sensor/air_pressure', '/debug/gazebo_baro'),
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # 8. System verification
    verify_system = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="Running system verification..."),
            ExecuteProcess(
                cmd=['bash', '-c', '''
                    echo "=== Checking PX4 Parameters ==="
                    echo "param show SYS_AUTOSTART" | timeout 3 ~/PX4-Autopilot/Tools/mavlink_shell.py 0.0.0.0:14550 2>/dev/null || echo "PX4 not responding"
                    echo "param show EKF2_HGT_REF" | timeout 3 ~/PX4-Autopilot/Tools/mavlink_shell.py 0.0.0.0:14550 2>/dev/null || echo "Could not check height reference"
                    echo "=== Checking Health ==="
                    echo "commander check" | timeout 3 ~/PX4-Autopilot/Tools/mavlink_shell.py 0.0.0.0:14550 2>/dev/null || echo "Commander not ready"
                    echo "=== Checking ROS Topics ==="
                    timeout 2 ros2 topic list | grep -E "(imu|gps|baro)" || echo "Sensor topics not available yet"
                    echo "=== System Check Complete ==="
                '''],
                shell=True,
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Environment setup
        gazebo_resource_path,
        
        # Launch arguments with corrected spawn height
        DeclareLaunchArgument('uav_name', default_value='blimp', description='Vehicle name'),
        DeclareLaunchArgument('namespace', default_value='blimp', description='ROS namespace'),
        DeclareLaunchArgument('instance', default_value='0', description='PX4 instance ID'),
        DeclareLaunchArgument('X', default_value='0.0', description='X spawn position'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Y spawn position'),
        DeclareLaunchArgument('Z', default_value='0.5', description='Z spawn position (CORRECTED: was 50.0)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Launch sequence - immediate starts
        gazebo,
        clock_bridge,
        robot_state_publisher,
        microxrce_agent,

        # Timed sequence
        spawn_blimp,           # T+3: Spawn blimp at 0.5m height
        start_px4,             # T+6: Start PX4 with correct config
        # start_ros_monitoring,  # T+9: Start ROS bridges
        # verify_system,         # T+12: Check system status
    ])