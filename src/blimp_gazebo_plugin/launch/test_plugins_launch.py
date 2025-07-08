#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_blimp_gazebo = get_package_share_directory('blimp_gazebo_plugin')
    
    # Get install directory for plugins
    install_dir = os.path.join(pkg_blimp_gazebo, '..', '..')
    plugin_dir = os.path.join(install_dir, 'lib', 'blimp_gazebo_plugin')
    
    # Set Gazebo plugin path
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            plugin_dir, ':',
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )
    
    # Debug: Print the plugin path
    print(f"Setting GZ_SIM_SYSTEM_PLUGIN_PATH to include: {plugin_dir}")
    
    # Set Gazebo resource path for models/worlds
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_blimp_gazebo, 'worlds'), ':',
            os.path.join(pkg_blimp_gazebo, 'models'), ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )
    
    # Path to test world
    world_file = os.path.join(pkg_blimp_gazebo, 'worlds', 'test_plugins.sdf')
    
    # Gazebo launch
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen'
    )
    
    # Bridge for Gazebo topics to ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # NormWind plugin topics
            '/test/wind_force@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
            '/test/wind_speed@geometry_msgs/msg/Vector3@gz.msgs.Vector3d',
            # DynamicVolume plugin topics
            '/test/dynamic_volume@std_msgs/msg/Float32@gz.msgs.Float',
            '/test/helium_mass@std_msgs/msg/Float32@gz.msgs.Float',
            # FinLiftDrag plugin topics
            '/test/airspeed_left@std_msgs/msg/Float32@gz.msgs.Float',
            '/test/airspeed_right@std_msgs/msg/Float32@gz.msgs.Float',
        ],
        output='screen'
    )
    
    # Test publisher for helium mass
    test_publisher = Node(
        package='blimp_gazebo_plugin',
        executable='test_helium_publisher.py',
        name='test_helium_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        gz_plugin_path,
        gz_resource_path,
        gazebo,
        bridge,
        # test_publisher  # Uncomment when you have the test script
    ])