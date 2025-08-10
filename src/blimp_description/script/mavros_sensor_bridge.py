#!/usr/bin/env python3
"""
MAVROS Sensor Bridge Node
Replaces gcs2ros.py and ros2gcs.py from LibrePilot system
Bridges MAVROS topics to blimp-specific topic names
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, VfrHud
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class MAVROSSensorBridge(Node):
    def __init__(self):
        super().__init__('mavros_sensor_bridge')
        
        # MAVROS subscribers (input from PX4)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.vel_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.vel_callback, 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.vfr_hud_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback, 10)
        
        # Blimp-specific publishers (output to blimp topics)
        # These match the original LibrePilot topic names
        self.blimp_imu_pub = self.create_publisher(Imu, 'tail/imu', 10)
        self.blimp_gps_pub = self.create_publisher(NavSatFix, 'tail/gps', 10)
        self.blimp_pose_pub = self.create_publisher(PoseStamped, 'tail/position', 10)
        self.blimp_odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.ground_speed_pub = self.create_publisher(Vector3Stamped, 'ground_speed', 10)
        self.airspeed_pub = self.create_publisher(Float64, 'tail/indicatedAirspeed', 10)
        
        # TF broadcaster for transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store latest data for odometry combination
        self.latest_pose = None
        self.latest_twist = None
        
        # Connection status
        self.px4_connected = False
        
        self.get_logger().info("MAVROS Sensor Bridge Started")

    def state_callback(self, msg):
        """Handle PX4 connection state"""
        if msg.connected and not self.px4_connected:
            self.get_logger().info("PX4 connected via MAVROS")
            self.px4_connected = True
        elif not msg.connected and self.px4_connected:
            self.get_logger().warn("PX4 disconnected")
            self.px4_connected = False

    def imu_callback(self, msg):
        """
        Bridge IMU data from MAVROS to blimp topic
        Topic: /mavros/imu/data -> tail/imu
        """
        # Republish IMU data with blimp frame
        imu_msg = msg
        imu_msg.header.frame_id = "tail_link"  # Match blimp URDF frame
        
        self.blimp_imu_pub.publish(imu_msg)

    def gps_callback(self, msg):
        """
        Bridge GPS data from MAVROS to blimp topic  
        Topic: /mavros/global_position/global -> tail/gps
        """
        # Republish GPS data with blimp frame
        gps_msg = msg
        gps_msg.header.frame_id = "tail_link"
        
        self.blimp_gps_pub.publish(gps_msg)

    def pose_callback(self, msg):
        """
        Bridge pose data and publish as blimp position
        Topic: /mavros/local_position/pose -> tail/position
        """
        # Store for odometry and republish
        self.latest_pose = msg
        
        # Republish pose with blimp frame
        pose_msg = msg
        pose_msg.header.frame_id = "map"  # Global frame
        pose_msg.child_frame_id = "base_link"  # Blimp base frame
        
        self.blimp_pose_pub.publish(pose_msg)
        
        # Broadcast TF transform
        self.broadcast_transform(msg)
        
        # Update odometry if we have velocity data
        if self.latest_twist:
            self.publish_odometry()

    def vel_callback(self, msg):
        """
        Bridge velocity data and convert to ground speed
        Topic: /mavros/local_position/velocity_local -> ground_speed + odometry
        """
        # Store for odometry
        self.latest_twist = msg.twist
        
        # Publish ground speed as Vector3Stamped (matches LibrePilot format)
        ground_speed_msg = Vector3Stamped()
        ground_speed_msg.header = msg.header
        ground_speed_msg.header.frame_id = "base_link"
        ground_speed_msg.vector.x = msg.twist.linear.x
        ground_speed_msg.vector.y = msg.twist.linear.y
        ground_speed_msg.vector.z = msg.twist.linear.z
        
        self.ground_speed_pub.publish(ground_speed_msg)
        
        # Update odometry if we have pose data
        if self.latest_pose:
            self.publish_odometry()

    def vfr_hud_callback(self, msg):
        """
        Bridge airspeed data from VFR HUD
        Topic: /mavros/vfr_hud -> tail/indicatedAirspeed
        """
        # Extract airspeed and publish as Float64 (matches LibrePilot format)
        airspeed_msg = Float64()
        airspeed_msg.data = float(msg.airspeed)
        
        self.airspeed_pub.publish(airspeed_msg)

    def publish_odometry(self):
        """
        Combine pose and velocity into odometry message
        This replaces the combined functionality of gcs2ros.py
        """
        if not self.latest_pose or not self.latest_twist:
            return
            
        odom_msg = Odometry()
        odom_msg.header = self.latest_pose.header
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        
        # Copy pose
        odom_msg.pose.pose = self.latest_pose.pose
        
        # Copy twist
        odom_msg.twist.twist = self.latest_twist
        
        # Set covariances (use default values)
        odom_msg.pose.covariance = [0.1] * 36
        odom_msg.twist.covariance = [0.1] * 36
        
        self.blimp_odom_pub.publish(odom_msg)

    def broadcast_transform(self, pose_msg):
        """
        Broadcast TF transform from map to base_link
        This maintains the transform tree for visualization
        """
        t = TransformStamped()
        
        t.header = pose_msg.header
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        # Copy translation
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        
        # Copy rotation
        t.transform.rotation = pose_msg.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    node = MAVROSSensorBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MAVROS Sensor Bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()