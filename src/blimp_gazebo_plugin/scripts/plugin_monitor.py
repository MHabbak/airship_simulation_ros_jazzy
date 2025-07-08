#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Wrench
import time

class PluginMonitor(Node):
    def __init__(self):
        super().__init__('plugin_monitor')
        
        # Subscribers for all plugin outputs
        self.wind_force_sub = self.create_subscription(
            Wrench, '/test/wind_force', self.wind_force_callback, 10)
        self.wind_speed_sub = self.create_subscription(
            Vector3, '/test/wind_speed', self.wind_speed_callback, 10)
        self.dynamic_volume_sub = self.create_subscription(
            Float32, '/test/dynamic_volume', self.dynamic_volume_callback, 10)
        
        # Publisher to control helium mass
        self.helium_pub = self.create_publisher(Float32, '/test/helium_mass', 10)
        
        # Timer to publish helium mass changes
        self.timer = self.create_timer(5.0, self.publish_helium_mass)
        self.helium_value = 1.0
        
        self.get_logger().info('Plugin Monitor Started - Watching for plugin outputs...')
        
    def wind_force_callback(self, msg):
        self.get_logger().info(f'Wind Force: F=({msg.force.x:.2f}, {msg.force.y:.2f}, {msg.force.z:.2f})')
        
    def wind_speed_callback(self, msg):
        self.get_logger().info(f'Wind Speed: V=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')
        
    def dynamic_volume_callback(self, msg):
        self.get_logger().info(f'Dynamic Volume: {msg.data:.3f} m³')
        
    def publish_helium_mass(self):
        # Cycle helium mass between 0.5 and 2.0
        self.helium_value += 0.2
        if self.helium_value > 2.0:
            self.helium_value = 0.5
            
        msg = Float32()
        msg.data = self.helium_value
        self.helium_pub.publish(msg)
        self.get_logger().info(f'Published Helium Mass: {self.helium_value:.2f} kg')

def main(args=None):
    rclpy.init(args=args)
    monitor = PluginMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()