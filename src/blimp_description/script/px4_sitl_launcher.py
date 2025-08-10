#!/usr/bin/env python3
"""
PX4 SITL Launcher Node
Manages PX4 SITL process and configuration for blimp simulation
Optional script for advanced PX4 management
"""

import rclpy
from rclpy.node import Node
import subprocess
import os
import signal
import time
from std_msgs.msg import String
from mavros_msgs.msg import State

class PX4SITLLauncher(Node):
    def __init__(self):
        super().__init__('px4_sitl_launcher')
        
        # PX4 process handle
        self.px4_process = None
        self.px4_running = False
        
        # Publishers for status
        self.status_pub = self.create_publisher(String, 'px4_status', 10)
        
        # Subscriber for MAVROS state
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        
        # Parameters
        self.declare_parameter('px4_dir', os.path.expanduser('~/PX4-Autopilot'))
        self.declare_parameter('sitl_world', 'none')  # No world by default
        self.declare_parameter('vehicle_type', 'none')  # Generic vehicle
        self.declare_parameter('auto_start', True)
        
        self.px4_dir = self.get_parameter('px4_dir').value
        self.sitl_world = self.get_parameter('sitl_world').value
        self.vehicle_type = self.get_parameter('vehicle_type').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f"PX4 SITL Launcher initialized")
        self.get_logger().info(f"PX4 directory: {self.px4_dir}")
        
        # Auto-start if enabled
        if self.auto_start:
            self.start_px4_sitl()

    def state_callback(self, msg):
        """Monitor MAVROS connection to PX4"""
        if msg.connected and not hasattr(self, '_mavros_connected'):
            self.get_logger().info("MAVROS successfully connected to PX4 SITL")
            self._mavros_connected = True

    def start_px4_sitl(self):
        """Start PX4 SITL process"""
        if self.px4_running:
            self.get_logger().warn("PX4 SITL already running")
            return False
            
        if not os.path.exists(self.px4_dir):
            self.get_logger().error(f"PX4 directory not found: {self.px4_dir}")
            return False
            
        try:
            # Build command based on configuration
            if self.sitl_world == 'none':
                # No world integration (for external Gazebo)
                cmd = ['make', 'px4_sitl_default', 'none']
            else:
                # With Gazebo world
                cmd = ['make', 'px4_sitl_default', f'gazebo-classic_{self.sitl_world}']
            
            self.get_logger().info(f"Starting PX4 SITL: {' '.join(cmd)}")
            
            # Start PX4 process
            self.px4_process = subprocess.Popen(
                cmd,
                cwd=self.px4_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.px4_running = True
            self.get_logger().info("PX4 SITL started successfully")
            
            # Give PX4 time to initialize
            time.sleep(3)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to start PX4 SITL: {str(e)}")
            return False

    def stop_px4_sitl(self):
        """Stop PX4 SITL process"""
        if not self.px4_running or not self.px4_process:
            self.get_logger().warn("PX4 SITL not running")
            return
            
        try:
            # Terminate process group
            os.killpg(os.getpgid(self.px4_process.pid), signal.SIGTERM)
            
            # Wait for process to finish
            self.px4_process.wait(timeout=10)
            
            self.px4_running = False
            self.px4_process = None
            
            self.get_logger().info("PX4 SITL stopped successfully")
            
        except subprocess.TimeoutExpired:
            # Force kill if necessary
            self.get_logger().warn("PX4 SITL didn't stop gracefully, force killing")
            os.killpg(os.getpgid(self.px4_process.pid), signal.SIGKILL)
            self.px4_running = False
            self.px4_process = None
            
        except Exception as e:
            self.get_logger().error(f"Error stopping PX4 SITL: {str(e)}")

    def restart_px4_sitl(self):
        """Restart PX4 SITL process"""
        self.get_logger().info("Restarting PX4 SITL...")
        self.stop_px4_sitl()
        time.sleep(2)
        self.start_px4_sitl()

    def publish_status(self):
        """Publish PX4 status"""
        status_msg = String()
        
        if self.px4_running and self.px4_process:
            # Check if process is still alive
            if self.px4_process.poll() is None:
                status_msg.data = "RUNNING"
            else:
                status_msg.data = "CRASHED"
                self.px4_running = False
                self.get_logger().error("PX4 SITL process crashed")
        else:
            status_msg.data = "STOPPED"
            
        self.status_pub.publish(status_msg)

    def cleanup(self):
        """Cleanup resources"""
        if self.px4_running:
            self.stop_px4_sitl()

def main(args=None):
    rclpy.init(args=args)
    
    node = PX4SITLLauncher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PX4 SITL Launcher")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()