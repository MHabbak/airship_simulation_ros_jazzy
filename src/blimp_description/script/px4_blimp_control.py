#!/usr/bin/env python3
"""
PX4 Blimp Control Node
Replaces gcs_blimp.py from LibrePilot system
Converts blimp control commands to PX4/MAVROS actuator commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import ActuatorControl, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64
import numpy as np

class PX4BlimpControl(Node):
    def __init__(self):
        super().__init__('px4_blimp_control')
        
        # Publishers for PX4 actuators
        self.actuator_pub = self.create_publisher(
            ActuatorControl, '/mavros/actuator_control', 10)
        
        # Subscribers for control commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Subscribe to PX4 state
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        
        # Service clients for arming/mode
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Initialize actuator message
        self.actuator_msg = ActuatorControl()
        self.actuator_msg.header.frame_id = "base_link"
        self.actuator_msg.group_mix = 0  # Use mixer group 0
        
        # Control state
        self.current_state = None
        self.is_armed = False
        self.control_mode = "MANUAL"
        
        # Blimp-specific actuator mapping
        # Based on original ROS1 implementation
        self.THRUST_LIMIT = 1.0      # Normalized thrust limit
        self.FIN_LIMIT = 0.5         # Normalized fin deflection limit
        self.VECTOR_LIMIT = 0.3      # Thrust vectoring limit
        
        # Timer for publishing actuator commands at 50Hz
        self.timer = self.create_timer(0.02, self.publish_actuators)
        
        # Initialize command values
        self.thrust_cmd = 0.0
        self.yaw_cmd = 0.0
        self.pitch_cmd = 0.0
        self.roll_cmd = 0.0
        
        self.get_logger().info("PX4 Blimp Control Node Started")
        self.get_logger().info("Waiting for MAVROS connection...")

    def state_callback(self, msg):
        """Handle PX4 state updates"""
        self.current_state = msg
        self.is_armed = msg.armed
        
        if msg.connected and not hasattr(self, '_connection_logged'):
            self.get_logger().info("MAVROS connected to PX4")
            self._connection_logged = True

    def cmd_vel_callback(self, msg):
        """
        Convert Twist commands to blimp actuator controls
        
        Twist mapping (same as original gcs_blimp.py):
        - linear.x: Forward thrust
        - linear.y: Lateral movement (via fins)
        - linear.z: Vertical movement (via thrust vectoring)
        - angular.x: Roll control (via differential fins)
        - angular.y: Pitch control (via thrust vectoring)
        - angular.z: Yaw control (via thrust vectoring + rudder)
        """
        
        # Extract commands with limits
        forward_thrust = np.clip(msg.linear.x, -self.THRUST_LIMIT, self.THRUST_LIMIT)
        lateral_cmd = np.clip(msg.linear.y, -self.FIN_LIMIT, self.FIN_LIMIT)
        vertical_cmd = np.clip(msg.linear.z, -self.VECTOR_LIMIT, self.VECTOR_LIMIT)
        
        roll_cmd = np.clip(msg.angular.x, -self.FIN_LIMIT, self.FIN_LIMIT)
        pitch_cmd = np.clip(msg.angular.y, -self.VECTOR_LIMIT, self.VECTOR_LIMIT)
        yaw_cmd = np.clip(msg.angular.z, -self.VECTOR_LIMIT, self.VECTOR_LIMIT)
        
        # Store for actuator publishing
        self.thrust_cmd = forward_thrust
        self.yaw_cmd = yaw_cmd
        self.pitch_cmd = pitch_cmd + vertical_cmd  # Combine pitch and vertical
        self.roll_cmd = roll_cmd

    def publish_actuators(self):
        """
        Publish actuator commands at 50Hz
        
        Actuator mapping (based on blimp URDF):
        Channel 0: Main thrust
        Channel 1: Thrust vector yaw
        Channel 2: Thrust vector pitch  
        Channel 3: Top fin deflection
        Channel 4: Bottom fin deflection
        Channel 5: Left fin deflection
        Channel 6: Right fin deflection
        Channel 7: Reserved
        """
        
        if not self.current_state or not self.current_state.connected:
            return
            
        # Map commands to actuator channels
        self.actuator_msg.controls[0] = float(self.thrust_cmd)      # Main thrust
        self.actuator_msg.controls[1] = float(self.yaw_cmd)         # Thrust vector yaw
        self.actuator_msg.controls[2] = float(self.pitch_cmd)       # Thrust vector pitch
        self.actuator_msg.controls[3] = float(self.roll_cmd)        # Top fin
        self.actuator_msg.controls[4] = float(-self.roll_cmd)       # Bottom fin (opposite)
        self.actuator_msg.controls[5] = float(self.pitch_cmd * 0.5) # Left fin (pitch assist)
        self.actuator_msg.controls[6] = float(self.pitch_cmd * 0.5) # Right fin (pitch assist)
        self.actuator_msg.controls[7] = 0.0                         # Reserved
        
        # Update timestamp and publish
        self.actuator_msg.header.stamp = self.get_clock().now().to_msg()
        self.actuator_pub.publish(self.actuator_msg)

    def arm_vehicle(self):
        """Arm the vehicle"""
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return False
            
        req = CommandBool.Request()
        req.value = True
        
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("Vehicle armed successfully")
            return True
        else:
            self.get_logger().error("Failed to arm vehicle")
            return False

    def set_offboard_mode(self):
        """Set vehicle to offboard mode"""
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Mode service not available")
            return False
            
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info("Offboard mode set successfully")
            return True
        else:
            self.get_logger().error("Failed to set offboard mode")
            return False

def main(args=None):
    rclpy.init(args=args)
    
    node = PX4BlimpControl()
    
    try:
        # Auto-arm after 5 seconds if connected
        def auto_setup():
            if node.current_state and node.current_state.connected:
                node.get_logger().info("Auto-arming vehicle...")
                if node.arm_vehicle():
                    node.set_offboard_mode()
        
        # Setup timer for auto-arm
        node.create_timer(5.0, auto_setup)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PX4 Blimp Control")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()