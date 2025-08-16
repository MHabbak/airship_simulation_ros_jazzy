#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ServoScaler(Node):
    def __init__(self):
        super().__init__('servo_scaler')
        self.sub = self.create_subscription(Float64, '/model/blimp/servo_0', self.cb, 10)
        self.pub = self.create_publisher(Float64, '/model/blimp/servo_0_scaled', 10)

    def cb(self, msg: Float64):
        out = Float64()
        out.data = msg.data * 2.0   # ±0.785 → ±1.571
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ServoScaler())

if __name__ == '__main__':
    main()
