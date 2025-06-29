#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from message_filters import Subscriber
from rclpy.parameter import Parameter

class FakeGPSDriftNode(Node):
    def __init__(self):
        super().__init__('fake_gps_drift_node')

        self.declare_parameter('offsetX', 0.0)
        self.declare_parameter('offsetY', 0.0)
        self.declare_parameter('offsetZ', 0.0)
        self.overrideoffset = [
            self.get_parameter('offsetX').value,
            self.get_parameter('offsetY').value,
            self.get_parameter('offsetZ').value
        ]
        self.add_on_set_parameters_callback(self.param_callback)

        # Internal drift state
        self.acc_sigma = [0.005, 0.005, 0.01]
        self.acc_velocity_timer = 240
        self.acc_offset_scale = [0.5, 0.5, 1]
        self.fakeoffset = [random.gauss(0.0, s) for s in self.acc_offset_scale]
        self.fakedest = [random.gauss(0.0, s) for s in self.acc_offset_scale]
        self.fakevel = [(self.fakedest[i] - self.fakeoffset[i]) / self.acc_velocity_timer for i in range(3)]
        self.lasttime = self.get_clock().now()
        self.legtime = 0.0

        # Publishers
        self.fakeoffset_pub = self.create_publisher(PointStamped, 'fakegpsoffset', 10)
        self.pose_pub = self.create_publisher(PointStamped, 'tail/pose_with_drift', 10)

        # Subscriber using message_filters
        sub = Subscriber(self, PoseWithCovarianceStamped, 'tail/pose_with_covariance')
        sub.registerCallback(self.pose_callback)

        self.get_logger().info('fake_gps_drift_node initialized')

    def param_callback(self, params):
        for param in params:
            if param.name == 'offsetX':
                self.overrideoffset[0] = param.value
            elif param.name == 'offsetY':
                self.overrideoffset[1] = param.value
            elif param.name == 'offsetZ':
                self.overrideoffset[2] = param.value
        return rclpy.parameter.ParameterCallbackResult(successful=True)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        now = self.get_clock().now()
        duration = (now - self.lasttime).nanoseconds / 1e9
        self.lasttime = now

        if duration < 1.0:
            self.legtime += duration
            if self.legtime > 1.3 * self.acc_velocity_timer:
                self.fakedest = [random.gauss(0.0, s) for s in self.acc_offset_scale]
                self.fakevel = [
                    (self.fakedest[i] - self.fakeoffset[i]) / self.acc_velocity_timer
                    for i in range(3)
                ]
                self.legtime = 0.0

            old = self.fakevel.copy()
            for i in range(3):
                self.fakevel[i] += random.gauss(0.0, self.acc_sigma[i]) * duration
                self.fakeoffset[i] += duration * 0.5 * (self.fakevel[i] + old[i])

        # apply override if set
        if any(self.overrideoffset):
            self.fakeoffset = self.overrideoffset.copy()

        offset_msg = PointStamped()
        offset_msg.header = msg.header
        offset_msg.point.x = -self.fakeoffset[0]
        offset_msg.point.y = -self.fakeoffset[1]
        offset_msg.point.z = -self.fakeoffset[2]
        self.fakeoffset_pub.publish(offset_msg)

        drift_msg = PointStamped()
        drift_msg.header = msg.header
        drift_msg.point = msg.pose.pose.position
        self.pose_pub.publish(drift_msg)

def main():
    rclpy.init()
    node = FakeGPSDriftNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
