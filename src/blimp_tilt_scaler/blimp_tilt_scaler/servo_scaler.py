#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ServoScaler(Node):
    """
    Scales PX4 servo commands from ±raw_max_deg to:
      - tilt_index:        ±tilt_target_deg
      - all other servos:  ±others_target_deg

    Subscribes:  /model/<model_name>/servo_<i>
    Publishes:   /model/<model_name>/servo_<i>_scaled
    """

    def __init__(self):
        super().__init__('servo_scaler')

        # Parameters (override with --ros-args -p name:=value)
        self.declare_parameter('model_name', 'blimp')
        self.declare_parameter('servo_count', 5)           # how many /servo_i to watch
        self.declare_parameter('tilt_index', 4)            # 0-based index of tilt (e.g., 4 for Servo 5)
        self.declare_parameter('raw_max_deg', 45.0)        # what PX4 actually outputs (±45°)
        self.declare_parameter('tilt_target_deg', 100.0)   # desired tilt travel (±100°)
        self.declare_parameter('others_target_deg', 60.0)  # desired non-tilt travel (±60°)

        self.model_name       = self.get_parameter('model_name').get_parameter_value().string_value
        self.servo_count      = self.get_parameter('servo_count').get_parameter_value().integer_value
        self.tilt_index       = self.get_parameter('tilt_index').get_parameter_value().integer_value
        self.raw_max_deg      = float(self.get_parameter('raw_max_deg').get_parameter_value().double_value)
        self.tilt_target_deg  = float(self.get_parameter('tilt_target_deg').get_parameter_value().double_value)
        self.others_target_deg= float(self.get_parameter('others_target_deg').get_parameter_value().double_value)

        # Gains
        if self.raw_max_deg <= 0.0:
            raise ValueError("raw_max_deg must be > 0")
        self.gain_tilt   = self.tilt_target_deg   / self.raw_max_deg   # e.g., 100/45 ≈ 2.2222
        self.gain_others = self.others_target_deg / self.raw_max_deg   # e.g.,  60/45 = 1.3333

        self.pubs = []
        self.subs = []
        for i in range(self.servo_count):
            in_topic  = f'/model/{self.model_name}/servo_{i}'
            out_topic = f'/model/{self.model_name}/servo_{i}_scaled'
            gain = self.gain_tilt if i == self.tilt_index else self.gain_others

            pub = self.create_publisher(Float64, out_topic, 10)
            cb  = self._make_cb(pub, gain, i, in_topic, out_topic)
            sub = self.create_subscription(Float64, in_topic, cb, 10)

            self.pubs.append(pub)
            self.subs.append(sub)

        self.get_logger().info(
            f"ServoScaler active for {self.servo_count} servos on model '{self.model_name}'. "
            f"tilt_index={self.tilt_index}, gains: tilt={self.gain_tilt:.6f}, others={self.gain_others:.6f}"
        )

    def _make_cb(self, pub, gain, idx, in_topic, out_topic):
        def _cb(msg: Float64):
            out = Float64()
            out.data = msg.data * gain
            pub.publish(out)
        return _cb

def main():
    rclpy.init()
    node = ServoScaler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
