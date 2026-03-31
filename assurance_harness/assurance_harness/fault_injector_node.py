# assurance_harness/fault_injector_node.py
import rclpy
import random
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped


class FaultInjector(Node):
    def __init__(self):
        super().__init__("fault_injector")
        self.declare_parameter("drop_rate", 0.3)
        self.declare_parameter("input_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("output_pose_topic", "/assurance/pose_injected")

        self.drop_rate = float(self.get_parameter("drop_rate").value)
        inp = self.get_parameter("input_pose_topic").value
        out = self.get_parameter("output_pose_topic").value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(PoseStamped, inp, self.cb, qos)
        self.pub = self.create_publisher(PoseStamped, out, 10)
        self.get_logger().info(
            f"FaultInjector {inp} -> {out}, drop_rate={self.drop_rate}"
        )

    def cb(self, msg: PoseStamped):
        if random.random() > self.drop_rate:
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = FaultInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
