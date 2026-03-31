import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TelemetryGate(Node):
    def __init__(self):
        super().__init__("telemetry_gate")
        # Upstream pose topic (change to match your bridge)
        self.declare_parameter("input_pose_topic", "/mavros/local_position/pose")
        input_topic = self.get_parameter("input_pose_topic").get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(PoseStamped, input_topic, self.cb, qos)
        self.pub = self.create_publisher(PoseStamped, "/assurance/pose", 10)
        self.get_logger().info(f"TelemetryGate subscribing to: {input_topic}")

    def cb(self, msg: PoseStamped):
        # Pass-through normalisation
        gated = PoseStamped()
        gated.header = msg.header
        gated.pose = msg.pose
        self.pub.publish(gated)

def main():
    rclpy.init()
    node = TelemetryGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
