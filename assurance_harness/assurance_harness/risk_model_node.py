import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from assurance_harness.dsm import ground_height, line_of_sight_clear

class RiskModel(Node):
    def __init__(self):
        super().__init__("risk_model")
        self.sub = self.create_subscription(PoseStamped, "/assurance/pose", self.cb, 10)
        self.pub = self.create_publisher(Float32, "/assurance/risk", 10)

        # Fixed ground control station (GCS) pose in the same frame as /assurance/pose
        self.declare_parameter("gcs_x", 0.0)
        self.declare_parameter("gcs_y", 0.0)
        self.declare_parameter("gcs_z", 2.0)

        self.get_logger().info("RiskModel publishing /assurance/risk (Float32: 0.0, 0.5, 1.0)")

    def cb(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        gcs = (
            self.get_parameter("gcs_x").value,
            self.get_parameter("gcs_y").value,
            self.get_parameter("gcs_z").value,
        )

        agl = z - ground_height(x, y)
        los = line_of_sight_clear((x, y, z), gcs)

        if agl < 15.0 and (not los):
            risk = 1.0
        elif agl < 15.0:
            risk = 0.5
        else:
            risk = 0.0

        self.pub.publish(Float32(data=risk))

def main():
    rclpy.init()
    node = RiskModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
