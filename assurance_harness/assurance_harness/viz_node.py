import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32

class RiskViz(Node):
    def __init__(self):
        super().__init__("risk_viz")
        self.risk = 0.0
        self.points = []  # list of (x, y, z, risk)

        self.create_subscription(Float32, "/assurance/risk", self.risk_cb, 10)
        self.create_subscription(PoseStamped, "/assurance/pose", self.pose_cb, 10)

        self.pub = self.create_publisher(Marker, "/assurance/route", 10)
        self.declare_parameter("frame_id", "map")
        self.get_logger().info("RiskViz publishing /assurance/route (visualization_msgs/Marker)")

    def risk_cb(self, msg: Float32):
        self.risk = float(msg.data)

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.points.append((float(p.x), float(p.y), float(p.z), self.risk))

        m = Marker()
        m.header = msg.header
        m.header.frame_id = self.get_parameter("frame_id").value
        m.ns = "assurance_route"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.2

        for x, y, z, _r in self.points:
            m.points.append(Point(x=x, y=y, z=z))

        # Encode current risk as colour
        m.color.a = 1.0
        if self.risk > 0.8:
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
        elif self.risk > 0.3:
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
        else:
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0

        self.pub.publish(m)

def main():
    rclpy.init()
    node = RiskViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
