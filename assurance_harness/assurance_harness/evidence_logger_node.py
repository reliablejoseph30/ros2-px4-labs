import rclpy
import csv
import time
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class EvidenceLogger(Node):
    def __init__(self):
        super().__init__("evidence_logger")
        self.pose = None
        self.risk = None

        out_path = Path("evidence.csv")
        self.file = out_path.open("w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t_unix", "x", "y", "z", "risk"])

        self.create_subscription(PoseStamped, "/assurance/pose", self.pose_cb, 10)
        self.create_subscription(Float32, "/assurance/risk", self.risk_cb, 10)

        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(f"EvidenceLogger writing: {out_path.resolve()}")

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def risk_cb(self, msg: Float32):
        self.risk = float(msg.data)

    def tick(self):
        if (self.pose is None) or (self.risk is None):
            return
        p = self.pose.pose.position
        self.writer.writerow([time.time(), float(p.x), float(p.y), float(p.z), self.risk])
        self.file.flush()

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = EvidenceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
