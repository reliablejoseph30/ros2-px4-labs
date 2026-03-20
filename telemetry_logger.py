#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv
import os
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.expanduser(f'~/mission_log_{timestamp}.csv')
        self.csv_file = open(log_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['ros_time', 'state', 'x', 'y', 'z'])
        self.current_state = 'UNKNOWN'
        self.current_pose = None

        # Match MAVROS QoS (best effort)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.create_subscription(String, '/mission/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, mavros_qos)
        self.create_timer(0.5, self.log_row)
        self.get_logger().info(f'TelemetryLogger writing to {log_path}')

    def state_cb(self, msg):
        self.current_state = msg.data

    def pose_cb(self, msg):
        self.current_pose = msg.pose.position

    def log_row(self):
        if self.current_pose is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([f'{t:.3f}', self.current_state, f'{self.current_pose.x:.3f}', f'{self.current_pose.y:.3f}', f'{self.current_pose.z:.3f}'])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
