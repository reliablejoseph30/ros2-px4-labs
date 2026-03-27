#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Setpoint: 5 meters above origin
        self.setpoint_msg = PoseStamped()
        self.setpoint_msg.pose.position.x = 0.0
        self.setpoint_msg.pose.position.y = 0.0
        self.setpoint_msg.pose.position.z = 5.0
        self.setpoint_msg.pose.orientation.w = 1.0

    def timer_callback(self):
        self.setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_msg.header.frame_id = "map"
        self.publisher.publish(self.setpoint_msg)
        self.get_logger().info(
            "Publishing position setpoint x=%.1f, y=%.1f, z=%.1f" %
            (self.setpoint_msg.pose.position.x,
             self.setpoint_msg.pose.position.y,
             self.setpoint_msg.pose.position.z)
        )

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    node.get_logger().info("Offboard control node started. Streaming setpoints...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
