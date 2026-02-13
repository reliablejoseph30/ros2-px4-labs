#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control_node')

        # Create publisher to send PoseStamped setpoints
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Create a timer to publish setpoints at 10 Hz
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the setpoint message
        self.setpoint_msg = PoseStamped()
        self.setpoint_msg.pose.position.x = 0.0
        self.setpoint_msg.pose.position.y = 0.0
        self.setpoint_msg.pose.position.z = 5.0  # 5 meters altitude

        # Orientation (no rotation relative to world)
        self.setpoint_msg.pose.orientation.x = 0.0
        self.setpoint_msg.pose.orientation.y = 0.0
        self.setpoint_msg.pose.orientation.z = 0.0
        self.setpoint_msg.pose.orientation.w = 1.0

    def timer_callback(self):
        # Update timestamp
        self.setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_msg.header.frame_id = "map"

        # Publish the setpoint
        self.publisher.publish(self.setpoint_msg)

        # Log message
        self.get_logger().info(
            f"Publishing position setpoint "
            f"x={self.setpoint_msg.pose.position.x:.1f}, "
            f"y={self.setpoint_msg.pose.position.y:.1f}, "
            f"z={self.setpoint_msg.pose.position.z:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    node.get_logger().info(
        "Offboard control node started. Streaming setpoints..."
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
