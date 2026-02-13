#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time


class OffboardControlNode(Node):
    def __init__(self):
        # Initialize the ROS node
        super().__init__('offboard_control_node')

        # Create a publisher to the setpoint_position/local topic
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Create a timer to periodically send setpoints
        self.timer = self.create_timer(0.5, self.send_setpoint)  # 2 Hz = 0.5s interval

        # Initialize PoseStamped message
        self.pose = PoseStamped()

    def send_setpoint(self):
        # Create a setpoint position in the form of PoseStamped
        self.pose.header = Header()
        self.pose.header.stamp = self.get_clock().now().to_msg()

        # Set a target position (example: x=5, y=0, z=2)
        self.pose.pose.position.x = 5.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0

        # Set the orientation (this is a simple placeholder for orientation)
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 1.0

        # Publish the setpoint to the topic
        self.publisher.publish(self.pose)
        self.get_logger().info('Publishing Setpoint: [x: %.2f, y: %.2f, z: %.2f]' % 
                               (self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z))


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the OffboardControlNode
    offboard_control_node = OffboardControlNode()

    # Spin the node so it continues executing
    rclpy.spin(offboard_control_node)

    # Shutdown and cleanup when done
    offboard_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
