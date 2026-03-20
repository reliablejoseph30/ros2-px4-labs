#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def generate_lawnmower_waypoints(
    origin_x,
    origin_y,
    altitude,
    line_spacing,
    waypoint_spacing,
    num_lines,
    line_length
):
    """
    Generate a lawnmower (boustrophedon) survey pattern.
    """

    waypoints = []

    for i in range(num_lines):
        x = origin_x + i * line_spacing

        # Alternate direction each line
        direction = 1 if i % 2 == 0 else -1

        num_wps = int(line_length / waypoint_spacing) + 1

        for j in range(num_wps):
            y = origin_y + direction * j * waypoint_spacing
            waypoints.append((x, y, altitude))

    return waypoints


class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__('mission_planner')

        # ── Parameters ─────────────────────────────
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('cruise_speed', 3.0)
        self.declare_parameter('line_spacing', 5.0)
        self.declare_parameter('waypoint_spacing', 3.0)
        self.declare_parameter('stationkeep_duration', 3.0)
        self.declare_parameter('num_lines', 4)
        self.declare_parameter('line_length', 24.0)

        # ── Read parameters ────────────────────────
        self.altitude = self.get_parameter('altitude').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.line_spacing = self.get_parameter('line_spacing').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.stationkeep_dur = self.get_parameter('stationkeep_duration').value
        self.num_lines = self.get_parameter('num_lines').value
        self.line_length = self.get_parameter('line_length').value

        # ── Log parameters ─────────────────────────
        self.get_logger().info(
            f'MissionPlanner — alt={self.altitude}m '
            f'speed={self.cruise_speed}m/s '
            f'line_spacing={self.line_spacing}m '
            f'wp_spacing={self.waypoint_spacing}m '
            f'stationkeep={self.stationkeep_dur}s '
            f'lines={self.num_lines} '
            f'line_length={self.line_length}m'
        )

        # ── Publisher ──────────────────────────────
        self.wp_pub = self.create_publisher(Path, '/mission/waypoints', 10)

        # ── Generate waypoints ─────────────────────
        waypoints = generate_lawnmower_waypoints(
            origin_x=0.0,
            origin_y=0.0,
            altitude=self.altitude,
            line_spacing=self.line_spacing,
            waypoint_spacing=self.waypoint_spacing,
            num_lines=self.num_lines,
            line_length=self.line_length,
        )

        self.get_logger().info(f'Generated {len(waypoints)} waypoints.')

        for idx, wp in enumerate(waypoints):
            self.get_logger().info(
                f'WP {idx:02d}: x={wp[0]:.1f} y={wp[1]:.1f} z={wp[2]:.1f}'
            )

        # ── Build Path message ─────────────────────
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for (x, y, z) in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # ── Publish at 1 Hz ────────────────────────
        self.path_msg = path_msg
        self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.wp_pub.publish(self.path_msg)
        self.get_logger().debug('Waypoints published.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
