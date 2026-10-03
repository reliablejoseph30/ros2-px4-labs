#!/usr/bin/env python3
"""
mission_planner.py
==================
ROS 2 node that generates and publishes a lawnmower (boustrophedon) survey
waypoint pattern for autonomous drone missions.

Publishes:
  /mission/waypoints  (nav_msgs/Path)  — full waypoint path at 1 Hz

All mission geometry is configurable via ROS 2 parameters:

  altitude          (float, default 10.0 m)  — flight altitude (ENU z)
  cruise_speed      (float, default 3.0 m/s) — informational; used by executor
  line_spacing      (float, default 5.0 m)   — lateral separation between survey lines
  waypoint_spacing  (float, default 3.0 m)   — longitudinal separation within a line
  num_lines         (int,   default 4)        — number of parallel survey lines
  line_length       (float, default 24.0 m)  — length of each survey line
  stationkeep_duration (float, default 3.0 s) — dwell time at each waypoint (passed to executor)
  origin_x          (float, default 0.0 m)   — survey area origin x (ENU)
  origin_y          (float, default 0.0 m)   — survey area origin y (ENU)

Usage:
    ros2 run ros2_px4_labs mission_planner
    ros2 run ros2_px4_labs mission_planner --ros-args -p num_lines:=6 -p altitude:=15.0
"""

from __future__ import annotations

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Type alias for a 3-D waypoint tuple (x, y, z) in metres (ENU)
Waypoint = Tuple[float, float, float]


def generate_lawnmower_waypoints(
    origin_x: float,
    origin_y: float,
    altitude: float,
    line_spacing: float,
    waypoint_spacing: float,
    num_lines: int,
    line_length: float,
) -> List[Waypoint]:
    """
    Generate a boustrophedon (lawnmower) survey pattern in the ENU frame.

    The pattern sweeps parallel lines along the Y axis, alternating direction
    on each line to minimise turn time.  Lines are separated by `line_spacing`
    along the X axis.

    Args:
        origin_x:         X coordinate of the first waypoint (m).
        origin_y:         Y coordinate of the first waypoint (m).
        altitude:         Constant flight altitude, z in ENU (m).
        line_spacing:     Lateral (X) distance between adjacent survey lines (m).
        waypoint_spacing: Longitudinal (Y) distance between waypoints on a line (m).
        num_lines:        Total number of parallel survey lines.
        line_length:      Length of each survey line along Y (m).

    Returns:
        Ordered list of (x, y, z) tuples representing the full survey path.
    """
    waypoints: List[Waypoint] = []
    num_wps_per_line = int(line_length / waypoint_spacing) + 1

    for i in range(num_lines):
        x = origin_x + i * line_spacing
        # Reverse direction on odd-numbered lines (boustrophedon pattern)
        direction = 1 if i % 2 == 0 else -1
        for j in range(num_wps_per_line):
            y = origin_y + direction * j * waypoint_spacing
            waypoints.append((x, y, altitude))

    return waypoints


def waypoints_to_path(
    waypoints: List[Waypoint],
    node: Node,
    frame_id: str = 'map',
) -> Path:
    """
    Convert a list of (x, y, z) tuples into a nav_msgs/Path message.

    Args:
        waypoints: Ordered list of ENU waypoints.
        node:      The calling ROS 2 node (used for the clock).
        frame_id:  TF frame for the path header (default: 'map').

    Returns:
        A fully populated nav_msgs/Path ready to publish.
    """
    path_msg = Path()
    path_msg.header.stamp = node.get_clock().now().to_msg()
    path_msg.header.frame_id = frame_id

    for (x, y, z) in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0  # identity quaternion — yaw controlled by PX4
        path_msg.poses.append(pose)

    return path_msg


class MissionPlannerNode(Node):
    """
    Plans and continuously publishes a lawnmower survey mission as a ROS 2 Path.

    The node generates waypoints once on startup and re-publishes the Path at
    1 Hz so that the MissionExecutorNode can subscribe at any time and always
    receive a valid plan.
    """

    PUBLISH_RATE_HZ: float = 1.0

    def __init__(self) -> None:
        super().__init__('mission_planner')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('cruise_speed', 3.0)
        self.declare_parameter('line_spacing', 5.0)
        self.declare_parameter('waypoint_spacing', 3.0)
        self.declare_parameter('stationkeep_duration', 3.0)
        self.declare_parameter('num_lines', 4)
        self.declare_parameter('line_length', 24.0)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)

        altitude = self.get_parameter('altitude').value
        cruise_speed = self.get_parameter('cruise_speed').value
        line_spacing = self.get_parameter('line_spacing').value
        waypoint_spacing = self.get_parameter('waypoint_spacing').value
        stationkeep_dur = self.get_parameter('stationkeep_duration').value
        num_lines = self.get_parameter('num_lines').value
        line_length = self.get_parameter('line_length').value
        origin_x = self.get_parameter('origin_x').value
        origin_y = self.get_parameter('origin_y').value

        self.get_logger().info(
            f'MissionPlanner — alt={altitude}m  speed={cruise_speed}m/s  '
            f'line_spacing={line_spacing}m  wp_spacing={waypoint_spacing}m  '
            f'stationkeep={stationkeep_dur}s  lines={num_lines}  '
            f'line_length={line_length}m  origin=({origin_x},{origin_y})'
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._wp_pub = self.create_publisher(Path, '/mission/waypoints', 10)

        # ── Generate waypoints ────────────────────────────────────────────────
        waypoints = generate_lawnmower_waypoints(
            origin_x=origin_x,
            origin_y=origin_y,
            altitude=altitude,
            line_spacing=line_spacing,
            waypoint_spacing=waypoint_spacing,
            num_lines=num_lines,
            line_length=line_length,
        )
        self.get_logger().info(f'Generated {len(waypoints)} waypoints.')
        for idx, (x, y, z) in enumerate(waypoints):
            self.get_logger().debug(f'  WP {idx:03d}: x={x:.1f}  y={y:.1f}  z={z:.1f}')

        # ── Build Path message (built once, re-stamped on each publish) ────────
        self._path_msg = waypoints_to_path(waypoints, self)

        # ── Publish timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._publish_waypoints)
        self.get_logger().info(
            f'MissionPlanner ready — publishing {len(waypoints)} waypoints '
            f'at {self.PUBLISH_RATE_HZ} Hz on /mission/waypoints'
        )

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_waypoints(self) -> None:
        """Re-stamp and publish the pre-built Path message."""
        self._path_msg.header.stamp = self.get_clock().now().to_msg()
        self._wp_pub.publish(self._path_msg)
        self.get_logger().debug(
            f'Published path with {len(self._path_msg.poses)} waypoints.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
