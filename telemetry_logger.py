#!/usr/bin/env python3
"""
telemetry_logger.py
====================
ROS 2 node that logs mission telemetry to a timestamped CSV file.

Subscribes to:
  /mission/state          (std_msgs/String)       — mission state machine output
  /mavros/local_position/pose (geometry_msgs/PoseStamped) — ENU position from MAVROS

Logs a row at 2 Hz containing: ROS timestamp, mission state, and x/y/z position.
The log file is written to ~/mission_log_<YYYYMMDD_HHMMSS>.csv.

Usage:
    ros2 run ros2_px4_labs telemetry_logger
"""

import csv
import os
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String


# CSV column headers
CSV_COLUMNS = ['ros_time_s', 'mission_state', 'x_m', 'y_m', 'z_m']

# Log rate (Hz)
LOG_RATE_HZ = 2.0


class TelemetryLoggerNode(Node):
    """
    Logs timestamped pose and mission-state data to CSV for post-mission KPI analysis.

    The node is deliberately lightweight — it only reads and writes; all
    mission logic lives in MissionExecutorNode.
    """

    def __init__(self) -> None:
        super().__init__('telemetry_logger')

        # ── Output file ──────────────────────────────────────────────────────
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.expanduser(f'~/mission_log_{timestamp}.csv')
        self._csv_file = open(log_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(CSV_COLUMNS)

        # ── Internal state ───────────────────────────────────────────────────
        self._current_state: str = 'UNKNOWN'
        self._current_pose: Optional[Point] = None
        self._rows_written: int = 0

        # ── QoS — match MAVROS best-effort profile ───────────────────────────
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            String,
            '/mission/state',
            self._state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            mavros_qos,
        )

        # ── Logging timer ────────────────────────────────────────────────────
        self.create_timer(1.0 / LOG_RATE_HZ, self._log_row)

        self.get_logger().info(
            f'TelemetryLogger started — writing to {log_path} at {LOG_RATE_HZ} Hz'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _state_callback(self, msg: String) -> None:
        """Update the cached mission state on each publication."""
        self._current_state = msg.data

    def _pose_callback(self, msg: PoseStamped) -> None:
        """Cache the latest ENU position from MAVROS."""
        self._current_pose = msg.pose.position

    # ── Logging ──────────────────────────────────────────────────────────────

    def _log_row(self) -> None:
        """Write one telemetry row to CSV; skips silently if no pose yet."""
        if self._current_pose is None:
            return

        ros_time_s = self.get_clock().now().nanoseconds / 1e9
        self._writer.writerow([
            f'{ros_time_s:.3f}',
            self._current_state,
            f'{self._current_pose.x:.3f}',
            f'{self._current_pose.y:.3f}',
            f'{self._current_pose.z:.3f}',
        ])
        self._csv_file.flush()
        self._rows_written += 1

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """Flush and close the CSV file before shutting down."""
        self._csv_file.close()
        self.get_logger().info(
            f'TelemetryLogger shutdown — {self._rows_written} rows written.'
        )
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
