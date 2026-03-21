#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from enum import Enum, auto
import math


# ── State machine ────────────────────────────────────────────────────────────
class MissionState(Enum):
    IDLE = auto()
    ARMED = auto()
    TAKEOFF = auto()
    NAVIGATING = auto()
    STATIONKEEPING = auto()
    RTL = auto()
    COMPLETE = auto()
    FAILSAFE = auto()


class MissionExecutorNode(Node):

    def __init__(self):
        super().__init__('mission_executor')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('stationkeep_duration', 3.0)
        self.declare_parameter('acceptance_radius', 0.5)
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('pose_timeout_s', 2.0)
        self.pose_timeout   = self.get_parameter ('pose_timeout_s').value
        self.last_pose_time = None  # set by pose callback

        self.stationkeep_duration = self.get_parameter('stationkeep_duration').value
        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.takeoff_altitude = self.get_parameter('altitude').value

        self.get_logger().info(
            f'MissionExecutor — stationkeep={self.stationkeep_duration}s  '
            f'acceptance_radius={self.acceptance_radius}m  '
            f'takeoff_alt={self.takeoff_altitude}m'
        )

        # ── Internal state ─────────────────────────────────────────────
        self.state = MissionState.IDLE
        self.waypoints = []        # list of PoseStamped
        self.wp_index = 0
        self.current_pose = None   # latest pose from MAVROS
        self.stationkeep_timer = None

        # ── Publishers ─────────────────────────────────────────────────
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.state_pub = self.create_publisher(
            String, '/mission/state', 10)

        # ── Subscribers ────────────────────────────────────────────────
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos
        )

        self.wp_sub = self.create_subscription(
            Path,
            '/mission/waypoints',
            self.waypoints_callback,
            10
        )

        # ── Service ───────────────────────────────────────────────────
        self.start_srv = self.create_service(
            Trigger,
            '/start_mission',
            self.handle_start_mission
        )

        # ── Control loop at 10 Hz ─────────────────────────────────────
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('MissionExecutor ready. Call /start_mission to begin.')

    # ── Callbacks ───────────────────────────────────────────────────
    def pose_callback(self, msg):
        self.current_pose = msg.pose.position
        self.last_pose_time = self.get_clock().now()   #watchdog timestamp

    def waypoints_callback(self, msg):
        if self.state == MissionState.IDLE:
            self.waypoints = msg.poses
            self.get_logger().info(f'Received {len(self.waypoints)} waypoints.')

    def handle_start_mission(self, request, response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Cannot start — currently in state {self.state.name}'
            return response
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints received yet. Is MissionPlannerNode running?'
            return response
        self.transition(MissionState.ARMED)
        response.success = True
        response.message = 'Mission accepted. Arming and taking off.'
        return response

    # ── State helpers ───────────────────────────────────────────────
    def transition(self, new_state):
        self.get_logger().info(f'State: {self.state.name} → {new_state.name}')
        self.state = new_state
        if new_state == MissionState.COMPLETE:
            self.get_logger().info('Mission complete. All waypoints visited.')
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    # ── Geometry helpers ───────────────────────────────────────────
    def distance_to(self, target_pose):
        if self.current_pose is None:
            return float('inf')
        dx = self.current_pose.x - target_pose.pose.position.x
        dy = self.current_pose.y - target_pose.pose.position.y
        dz = self.current_pose.z - target_pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_setpoint(self, pose_stamped):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose = pose_stamped.pose
        self.setpoint_pub.publish(msg)

    def takeoff_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = self.takeoff_altitude
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    # ── Stationkeep logic ──────────────────────────────────────────
    def on_waypoint_reached(self):
        self.get_logger().info(
            f'Waypoint {self.wp_index}/{len(self.waypoints)-1} reached — '
            f'stationkeeping for {self.stationkeep_duration}s'
        )
        self.transition(MissionState.STATIONKEEPING)
        self.stationkeep_timer = self.create_timer(
            self.stationkeep_duration,
            self.on_stationkeep_complete
        )

    def on_stationkeep_complete(self):
        if self.stationkeep_timer is not None:
            self.stationkeep_timer.cancel()
            self.stationkeep_timer = None
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.get_logger().info('All waypoints complete. Returning to launch.')
            self.transition(MissionState.RTL)
        else:
            self.get_logger().info(f'Advancing to waypoint {self.wp_index}.')
            self.transition(MissionState.NAVIGATING)

    def check_pose_watchdog(self):
        """
        Detects pose topic dropout. Transitions to FAILSAFE if the
        pose topic has been silent for longer than pose_timeout_s.
        Only activates when the mission is actively flying.
        """
        active_states = {
            MissionState.TAKEOFF,
            MissionState.NAVIGATING,
            MissionState.STATIONKEEPING,
            MissionState.RTL,
        }
        if self.state not in active_states:
            return  # not flying — watchdog not needed
        if self.last_pose_time is None:
            return  # no pose received yet — normal at startup
        elapsed = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if elapsed > self.pose_timeout:
            self.get_logger().error(
                f'WATCHDOG TRIGGERED: no pose for {elapsed:.2f}s '
                f'(timeout={self.pose_timeout}s). Entering FAILSAFE.'
            )
            self.transition(MissionState.FAILSAFE)

    # ── Main control loop ──────────────────────────────────────────
    def control_loop(self):
        self.check_pose_watchdog()  # FIRST — always check before acting
        self.publish_state()

        if self.state == MissionState.IDLE:
            return

        elif self.state == MissionState.ARMED:
            self.takeoff_setpoint()
            if self.current_pose is not None:
                self.transition(MissionState.TAKEOFF)

        elif self.state == MissionState.TAKEOFF:
            self.takeoff_setpoint()
            if self.current_pose is not None:
                dz = abs(self.current_pose.z - self.takeoff_altitude)
                if dz < self.acceptance_radius:
                    self.get_logger().info('Takeoff altitude reached.')
                    self.wp_index = 0
                    self.transition(MissionState.NAVIGATING)

        elif self.state == MissionState.NAVIGATING:
            target = self.waypoints[self.wp_index]
            self.publish_setpoint(target)
            if self.distance_to(target) < self.acceptance_radius:
                self.on_waypoint_reached()

        elif self.state == MissionState.STATIONKEEPING:
            target = self.waypoints[self.wp_index]
            self.publish_setpoint(target)

        elif self.state == MissionState.RTL:
            rtl_pose = PoseStamped()
            rtl_pose.pose.position.x = 0.0
            rtl_pose.pose.position.y = 0.0
            rtl_pose.pose.position.z = self.takeoff_altitude
            rtl_pose.pose.orientation.w = 1.0
            self.publish_setpoint(rtl_pose)
            if self.current_pose is not None:
                dist = math.sqrt(self.current_pose.x**2 + self.current_pose.y**2)
                if dist < self.acceptance_radius:
                    self.transition(MissionState.COMPLETE)

        elif self.state == MissionState.COMPLETE:
            pass
        
        elif self.state == MissionState.FAILSAFE:
            # Stop publishing setpoints — let PX4 offboard loss failsafe take over.
            # PX4 will detect setpoint dropout and auto-land via COM_OF_LOSS_T.
            pass  # logged once in transition()


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
