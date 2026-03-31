from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="assurance_harness",
            executable="telemetry_gate_node",
            name="telemetry_gate",
            parameters=[{"input_pose_topic": "/mavros/local_position/pose"}],
        ),
        Node(
            package="assurance_harness",
            executable="risk_model_node",
            name="risk_model"
        ),
        Node(
            package="assurance_harness",
            executable="evidence_logger_node",
            name="evidence_logger"
        ),
        Node(
            package="assurance_harness",
            executable="viz_node",
            name="viz"
        ),
    ])
