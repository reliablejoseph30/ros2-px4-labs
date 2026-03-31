from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="assurance_harness",
            executable="fault_injector_node",
            name="fault_injector",
            parameters=[
                {"drop_rate": 0.3},
                {"input_pose_topic": "/mavros/local_position/pose"},
                {"output_pose_topic": "/assurance/pose_injected"},
            ],
        ),
        Node(
            package="assurance_harness",
            executable="telemetry_gate_node",
            name="telemetry_gate",
            parameters=[{"input_pose_topic": "/assurance/pose_injected"}],
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
