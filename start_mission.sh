#!/bin/bash
# ─────────────────────────────────────────────────────────────
# start_mission.sh
# Full mission startup script for ROS2 + PX4 + MAVROS
# Usage: bash start_mission.sh
# ─────────────────────────────────────────────────────────────

echo "=== Disabling NTP to prevent time jumps ==="
sudo timedatectl set-ntp false

echo "=== Killing any existing processes ==="
pkill -f px4
pkill -f gzserver
pkill -f gzclient
pkill -f mavros
pkill -f mission_planner
pkill -f mission_executor
pkill -f telemetry_logger
sleep 5

echo "=== Starting PX4 + Gazebo in new terminal ==="
gnome-terminal --title="PX4-Gazebo" -- bash -c "
    cd ~/PX4-Autopilot && \
    source Tools/simulation/gazebo-classic/setup_gazebo.bash \
    \$(pwd) \$(pwd)/build/px4_sitl_default && \
    Tools/simulation/gazebo-classic/sitl_run.sh \
    \$(pwd)/build/px4_sitl_default/bin/px4 \
    none iris empty \
    \$(pwd) \$(pwd)/build/px4_sitl_default; \
    exec bash"

echo "=== Waiting 30s for PX4 + Gazebo to be ready ==="
sleep 30

echo "=== Starting MAVROS in new terminal ==="
gnome-terminal --title="MAVROS" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    ros2 launch mavros px4.launch \
    fcu_url:=udp://:14540@127.0.0.1:14557; \
    exec bash"

echo "=== Waiting 20s for MAVROS to fully connect ==="
sleep 20

echo "=== Starting mission_planner in new terminal ==="
gnome-terminal --title="MissionPlanner" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    cd ~/drone_mission && \
    python3 mission_planner.py; \
    exec bash"

sleep 5

echo "=== Starting mission_executor in new terminal ==="
gnome-terminal --title="MissionExecutor" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    cd ~/drone_mission && \
    python3 mission_executor.py; \
    exec bash"

sleep 5

echo "=== Starting telemetry_logger in new terminal ==="
gnome-terminal --title="TelemetryLogger" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    cd ~/drone_mission && \
    python3 telemetry_logger.py; \
    exec bash"

echo "=== Starting state monitor in new terminal ==="
gnome-terminal --title="StateMonitor" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    ros2 topic echo /mission/state; \
    exec bash"

echo "=== Waiting 8s for setpoints to stream ==="
sleep 8

echo "=== Triggering mission via /start_mission service ==="
source /opt/ros/humble/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger '{}'

sleep 2

echo "=== Setting OFFBOARD mode ==="
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{custom_mode: 'OFFBOARD'}" > /dev/null 2>&1
echo "=== OFFBOARD mode set ==="

sleep 3

echo "=== Arming drone ==="
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
    "{value: true}" > /dev/null 2>&1
echo "=== Arm command sent ==="

echo ""
echo "=== All systems started! ==="
echo "=== Watch the Gazebo window for takeoff ==="
echo "=== To stop everything run: bash stop_mission.sh ==="
