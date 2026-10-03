# ROS 2 PX4 Labs — Oloruntoba Joseph

> MSc Advanced Drone Technology · University of the West of Scotland  
> Practical lab work covering ROS 2 integration with PX4, autonomous mission execution, fault injection, and telemetry analysis.

---

## Overview

This repository contains lab assignments completed as part of the MSc Advanced Drone Technology programme at UWS. Each lab builds progressively toward a full autonomous drone system, using ROS 2 (Humble) and the PX4 autopilot stack in a simulated environment (Gazebo / SITL).

---

## Labs

### Lab 1 — Offboard Control
- Implemented an offboard setpoint node (`offboard_setpoint.py`) to command a drone to ascend to 5 m and hold position
- Established the ROS 2 ↔ PX4 communication bridge via uXRCE-DDS

### Lab 3 — ROS 2 Assurance Harness with Fault Injection
- Built a full assurance harness (`assurance_harness/`) to monitor mission-critical ROS 2 topics
- Implemented fault injection scenarios (FM-01 to FM-04) covering GPS denial, link loss, and motor failure modes
- Validated system behaviour and logged responses for post-mission analysis

### Lab 4 — Autonomous Mission Execution & Telemetry
- Developed `mission_planner.py` and `mission_executor.py` for waypoint-based autonomous flight
- Logged mission telemetry to CSV (`mission_log_20260320_175113.csv`) and generated altitude and ground track plots
- Implemented a pose watchdog and FAILSAFE state machine (`mission_executor.py`)
- Ran 16 unit tests across mission nodes to validate behaviour under nominal and degraded conditions

---

## Repository Structure

```
ros2-px4-labs/
├── assurance_harness/        # Lab 3: fault injection & monitoring nodes
├── src/offboard_control/     # Lab 1 & 4: ROS 2 offboard control package
├── mission_planner.py        # Waypoint mission planning node
├── mission_executor.py       # Mission execution with FAILSAFE state machine
├── mission_log_*.csv         # Recorded telemetry data
├── plot_altitude.png         # Altitude profile plot
├── plot_groundtrack.png      # Ground track plot
├── plot_mission.py           # Plotting utilities
├── analyse_kpis.py           # KPI analysis from mission logs
├── telemetry_logger.py       # Real-time telemetry logging node
├── lab_drone_setup.sh        # Automated lab environment setup script
└── start_mission.sh          # Mission launch script
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| Autopilot | PX4 SITL (v1.14) |
| Middleware | ROS 2 Humble |
| Bridge | uXRCE-DDS / Micro-XRCE-DDS Agent |
| Simulation | Gazebo Classic |
| Language | Python 3.10, Shell |
| Analysis | pandas, matplotlib |

---

## Author

**Oloruntoba Joseph** — MSc Advanced Drone Technology, UWS  
GitHub: [reliablejoseph30](https://github.com/reliablejoseph30)  
LinkedIn: [linkedin.com/in/oloruntoba-joseph](https://linkedin.com/in/oloruntoba-joseph)
