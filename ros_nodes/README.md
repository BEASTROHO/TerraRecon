# ROS Nodes

This folder contains Python-based ROS nodes for rover control.

## Nodes

- `sensor_node.py`: Publishes simulated or real sensor data to `/sensor_data`
- `ai_node.py`: Subscribes to `/sensor_data`, publishes decisions to `/motor_commands`
- `motor_node.py`: Subscribes to `/motor_commands`, executes motor actions via `rover_control`

## Topics

- `/sensor_data` (`std_msgs/Float32`)
- `/motor_commands` (`std_msgs/String`)

## Usage

These nodes are designed to be launched from a ROS package (e.g., `rover_system`) using a launch file.
Ensure they are executable:

```bash
chmod +x ros_nodes/*.py
