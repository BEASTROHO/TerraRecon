 --------------------------------🛰️ TerraRecon: Autonomous Terrain Reconnaissance Rover--------------------------------
TerraRecon is a compact, autonomous rover designed for terrain mapping and object detection in remote or hazardous environments. The system integrates GPS, LiDAR, IMU, and IR camera modules to enable real-time navigation, obstacle avoidance, and environmental analysis. Built with modular hardware and intelligent software, TerraRecon is suitable for applications ranging from disaster response to planetary exploration.
The rover’s control architecture is developed in Python, with ROS nodes managing sensor fusion, navigation, and telemetry. GPS provides geolocation, while LiDAR and IMU data are combined to generate accurate terrain maps. An onboard IR camera captures visual data, which is processed using a lightweight YOLOv5 model for object detection. The AI module identifies terrain features and obstacles, allowing the rover to adjust its path dynamically.
RF communication enables remote monitoring and control, transmitting telemetry data and receiving commands from a ground station. The system is designed for energy efficiency, powered by a solar-assisted battery unit to support extended missions.
This repository includes all source code, configuration files, and documentation required to replicate or extend the project. Key directories include:
- rover_control/: Motor drivers, sensor interfaces, and main controller
- ai_module/: YOLOv5 inference and object detection utilities
- ros_nodes/: ROS modules for navigation and sensor fusion
- config/: System settings and calibration data
- docs/: Architecture diagrams and technical documentation
To get started, clone the repository and install dependencies using requirements.txt. Run the main controller to initiate autonomous operation.
TerraRecon is released under the MIT License to encourage academic collaboration and open-source development. Contributions and forks are welcome.
