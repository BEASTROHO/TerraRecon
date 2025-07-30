# 🛰️ TerraRecon

**Autonomous Terrain Reconnaissance Rover**

TerraRecon is a modular, autonomous rover designed for terrain mapping, object detection, and remote exploration in challenging environments. It integrates GPS, LiDAR, IMU, and IR camera modules with Python and ROS to deliver real-time navigation, sensor fusion, and AI-based vision. The system supports RF telemetry for remote control and monitoring, making it suitable for disaster response, planetary research, and autonomous scouting missions.

---

## 🚀 Features

- **Autonomous Navigation** using GPS and sensor fusion
- **Real-Time Object Detection** with YOLOv5
- **Terrain Mapping** via LiDAR and IMU integration
- **RF Communication** for remote telemetry and control
- **Modular ROS Nodes** for scalable deployment
- **Energy-Efficient Design** with solar-assisted power

---

## 📁 Project Structure

```
TerraRecon/
├── README.md
├── LICENSE
├── requirements.txt
├── rover_control/
│   ├── motor_driver.py
│   ├── imu_reader.py
│   ├── lidar_processor.py
│   ├── camera_stream.py
│   └── main_controller.py
├── ai_module/
│   ├── yolov5_inference.py
│   ├── object_detection_utils.py
│   └── model/
│       └── yolov5s.pt
├── ros_nodes/
│   ├── navigation_node.py
│   ├── sensor_fusion_node.py
│   └── rf_comm_node.py
└── config/
    ├── system_config.yaml
    └── calibration_data.json
```

---

## ⚙️ Installation

```bash
git clone https://github.com/your-username/TerraRecon.git
cd TerraRecon
pip install -r requirements.txt
```

---

## 🧠 Run the Rover

```bash
python rover_control/main_controller.py
```

To run individual modules:

```bash
python ai_module/yolov5_inference.py
rosrun ros_nodes navigation_node.py
```

---

## 📜 License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## 📚 Citation

If you're using TerraRecon for academic purposes, please cite:

```
Roho, "TerraRecon: Autonomous Terrain Reconnaissance Rover", IEEE Submission, 2025.
```

---

## 🤝 Contributions

Pull requests are welcome. For major changes, please open an issue first to discuss what you’d like to modify.

---

## 📬 Contact

For questions or collaboration inquiries, reach out via GitHub or email hemamahendiran0@gmail.com .
![Python](https://img.shields.io/badge/Python-3.9-blue)
![ROS](https://img.shields.io/badge/ROS-Noetic-green)
![License](https://img.shields.io/badge/License-MIT-yellow)


