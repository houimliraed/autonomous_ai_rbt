# 🤖 AutonomBot

**An intelligent autonomous mobile robot built with ROS 2 for indoor navigation and obstacle avoidance**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson_Xavier-green.svg)](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit)
[![License](https://img.shields.io/badge/License-Apache_2.0-orange.svg)](LICENSE)
[![Build Status](https://github.com/houimliraed/autonomous_ai_rbt/workflows/CI/badge.svg)](https://github.com/houimliraed/autonomous_ai_rbt/actions)

## 🚀 Overview

AutonomBot is a sophisticated autonomous mobile robot designed for indoor environments. It combines multiple sensors and advanced algorithms to navigate safely while avoiding obstacles in real-time.

**Major Contributions & Enhancements:**
- 🤖 **YOLOv8 Integration** - Custom object detection and obstacle avoidance
- 🔧 **Jetson Xavier Optimization** - Performance-tuned for embedded deployment  
- 🛠️ **Hardware Interface Development** - Custom firmware and sensor fusion
- 📊 **System Validation** - Comprehensive testing and performance monitoring
- 🗺️ **Navigation Enhancement** - Advanced behavior trees and path planning

### ✨ Key Features

- 🧭 **Autonomous Navigation** - SLAM-based mapping and path planning
- 👁️ **Multi-Sensor Fusion** - LiDAR, Camera, and Ultrasonic sensors
- 🎯 **Obstacle Avoidance** - Real-time object detection with YOLOv8
- 🗺️ **SLAM Mapping** - Simultaneous localization and mapping
- 🔧 **Hardware Interface** - Direct motor control and sensor integration
- 📊 **Real-time Monitoring** - System performance and sensor validation

## 🏗️ System Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Sensors   │───▶│   Fusion    │───▶│ Navigation  │
│ LiDAR+Cam+  │    │   Layer     │    │   Stack     │
│ Ultrasonic  │    │             │    │             │
└─────────────┘    └─────────────┘    └─────────────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Vision    │    │   Mapping   │    │   Control   │
│  Processing │    │    SLAM     │    │  Hardware   │
│   YOLOv8    │    │   & Localization │ │  Interface  │
└─────────────┘    └─────────────┘    └─────────────┘
```

## 📦 Package Structure

| Package | Description |
|---------|-------------|
| `autonombot_launch` | Launch files and system startup |
| `autonombot_drivers` | Robot control and kinematics |
| `autonombot_description` | URDF models and robot description |
| `autonombot_firmware` | Hardware interfaces and drivers |
| `autonombot_localization` | Localization algorithms |
| `autonombot_mapping` | SLAM and map generation |
| `autonombot_navigation` | Path planning and navigation |
| `autonombot_vision` | Computer vision and object detection |

## 🛠️ Hardware Requirements

- **Computer**: NVIDIA Jetson Xavier AGX (or compatible)
- **LiDAR**: 360° scanning laser rangefinder
- **Camera**: Intel RealSense or compatible RGB-D camera
- **Sensors**: HC-SR04 ultrasonic sensors
- **Motors**: Differential drive system
- **Microcontroller**: Arduino for motor control

## ⚡ Quick Start

### 1. Prerequisites

```bash
# Install ROS 2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-nav2-* ros-humble-slam-toolbox
```

### 2. Build the Project

```bash
```bash
git clone https://github.com/houimliraed/autonomous_ai_rbt.git
cd autonomous_ai_rbt
```

# Install Python dependencies
pip install -r requirements.txt

# Build with colcon
colcon build

# Source the workspace
source install/setup.bash
```

### 3. Launch the Robot

```bash
# Launch complete system
ros2 launch autonombot_launch real_robot.launch.py

# Launch simulation only
ros2 launch autonombot_launch simulated_robot.launch.py
```

### 4. Validate Hardware

```bash
# Check sensor connectivity
python3 src/autonombot_firmware/scripts/validate_hardware.py
```

## 🎮 Usage

### Navigation Commands

```bash
# Start navigation
ros2 launch autonombot_navigation navigation.launch.py

# Set a goal pose
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

### Mapping

```bash
# Start SLAM mapping
ros2 launch autonombot_mapping slam.launch.py

# Save the map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Vision System

```bash
# Start object detection
ros2 launch autonombot_vision vision.launch.py

# View detected obstacles
ros2 topic echo /vision_obstacles
```

## 📊 System Performance

- **Navigation Accuracy**: ±5cm positioning precision
- **Obstacle Detection**: Real-time at 30 FPS
- **Mapping Rate**: 10 Hz SLAM updates
- **Response Time**: <100ms obstacle avoidance
- **Battery Life**: 2-4 hours continuous operation

## 🔧 Configuration

Key configuration files:
- `config/navigation_params.yaml` - Navigation parameters
- `config/sensor_fusion.yaml` - Sensor fusion settings
- `config/robot_description.yaml` - Robot physical parameters

## 🐛 Troubleshooting

### Common Issues

**LiDAR not detected:**
```bash
# Check USB connection
lsusb | grep -i lidar

# Check permissions
sudo chmod 666 /dev/ttyUSB*
```

**Navigation not working:**
```bash
# Verify transforms
ros2 run tf2_tools view_frames

# Check topics
ros2 topic list | grep nav
```

**Build failures:**
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 📈 Development Status

**Enhanced Features (by Ray):**
- [x] **YOLOv8 Vision Integration** - Custom object detection pipeline
- [x] **Jetson Xavier Optimization** - Performance tuning and deployment
- [x] **Advanced Sensor Fusion** - Multi-sensor integration and validation
- [x] **Enhanced Navigation** - Improved behavior trees and recovery strategies
- [x] **Hardware Interface** - Custom firmware and motor control optimization
- [x] **System Validation** - Comprehensive testing and monitoring framework

**Foundation Framework (Antonio Brandi):**
- [x] **Core Navigation** - ROS 2 Nav2 stack integration
- [x] **SLAM Mapping** - Real-time map generation
- [x] **Robot Description** - URDF modeling and basic kinematics
- [x] **Package Structure** - Modular ROS 2 workspace organization

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

**Project Development:**
- **Ray** - YOLOv8 integration, system optimization, and deployment enhancements
- **Antonio Brandi** - Original project foundation, ROS 2 framework structure, and base navigation implementation

**Technical Stack:**
- ROS 2 Community for the robust robotics framework
- Nav2 Team for the navigation stack foundation
- Ultralytics for YOLOv8 object detection model
- OpenCV for computer vision processing
- NVIDIA for Jetson platform optimization and CUDA support

**Academic Context:**
This project builds upon Antonio Brandi's foundational ROS 2 autonomous robot implementation, with significant enhancements including custom vision integration, hardware optimization, sensor fusion improvements, and real-world validation for practical deployment.

---

**🔗 Quick Links:**
- [Technical Documentation](AUTONOMBOT_TECHNICAL_REPORT.md)
- [System Diagrams](SYSTEM_DIAGRAMS.md)
- [API Reference](docs/api/)
- [Hardware Setup Guide](docs/hardware/)

---

*Built with ❤️ for autonomous robotics*
