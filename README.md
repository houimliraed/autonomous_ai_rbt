# AutonomBot - Autonomous Navigation Robot

[![ROS 2 Build](https://github.com/houimliraed/autonomous_ai_rbt/workflows/ROS%202%20Build%20and%20Test/badge.svg)](https://github.com/houimliraed/autonomous_ai_rbt/actions)

An autonomous mobile robot (AMR) built with ROS 2 Humble for indoor navigation using LiDAR, camera vision, and ultrasonic sensors as a part of University Project. Thanks to the team for designing the hardware and thanks to AntoBrandi for the full ROS2 course.

## 📹 Video Demo

<!-- Add your video demo here -->
*Video demo will be available soon*

## Features

- **Autonomous Navigation**: Path planning and obstacle avoidance using Nav2
- **Multi-Sensor Fusion**: LiDAR, RealSense camera, and ultrasonic sensors
- **Computer Vision**: YOLOv8 object detection and recognition
- **SLAM Mapping**: Real-time mapping and localization
- **Hardware Support**: Optimized for NVIDIA Jetson Xavier AGX

## Hardware Requirements

- **Compute**: NVIDIA Jetson Xavier AGX or compatible
- **LiDAR**: RPLiDAR A1/A2 or compatible
- **Camera**: Intel RealSense D435i or D455
- **Sensors**: Ultrasonic sensors for proximity detection
- **Motors**: Differential drive system with encoders

## Quick Start

### Simulation

1. **Clone and build the project:**
   ```bash
   git clone https://github.com/houimliraed/autonomous_ai_rbt.git
   cd autonomous_ai_rbt
   colcon build
   source install/setup.bash
   ```

2. **Launch simulation:**
   ```bash
   ros2 launch autonombot_launch simulated_robot.launch.py
   ```

3. **Open RViz for visualization:**
   ```bash
   rviz2 -d src/autonombot_launch/config/autonombot.rviz
   ```

### Real Robot (Jetson Xavier)

1. **Setup on Jetson:**
   ```bash
   # Install ROS 2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop
   
   # Clone and build
   git clone https://github.com/houimliraed/autonomous_ai_rbt.git
   cd autonomous_ai_rbt
   colcon build
   source install/setup.bash
   ```

2. **Launch real robot:**
   ```bash
   ros2 launch autonombot_launch real_robot.launch.py
   ```

3. **Start navigation:**
   ```bash
   ros2 launch autonombot_navigation navigation.launch.py
   ```

## Package Structure

- `autonombot_launch` - Launch files and configurations
- `autonombot_drivers` - Hardware drivers and interfaces
- `autonombot_description` - Robot URDF and simulation models
- `autonombot_firmware` - Low-level hardware control
- `autonombot_localization` - Localization and SLAM
- `autonombot_mapping` - Map creation and management
- `autonombot_navigation` - Path planning and navigation
- `autonombot_planning` - High-level mission planning
- `autonombot_vision` - Computer vision and object detection

## Usage

### Basic Navigation
Send navigation goals through RViz or use the command line:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

### Manual Control
Control the robot manually during testing:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Mapping
Create a new map of your environment:
```bash
ros2 launch autonombot_mapping slam.launch.py
```

## Dependencies

- ROS 2 Humble
- Nav2 Navigation Stack
- Gazebo Simulation
- OpenCV 4.x
- YOLOv8 (Ultralytics)
- PCL (Point Cloud Library)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
