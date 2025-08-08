# 🚀 AUTONOMBOT JETSON XAVIER DEPLOYMENT CHECKLIST
## Complete Pre-Deployment Analysis & Setup Guide

**Date:** August 8, 2025  
**Target Platform:** NVIDIA Jetson Xavier AGX  
**Hardware:** LiDAR, RealSense Camera, Ultrasonic Sensors, Differential Drive  

---

## ✅ PROJECT STATUS OVERVIEW

### 🎯 **SIMULATION READINESS: COMPLETE ✓**
- [x] All packages renamed and references fixed
- [x] Improved LiDAR performance (20Hz update rate)
- [x] Enhanced RViz visualization with camera integration
- [x] Speed optimizations for better obstacle detection
- [x] Costmap configurations tuned for reliable navigation

### 🎯 **HARDWARE DEPLOYMENT READINESS: READY ✓**
- [x] Real robot launch files configured
- [x] Hardware validation scripts available
- [x] Jetson Xavier setup automation ready
- [x] All sensor drivers integrated

---

## 🔧 PRE-DEPLOYMENT HARDWARE CHECKLIST

### **1. Jetson Xavier System Setup**

#### **Required Software Stack:**
```bash
# Core system requirements
- Ubuntu 20.04 LTS (JetPack 5.x)
- ROS 2 Humble Hawksbill
- Python 3.8+
- CUDA 11.4+ (included with JetPack)
```

#### **GPIO and Permissions:**
```bash
# GPIO libraries for ultrasonic sensors
- Jetson.GPIO
- Python RPi.GPIO compatibility layer
- gpiod libraries for modern GPIO access
```

#### **Hardware Connections Verification:**
- [ ] **HC-SR04 Ultrasonic:** GPIO 18 (TRIG), GPIO 24 (ECHO)
- [ ] **RPLiDAR A1/A2:** USB connection with 5V power
- [ ] **RealSense Camera:** USB 3.0 port
- [ ] **Arduino Motor Controller:** USB/Serial (/dev/ttyACM0)
- [ ] **Power Supply:** Adequate power for all components

### **2. Dependencies Installation**

#### **ROS 2 Package Dependencies:**
```bash
# Navigation and control
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-joint-state-publisher-gui

# Sensor drivers
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-rplidar-ros

# Control and hardware interface
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

#### **Vision System Dependencies:**
```bash
# Python packages for YOLO
pip3 install ultralytics>=8.0.0
pip3 install opencv-python>=4.5.0
pip3 install torch torchvision
pip3 install numpy>=1.19.0
```

#### **Hardware Interface Libraries:**
```bash
# GPIO control
sudo pip3 install Jetson.GPIO
sudo pip3 install RPi.GPIO

# Serial communication
sudo apt install python3-serial
```

---

## 🛠️ DEPLOYMENT PROCEDURE

### **Step 1: Automated Setup (Recommended)**
```bash
# 1. Transfer workspace to Jetson Xavier
scp -r autonombot_ws jetson_user@jetson_ip:~/

# 2. SSH into Jetson Xavier
ssh jetson_user@jetson_ip

# 3. Run automated setup script
cd ~/autonombot_ws/src/autonombot_firmware/scripts
sudo chmod +x jetson_setup.sh
./jetson_setup.sh
```

**Setup Script Actions:**
- [x] System package updates
- [x] ROS 2 Humble installation verification
- [x] Hardware dependencies installation
- [x] GPIO permissions configuration
- [x] USB device rules setup
- [x] Performance optimization settings

### **Step 2: Workspace Build**
```bash
# Navigate to workspace
cd ~/autonombot_ws

# Build all packages
colcon build

# Source workspace
source install/setup.bash
echo "source ~/autonombot_ws/install/setup.bash" >> ~/.bashrc
```

### **Step 3: Hardware Validation**
```bash
# Run comprehensive hardware validation
ros2 run autonombot_firmware validate_hardware.py
```

**Expected Validation Results:**
```
HARDWARE VALIDATION COMPLETE
ULTRASONIC SENSOR: ✓ OPERATIONAL (Range: 0.02-4.00m)
LIDAR SENSOR: ✓ OPERATIONAL (360° scans at 10Hz)
CAMERA_COLOR: ✓ OPERATIONAL (640x480@30fps)
CAMERA_DEPTH: ✓ OPERATIONAL (640x480@30fps)
🎉 ALL SENSORS OPERATIONAL - ROBOT READY FOR DEPLOYMENT!
```

### **Step 4: Real Robot Launch**
```bash
# Launch complete autonomous robot system
ros2 launch autonombot_launch real_robot.launch.py

# Alternative: Launch with SLAM enabled
ros2 launch autonombot_launch real_robot.launch.py use_slam:=true

# Alternative: Launch without advanced vision
ros2 launch autonombot_launch real_robot.launch.py use_advanced_vision:=false
```

---

## 🎛️ CONFIGURATION FILES FOR REAL HARDWARE

### **1. LiDAR Configuration (`rplidar_a1.yaml`)** ✓
```yaml
rplidar_node:
  ros__parameters:
    channel_type: serial
    serial_port: /dev/rplidar        # Device symlink
    serial_baudrate: 115200
    frame_id: laser_link
    inverted: false
    angle_compensate: true
    scan_mode: Sensitivity           # Optimized for obstacle detection
```

### **2. Camera Configuration (RealSense)** ✓
```yaml
# Configured in launch file
enable_color: true
enable_depth: true
depth_module.profile: "640,480,30"
rgb_camera.profile: "640,480,30"
```

### **3. Motor Control Configuration** ✓
```yaml
# autonombot_drivers.yaml
autonombot_drivers:
  ros__parameters:
    # Reduced speeds for real hardware safety
    max_linear_velocity: 0.3         # m/s (reduced from 0.7)
    max_angular_velocity: 2.0        # rad/s (reduced from 8.5)
    wheel_separation: 0.160          # m
    wheel_radius: 0.033              # m
```

---

## 🔍 MONITORING AND DIAGNOSTICS

### **Real-Time System Monitoring**
```bash
# Monitor all sensor topics
ros2 topic list

# Check sensor data rates
ros2 topic hz /scan
ros2 topic hz /camera/color/image_raw
ros2 topic hz /ultrasonic_range

# Monitor navigation status
ros2 topic echo /amcl_pose
ros2 topic echo /cmd_vel

# System diagnostics
ros2 topic echo /diagnostics
```

### **Performance Monitoring**
```bash
# Check CPU and GPU usage
htop
tegrastats

# Monitor ROS 2 node performance
ros2 run rqt_top rqt_top

# Network monitoring (if using remote monitoring)
ros2 run rqt_graph rqt_graph
```

---

## 🚨 SAFETY CONSIDERATIONS

### **Emergency Procedures** ⚠️
1. **Emergency Stop:** Press joystick button or send `geometry_msgs/Twist` with all zeros
2. **Hardware Kill:** Power button on Jetson Xavier
3. **Remote Emergency:** SSH kill command: `sudo pkill -f autonombot`

### **Safety Features Implemented** ✅
- [x] Collision avoidance with LiDAR and ultrasonic sensors
- [x] Reduced speed limits for real hardware operation
- [x] Joystick override capability
- [x] Automatic emergency braking on sensor failure
- [x] Hardware watchdog timers

---

## 🐛 TROUBLESHOOTING GUIDE

### **Common Issues and Solutions**

#### **1. LiDAR Connection Issues**
```bash
# Check device detection
ls /dev/ttyUSB*
ls /dev/rplidar

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER

# Restart LiDAR node
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0
```

#### **2. Camera Connection Issues**
```bash
# Check RealSense detection
lsusb | grep Intel
rs-enumerate-devices

# Install RealSense SDK if needed
sudo apt install librealsense2-dkms librealsense2-utils
```

#### **3. GPIO Permission Issues**
```bash
# Fix GPIO permissions
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### **4. Performance Issues**
```bash
# Enable Jetson performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Check thermal throttling
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

---

## 📋 FINAL DEPLOYMENT CHECKLIST

### **Before Lab Visit:**
- [ ] Backup current working simulation
- [ ] Prepare workspace transfer (USB drive or GitHub)
- [ ] Print this deployment guide
- [ ] Verify all hardware components available

### **At Lab - Hardware Setup:**
- [ ] Connect all sensors (ultrasonic, LiDAR, camera)
- [ ] Verify power connections
- [ ] Connect Arduino motor controller
- [ ] Test basic connectivity (SSH, power)

### **At Lab - Software Deployment:**
- [ ] Transfer workspace to Jetson Xavier
- [ ] Run automated setup script
- [ ] Build workspace (`colcon build`)
- [ ] Source workspace (`source install/setup.bash`)
- [ ] Run hardware validation
- [ ] Test basic movement with joystick
- [ ] Test autonomous navigation

### **Final Validation:**
- [ ] All sensors publishing data
- [ ] Robot responds to joystick commands
- [ ] Autonomous navigation functional
- [ ] Emergency stop procedures working
- [ ] Performance acceptable (no thermal issues)

---

## 🎯 SUCCESS CRITERIA

### **Deployment Successful When:**
✅ All hardware validation tests pass  
✅ Robot moves smoothly with joystick control  
✅ LiDAR shows obstacle detection in RViz  
✅ Camera stream visible and processing objects  
✅ Autonomous navigation completes simple path  
✅ Emergency stop functions properly  
✅ System runs stable for >10 minutes  

---

## 📞 SUPPORT CONTACT

**For Issues During Deployment:**
- Hardware validation logs: `~/autonombot_ws/logs/`
- ROS 2 logs: `ros2 log view`
- System logs: `journalctl -f`
- Performance monitoring: `tegrastats`

**Critical Commands for Quick Recovery:**
```bash
# Restart all ROS 2 nodes
sudo pkill -f ros2
ros2 launch autonombot_launch real_robot.launch.py

# Reset hardware interfaces
sudo systemctl restart serial-getty@ttyACM0.service

# Emergency workspace rebuild
cd ~/autonombot_ws && colcon build --packages-select autonombot_firmware autonombot_launch
```

---

**🎉 DEPLOYMENT CONFIDENCE: HIGH**
**The project is well-prepared for real hardware deployment with comprehensive safety measures and validation procedures in place.**
