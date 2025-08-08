# Autonombot Firmware - Jetson Xavier Deployment Guide

## 🚀 Production-Ready Firmware for Real Hardware

This firmware package is specifically optimized for deployment on NVIDIA Jetson Xavier with real hardware sensors:
- **HC-SR04 Ultrasonic Sensor** (GPIO pins 18/24)
- **RealSense 3D Camera** (USB 3.0)
- **RPLiDAR A1** (USB/Serial)
- **Arduino-based Motor Control** (Serial/USB)

## 📋 Prerequisites

### Hardware Requirements
- NVIDIA Jetson Xavier (NX or AGX)
- HC-SR04 Ultrasonic Sensor connected to GPIO pins 18 (TRIG) and 24 (ECHO)
- Intel RealSense D435i or similar 3D camera
- RPLiDAR A1 or A2
- Arduino Uno/Nano with motor driver circuit
- MicroSD card (32GB+ recommended)
- USB hub (if needed for multiple devices)

### Software Requirements
- Ubuntu 20.04 LTS (JetPack 5.x)
- ROS 2 Humble
- Python 3.8+
- Required GPIO libraries

## 🔧 Installation

### Step 1: Automated Setup (Recommended)
```bash
# Download and run the automated setup script
cd ~/autonombot_ws/src/autonombot_firmware/scripts
sudo chmod +x jetson_setup.sh
./jetson_setup.sh
```

The setup script will:
- Install all required packages and dependencies
- Configure GPIO permissions
- Set up device permissions and udev rules
- Optimize Jetson performance settings
- Create workspace structure

### Step 2: Manual Setup (Advanced Users)
If you prefer manual installation:

```bash
# Install GPIO libraries
sudo apt install -y python3-jetson-gpio python3-rpi-gpio python3-gpiod

# Install ROS 2 dependencies
sudo apt install -y \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-rplidar-ros \
    ros-humble-realsense2-camera \
    libserial-dev

# Install Python dependencies
pip3 install --user pyserial smbus

# Configure permissions
sudo usermod -a -G dialout,gpio,video $USER
```

## 🏗️ Building the Firmware

```bash
# Navigate to workspace
cd ~/autonombot_ws

# Build the firmware package
colcon build --packages-select autonombot_firmware

# Source the workspace
source install/setup.bash
```

## ✅ Hardware Validation

Before deploying, validate all sensors are working:

```bash
# Run hardware validation script
ros2 run autonombot_firmware validate_hardware.py
```

This will test:
- ✅ Ultrasonic sensor data quality
- ✅ LiDAR scan data
- ✅ Camera color and depth streams
- ✅ Communication latency

Expected output:
```
HARDWARE VALIDATION COMPLETE
ULTRASONIC SENSOR: ✓ OPERATIONAL
LIDAR SENSOR: ✓ OPERATIONAL  
CAMERA_COLOR: ✓ OPERATIONAL
CAMERA_DEPTH: ✓ OPERATIONAL
🎉 ALL SENSORS OPERATIONAL - ROBOT READY FOR DEPLOYMENT!
```

## 🚀 Deployment

### Launch Hardware Interface
```bash
# Launch all hardware drivers
ros2 launch autonombot_firmware jetson_hardware.launch.py
```

### Launch Complete Robot System
```bash
# Launch complete autonombot with navigation
ros2 launch autonombot_launch real_robot.launch.py
```

### Launch Options
You can customize the launch with parameters:
```bash
# Custom ultrasonic frequency
ros2 launch autonombot_firmware jetson_hardware.launch.py ultrasonic_frequency:=30.0

# Disable diagnostics for performance
ros2 launch autonombot_firmware jetson_hardware.launch.py enable_diagnostics:=False

# Custom serial port for Arduino
ros2 launch autonombot_firmware jetson_hardware.launch.py serial_port:=/dev/ttyUSB0
```

## 📊 Monitoring and Diagnostics

### System Status
```bash
# Monitor overall sensor status
ros2 topic echo /sensor_manager/status

# Monitor detailed diagnostics
ros2 topic echo /diagnostics

# Monitor individual sensors
ros2 topic echo /ultrasonic_range
ros2 topic echo /scan
ros2 topic echo /camera/color/image_raw
```

### Performance Monitoring
```bash
# Check topic frequencies
ros2 topic hz /ultrasonic_range    # Expected: ~20 Hz
ros2 topic hz /scan                # Expected: ~10 Hz
ros2 topic hz /camera/color/image_raw  # Expected: ~30 Hz

# Monitor system resources
htop
nvidia-smi  # For Jetson GPU usage
```

## 🔧 Configuration

### Sensor Configuration
The firmware can be configured via parameters:

**Ultrasonic Sensor (`hcsr04_driver.py`)**:
```yaml
hcsr04_driver:
  ros__parameters:
    trig_pin: 18           # GPIO pin for trigger
    echo_pin: 24           # GPIO pin for echo
    frequency: 20.0        # Measurement frequency (Hz)
    min_range: 0.02        # Minimum range (m)
    max_range: 4.0         # Maximum range (m)
    use_median_filter: true
    filter_window_size: 5
```

**Sensor Manager (`jetson_sensor_manager.py`)**:
```yaml
jetson_sensor_manager:
  ros__parameters:
    enable_ultrasonic: true
    enable_lidar: true
    enable_camera: true
    diagnostics_frequency: 1.0
    sensor_timeout: 5.0
```

### Hardware Configuration
**GPIO Connections**:
- Ultrasonic TRIG → GPIO 18 (Pin 12)
- Ultrasonic ECHO → GPIO 24 (Pin 18)
- Ground → Pin 6 or 9
- 5V Power → Pin 2 or 4

**USB Connections**:
- RealSense Camera → USB 3.0 port
- RPLiDAR → USB port or Serial
- Arduino → USB port

## 🐛 Troubleshooting

### Common Issues

**1. GPIO Permission Denied**
```bash
# Fix GPIO permissions
sudo usermod -a -G gpio $USER
# Logout and login again
```

**2. Ultrasonic Sensor Not Working**
```bash
# Check GPIO availability
ls /dev/gpiochip*

# Test GPIO library
python3 -c "import Jetson.GPIO as GPIO; print('GPIO OK')"

# Check pin connections and wiring
```

**3. Camera Not Detected**
```bash
# Check USB connection
lsusb | grep Intel

# Install RealSense tools
sudo apt install librealsense2-utils
realsense-viewer  # GUI test tool
```

**4. LiDAR Not Responding**
```bash
# Check USB device
lsusb | grep CP210

# Check permissions
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0  # Temporary fix
```

**5. Arduino Communication Issues**
```bash
# Check Arduino connection
ls -l /dev/ttyACM*

# Test serial communication
sudo apt install minicom
minicom -D /dev/ttyACM0 -b 115200
```

### Debug Mode
Enable debug logging:
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch autonombot_firmware jetson_hardware.launch.py
```

### Hardware Reset
If sensors become unresponsive:
```bash
# Reset GPIO
sudo echo 0 > /sys/class/gpio/export
sudo echo 18 > /sys/class/gpio/unexport
sudo echo 24 > /sys/class/gpio/unexport

# Restart drivers
sudo systemctl restart nvgetty
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 📈 Performance Optimization

### Jetson Performance Mode
```bash
# Set maximum performance
sudo nvpmodel -m 0      # Max performance mode
sudo jetson_clocks      # Enable max clocks

# Monitor performance
sudo tegrastats         # Real-time stats
nvidia-smi             # GPU utilization
```

### Memory Optimization
```bash
# Increase swap for build processes
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 🔒 Safety Considerations

### Emergency Stop
The system includes multiple safety layers:
- Ultrasonic sensor for close-range obstacle detection
- LiDAR for 360° scanning
- Emergency stop via joystick or software command

### Automatic Failsafes
- Sensor timeout detection
- Invalid data filtering
- Graceful degradation if sensors fail

## 📚 API Reference

### Published Topics
- `/ultrasonic_range` (sensor_msgs/Range): Ultrasonic distance measurements
- `/scan` (sensor_msgs/LaserScan): LiDAR scan data
- `/camera/color/image_raw` (sensor_msgs/Image): Camera color stream
- `/camera/depth/image_rect_raw` (sensor_msgs/Image): Camera depth stream
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): System diagnostics
- `/sensor_manager/status` (std_msgs/String): Overall system status

### Services
- Hardware validation and calibration services available via the sensor manager

## 📝 Change Log

### v2.0.0 - Jetson Xavier Production Release
- ✅ Multi-platform GPIO support (Jetson.GPIO, RPi.GPIO, gpiod)
- ✅ Enhanced ultrasonic driver with filtering and error handling
- ✅ Comprehensive sensor monitoring and diagnostics
- ✅ Production-ready launch files and configurations
- ✅ Automated setup and validation scripts
- ✅ Performance optimizations for Jetson Xavier

## 🤝 Support

For issues or questions:
1. Run the hardware validation script
2. Check the troubleshooting guide above
3. Review logs: `ros2 log view`
4. Monitor diagnostics: `ros2 topic echo /diagnostics`

## 🔗 Related Packages
- `autonombot_launch`: Complete robot launch files
- `autonombot_navigation`: Navigation and path planning
- `autonombot_localization`: EKF localization with ultrasonic
- `autonombot_vision`: YOLO-based vision processing
