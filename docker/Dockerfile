# Use ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    nano \
    htop \
    usbutils \
    python3-serial \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-realsense2-camera \
    ros-humble-rplidar-ros \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR $WORKSPACE

# Copy source code
COPY src/ src/

# Install Python dependencies
RUN pip3 install -r src/autonombot_vision/requirements.txt

# Initialize rosdep and install dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source $WORKSPACE/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]

# Labels for metadata
LABEL maintainer="your-email@example.com"
LABEL description="AutonomBot - Autonomous Navigation Robot"
LABEL version="1.0.0"
LABEL ros.distro="humble"
