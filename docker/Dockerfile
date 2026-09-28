# ==============================================================================
# MetroLidar2026: Autonomous Metro Obstacle Detection & Clearance Verification
# Multi-Platform Support: macOS (Apple Silicon arm64 / Intel amd64) & Linux Stand
# Base: Official ROS 2 Humble (lightweight, multi-arch, rock-solid)
# ==============================================================================

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 1. Install locale, build toolchain, PCL, Eigen3, OpenMP, and ROS 2 middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    build-essential \
    cmake \
    git \
    pkg-config \
    libpcl-dev \
    libeigen3-dev \
    libomp-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-foxglove-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-conversions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    sqlite3 \
    libsqlite3-dev \
    python3-aiohttp \
    python3-websockets \
    python3-numpy \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# 2. Update rosdep package index
RUN rosdep update --rosdistro humble || true

# 3. Create workspace directory and copy project files
WORKDIR /ros2_ws

ARG BUILD_DATE=unknown
COPY docker/cyclonedds.xml /ros2_ws/cyclonedds.xml
COPY src/ /ros2_ws/src/

# 4. Resolve dependencies and build C++ packages with Release optimization
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 5. Configure entrypoint script
COPY docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]

# Default launch command
CMD ["ros2", "launch", "metro_obstacle_detector", "obstacle_detector.launch.py"]
