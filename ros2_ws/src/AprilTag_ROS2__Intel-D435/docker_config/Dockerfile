FROM osrf/ros:humble-desktop

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Update system packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libusb-1.0-0-dev \
    pkg-config \
    libudev-dev \
    libssl-dev \
    vim \
    nano \
    htop \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for AprilTag
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python \
    pyrealsense2 \
    dt-apriltags \
    scipy

# Install Intel RealSense SDK dependencies
RUN apt-get update && \
    apt-get install -y \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/ws/src

# Copy AprilTag project
COPY . /root/ws/src/apriltag_ros2_intel_d435/

WORKDIR /root/ws

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Install dependencies
RUN cd /root/ws && \
    rosdep install --from-paths src --ignore-src -y || true

# Build the workspace
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    cd /root/ws && \
    colcon build --symlink-install'

# Source setup script
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc

# Set up entry point
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "source /root/ws/install/setup.bash && bash"]
