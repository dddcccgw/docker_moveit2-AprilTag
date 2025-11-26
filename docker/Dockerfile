# ViperX-300S AprilTag Grasping System - Docker Image
# Base: Ubuntu 22.04 + ROS2 Humble + MoveIt2 + Intel RealSense + AprilTag
# 
# Uncomment the line below to enable NVIDIA GPU support (for future AI/ML features):
# FROM nvidia/cuda:12.8.0-runtime-ubuntu22.04
FROM ubuntu:22.04

# Environment Configuration
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Taipei \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    PIP_NO_CACHE_DIR=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0

# Add ROS 2 apt repository and key
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 \
    lsb-release \
    curl \
    ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# 基本工具與開發套件 (包含 GUI、多媒體、串口支援)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl vim nano \
    locales \
    python3 \
    python3-pip \
    python3-dev \
    python3-zmq \
    python3-opencv \
    build-essential cmake \
    software-properties-common \
    sudo \
    udev \
    usbutils \
    libasound2-dev \
    ffmpeg \
    libsm6 \
    libxext6 \
    libxrender-dev \
    unzip \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    python3-pyqt5 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xfixes0 \
    x11-apps \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble Desktop Full and ROS tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    whiptail \
    iproute2 \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# 安裝 MoveIt2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-ros-perception \
    && rm -rf /var/lib/apt/lists/*

# 安裝 Intel RealSense SDK 2.0
# 添加 Intel RealSense repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && rm -rf /var/lib/apt/lists/*

# 安裝 librealsense2 套件 (不含 DKMS kernel modules)
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-udev-rules \
    && rm -rf /var/lib/apt/lists/*

# 安裝 pyrealsense2 Python 綁定 (透過 pip)
RUN pip3 install --no-cache-dir pyrealsense2

# 安裝 RealSense ROS2 driver
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# 安裝 Dynamixel SDK 和 ros2_control 相關套件 (Interbotix 需要)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-dynamixel-sdk \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# 安裝 AprilTag 相關套件 (核心視覺定位功能)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-apriltag \
    ros-humble-apriltag-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-image-pipeline \
    libopencv-dev \
    libfreeimage-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep database
RUN rosdep init 2>/dev/null || true && rosdep update

# Use bash for subsequent RUN commands so we can 'source'
SHELL ["/bin/bash", "-lc"]

# Python dependencies - 使用 constraints 鎖定版本避免衝突
# 1. 先安裝基礎套件
RUN pip3 install 'pip<25' 'setuptools<80,>=68' 'wheel>=0.40' 'packaging>=24.0'

# 2. 建立 constraints 文件，鎖定 numpy 1.x
RUN printf "numpy==1.26.4\nsympy==1.14.0\nmpmath>=1.3.0\n" > /tmp/constraints.txt

# 3. 安裝 numpy 和科學計算套件
RUN pip3 install -c /tmp/constraints.txt numpy==1.26.4 && \
    pip3 install --no-cache-dir 'scipy>=1.11,<1.12'

# 4. 安裝 sympy (避免 distutils 衝突)
RUN pip3 install --ignore-installed -c /tmp/constraints.txt sympy==1.14.0 "mpmath>=1.3.0"

# 5. 安裝機器人相關 Python 套件 (核心依賴)
RUN pip3 install -c /tmp/constraints.txt \
    modern_robotics \
    pyserial \
    urchin \
    PyQt5>=5.15.0

# 6. 安裝 PyTorch (可選 - 僅在需要 AI/ML 功能時取消註解)
# RUN pip3 install -c /tmp/constraints.txt --index-url https://download.pytorch.org/whl/cu128 \
#     torch torchvision torchaudio

# 7. 安裝其他可選套件 (取消註解以啟用)
# Web/API 服務 (如需遠端控制介面):
# RUN pip3 install -c /tmp/constraints.txt fastapi==0.112.4 uvicorn[standard] gradio==4.16.0
# 
# AI/ML 套件 (如需深度學習功能):
# RUN pip3 install -c /tmp/constraints.txt accelerate bitsandbytes ultralytics

# 8. 安裝其他必要套件
RUN pip3 install --upgrade transforms3d>=0.4.2 && \
    pip3 install pillow==10.2.0

# 設定 udev 規則 (硬體裝置權限)
# Intel RealSense D435 Camera
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="0411", MODE="0666"' > /etc/udev/rules.d/99-realsense.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8087", MODE="0666"' >> /etc/udev/rules.d/99-realsense.rules

# Interbotix ViperX-300S (Dynamixel U2D2 - FTDI USB-to-Serial)
RUN echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666"' > /etc/udev/rules.d/99-interbotix.rules && \
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout"' >> /etc/udev/rules.d/99-interbotix.rules

# 將 root 用戶加入 dialout 組 (允許訪問串口設備)
RUN usermod -aG dialout root

WORKDIR /workspace

# 創建 ROS2 工作區目錄結構
RUN mkdir -p /workspace/ros2_ws/src

# 配置 bash 環境
RUN echo "# ROS2 Humble Environment" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Workspace Environment (if built)" >> /root/.bashrc && \
    echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then" >> /root/.bashrc && \
    echo "    source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Custom Prompt" >> /root/.bashrc && \
    echo "export PS1='\\[\\033[01;32m\\]\\u@viperx300s\\[\\033[00m\\]:\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\$ '" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Welcome Message" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo '═══════════════════════════════════════════════════════════'" >> /root/.bashrc && \
    echo "echo '  ViperX-300S AprilTag Grasping System'" >> /root/.bashrc && \
    echo "echo '  ROS2 Humble + MoveIt2 + Intel RealSense'" >> /root/.bashrc && \
    echo "echo '═══════════════════════════════════════════════════════════'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo 'Quick Start:'" >> /root/.bashrc && \
    echo "echo '  1. Build workspace: cd /workspace/ros2_ws && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "echo '  2. Source workspace: source install/setup.bash'" >> /root/.bashrc && \
    echo "echo '  3. Launch pipeline: ros2 launch apriltag_robot_control pipeline.launch.py'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo 'Documentation: /workspace/README.md'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc

# Ensure login shells also load .bashrc
RUN printf '%s\n' \
    "if [ -f ~/.bashrc ]; then" \
    "    . ~/.bashrc" \
    "fi" > /root/.bash_profile

# Health check (optional - checks if ROS2 is accessible)
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" || exit 1

# Keep container alive for interactive work
CMD ["sleep", "infinity"]
