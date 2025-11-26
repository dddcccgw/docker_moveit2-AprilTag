# ROS2 Humble Docker Setup Guide

## Quick Start

### 1. Build Docker Image

```bash
# Using Docker Compose (recommended)
docker-compose build

# Or using Docker directly
docker build -t apriltag_ros2:humble .
```

### 2. Run Container

```bash
# Using Docker Compose
docker-compose up -it

# Or using Docker directly
docker run -it \
  --name apriltag_humble \
  --volumes-from=$(docker ps -q) \
  --device=/dev/bus/usb:/dev/bus/usb \
  --privileged \
  --net=host \
  apriltag_ros2:humble
```

### 3. Verify Installation Inside Container

```bash
# Check ROS 2 installation
ros2 --version

# Check AprilTag packages
ros2 pkg list | grep apriltag

# Test AprilTag detector (if camera connected)
ros2 run apriltag_detector apriltag_map
```

## Features

### Included in Docker Image

- **ROS 2 Humble** (full desktop distribution)
- **Build Tools**: CMake, Colcon, ROS build dependencies
- **Python Dependencies**: NumPy, OpenCV, RealSense SDK
- **AprilTag Library**: dt-apriltags
- **Intel RealSense SDK**: For D435 camera support
- **Pre-built Workspace**: All packages compiled and ready to use

### Volumes & Devices

- **Source Code**: Current directory mounted to `/root/ws/src/apriltag_ros2_intel_d435/`
- **USB Devices**: Access to `/dev/bus/usb/` for camera connection
- **X11 Display**: Optional GUI support (Linux/macOS with X11)

### Network

- **Host Network**: Uses host network for ROS 2 communication
- **ROS_DOMAIN_ID**: Set to 0 (can be changed in docker-compose.yml)

## Usage Examples

### Build the workspace inside container

```bash
cd /root/ws
colcon build --symlink-install
```

### Run AprilTag node

```bash
# Terminal 1: Start ROS 2 daemon
ros2 daemon start

# Terminal 2: Run the AprilTag detector
ros2 run apriltag_detector apriltag_map

# Terminal 3: Run the camera validator
ros2 run apriltag_detector camera_validator

# Terminal 4: Record calibration data
ros2 run apriltag_detector record_calibration
```

### Debug/Development

```bash
# Enter interactive bash
/bin/bash

# Source setup script manually
source /root/ws/install/setup.bash

# Check ROS 2 environment
printenv | grep ROS

# List available topics
ros2 topic list

# Echo a topic
ros2 topic echo /apriltag/detections
```

## Advanced Configuration

### Custom ROS Domain ID

Edit `docker-compose.yml`:

```yaml
environment:
  - ROS_DOMAIN_ID=42  # Change from 0 to 42
```

### Mount Additional Directories

```yaml
volumes:
  - ./data:/root/data
  - ./config:/root/config
```

### GPU Support (NVIDIA)

For NVIDIA GPU support, modify `docker-compose.yml`:

```yaml
runtime: nvidia
environment:
  - NVIDIA_VISIBLE_DEVICES=all
```

Requires `nvidia-docker` to be installed.

### Environment Variables

```bash
# Set inside container
export ROS_LOCALHOST_ONLY=0  # For multi-machine setup
export ROS_LOG_DIR=/root/logs
```

## Troubleshooting

### Camera Not Detected

```bash
# Inside container
lsusb | grep Intel
python3 -c "import pyrealsense2; print('OK')"
```

### Build Errors

```bash
# Clean and rebuild
cd /root/ws
rm -rf build install log
colcon build --symlink-install
```

### Package Not Found

```bash
# Ensure setup is sourced
source /root/ws/install/setup.bash
ros2 pkg list
```

### Display Issues

```bash
# On host machine (Linux/macOS with X11)
xhost +local:docker
```

## Stopping Container

```bash
# From host machine
docker-compose down

# Or just stop without removing
docker-compose stop

# Resume
docker-compose start
```

## Cleanup

```bash
# Remove container
docker-compose rm -f

# Remove image
docker rmi apriltag_ros2:humble

# Remove all unused Docker resources
docker system prune -a
```

## Tips

1. **Development**: Use `--symlink-install` flag with `colcon build` for faster iteration
2. **Debugging**: Add `--event-handlers console_direct+` to colcon for verbose output
3. **Logging**: Check `/root/ws/log/` for build and run logs
4. **Multiple Terminals**: Use `docker exec -it apriltag_humble bash` to open more shells

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)
- [Docker Compose Reference](https://docs.docker.com/compose/compose-file/)
- [OSRF Docker Images](https://hub.docker.com/r/osrf/ros/)
