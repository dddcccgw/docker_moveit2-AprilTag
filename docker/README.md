# Docker Environment Guide

This directory contains Docker configuration for the ViperX-300S AprilTag Grasping System.

## 📁 Files Overview

| File | Purpose |
|------|---------|
| `Dockerfile` | Container image definition with all dependencies |
| `docker-compose.yml` | Service configuration (USB devices, network, volumes) |
| `build.sh` | Build the Docker image |
| `run.sh` | Start and enter the container |
| `stop.sh` | Stop the container |

---

## 🚀 Quick Start

### 1. Build the Image (First Time Only)

```bash
cd docker
./build.sh
```

**Note**: First build takes 10-20 minutes as it installs ROS2, MoveIt2, RealSense SDK, and all dependencies.

### 2. Start the Container

```bash
./run.sh
```

This will:
- Configure X11 display forwarding for GUI applications
- Start the container in background
- Check hardware connections (robot arm, camera)
- Enter the container shell

### 3. Inside Container - Build ROS2 Workspace

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch the System

```bash
# Simulation mode (no hardware required):
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=fake \
  auto_start:=true

# Real hardware mode:
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=actual \
  auto_start:=true
```

### 5. Stop the Container

```bash
./stop.sh
```

---

## 🔧 Container Configuration

### Included Software

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Desktop Full
- **Motion Planning**: MoveIt2 + ros2_control
- **Vision**: Intel RealSense SDK 2.0
- **AprilTag**: Detection libraries and ROS packages
- **Robot**: Interbotix SDK + Dynamixel SDK
- **Tools**: RViz2, rqt, tf2_tools, colcon

### Hardware Access

The container has direct access to:

#### ViperX-300S Robot Arm
- Device: `/dev/ttyUSB0` (Dynamixel U2D2 USB-to-Serial)
- Permissions: Configured via udev rules
- Symlink: `/dev/ttyDXL` (created automatically)

#### Intel RealSense D435 Camera
- USB bus: `/dev/bus/usb`
- Video devices: `/dev/video0` through `/dev/video5`
- Utilities: `rs-enumerate-devices`, `realsense-viewer`

### Network Configuration

- **Mode**: `host` - Container shares host network
- **Why**: Simplifies ROS2 DDS communication (no port mapping needed)
- **ROS_DOMAIN_ID**: 0 (default, configurable via environment variable)

### Volume Mounts

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `../` | `/workspace` | Project root (live code editing) |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display socket (GUI) |
| `/dev/shm` | `/dev/shm` | Shared memory (ROS2 performance) |

---

## 🎓 Advanced Usage

### Enter Running Container

If container is already running:

```bash
./run.sh              # Automatically enters if running
# or
docker exec -it viperx300s_robot bash
```

### Open Multiple Terminals

Open separate terminals for different ROS2 nodes:

```bash
# Terminal 1
docker exec -it viperx300s_robot bash
ros2 launch apriltag_robot_control pipeline.launch.py

# Terminal 2
docker exec -it viperx300s_robot bash
ros2 topic echo /pipeline_state

# Terminal 3
docker exec -it viperx300s_robot bash
rviz2
```

### View Container Logs

```bash
docker compose logs -f
```

### Rebuild After Code Changes

```bash
# Inside container
cd /workspace/ros2_ws
colcon build --packages-select apriltag_robot_control --symlink-install
source install/setup.bash
```

### Manual Container Management

```bash
# Start without entering
docker compose up -d

# Stop
docker compose down

# Restart
docker compose restart

# View status
docker compose ps
```

---

## 🐛 Troubleshooting

### Build Fails

**Issue**: `docker compose build` fails

**Solutions**:
1. Check Docker is installed: `docker --version`
2. Check Docker Compose: `docker compose version`
3. Ensure you're in `docker/` directory
4. Check internet connection (downloads packages)
5. Clear cache and rebuild: `docker compose build --no-cache`

### Container Won't Start

**Issue**: `./run.sh` fails to start container

**Solutions**:
1. Check if image exists: `docker images | grep viperx300s`
2. Build image first: `./build.sh`
3. Check Docker daemon: `systemctl status docker`
4. View detailed errors: `docker compose up` (without `-d`)

### GUI Applications Don't Display

**Issue**: RViz2 or other GUI apps show errors

**Solutions**:
```bash
# On host machine
xhost +local:docker

# Check DISPLAY variable inside container
echo $DISPLAY

# Test with simple X11 app
xeyes
```

### USB Devices Not Found

**Issue**: `/dev/ttyUSB0` not found, robot not connecting

**Solutions**:
```bash
# On host machine - check USB devices
ls -l /dev/ttyUSB*

# Add user to dialout group (requires logout/login)
sudo usermod -aG dialout $USER

# Check device permissions
ls -l /dev/ttyUSB0
# Should show: crw-rw-rw- or similar (666 permissions)

# If not, fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### Camera Not Detected

**Issue**: RealSense camera not found

**Solutions**:
```bash
# Inside container
rs-enumerate-devices

# If no devices found, check on host
lsusb | grep Intel

# Restart udev (on host)
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### ROS2 Communication Issues

**Issue**: ROS2 nodes can't communicate

**Solutions**:
```bash
# Inside container - check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Should be 0 (or same across all nodes)

# Test communication
ros2 topic list
ros2 node list
```

### Out of Disk Space

**Issue**: Build fails with "no space left on device"

**Solutions**:
```bash
# Check disk usage
df -h

# Clean up Docker
docker system prune -a
docker volume prune

# Remove old images
docker images | grep '<none>' | awk '{print $3}' | xargs docker rmi
```

---

## 🔐 Security Notes

### Privileged Mode

The container runs in `privileged: true` mode to access USB devices. This gives the container full access to host devices.

**Why needed**: 
- Direct USB device access for robot arm and camera
- Hardware control requires kernel-level permissions

**Alternatives** (more secure but complex):
- Use `--device` flags for specific devices only
- Configure device cgroups
- Run without privileged mode (may not work with all hardware)

### Host Network

Container uses `network_mode: host` for simplified ROS2 communication.

**Why needed**:
- ROS2 uses DDS which requires multicast
- Avoids complex port mapping

**Security**: Container can access host network directly.

---

## 📝 Configuration Options

### Enable GPU Support

Uncomment in `docker-compose.yml`:

```yaml
runtime: nvidia
environment:
  - NVIDIA_VISIBLE_DEVICES=all
  - NVIDIA_DRIVER_CAPABILITIES=all
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu, compute, utility]
```

**Requirements**:
- NVIDIA GPU
- NVIDIA Container Toolkit installed
- nvidia-docker2 installed

### Resource Limits

Uncomment in `docker-compose.yml`:

```yaml
deploy:
  resources:
    limits:
      cpus: '4'      # Limit CPU cores
      memory: 8G     # Limit RAM
```

### Change ROS Domain ID

```bash
# Set before starting container
export ROS_DOMAIN_ID=15
./run.sh

# Or edit docker-compose.yml
environment:
  - ROS_DOMAIN_ID=15
```

### Persist Bash History

Uncomment in `docker-compose.yml`:

```yaml
volumes:
  - ./bash_history:/root/.bash_history:rw
```

---

## 🧹 Maintenance

### Update Base Image

```bash
# Pull latest Ubuntu base
docker pull ubuntu:22.04

# Rebuild
./build.sh
```

### Clean Up

```bash
# Remove container
docker compose down

# Remove image
docker rmi viperx300s-ros2:humble

# Remove all unused Docker resources
docker system prune -a
```

### Backup Container State

```bash
# Commit container to new image
docker commit viperx300s_robot viperx300s-ros2:backup

# Save image to file
docker save viperx300s-ros2:backup | gzip > viperx300s-backup.tar.gz

# Load image from file
docker load < viperx300s-backup.tar.gz
```

---

## 📚 Additional Resources

- [Docker Documentation](https://docs.docker.com/)
- [Docker Compose Specification](https://docs.docker.com/compose/compose-file/)
- [ROS2 Docker Guide](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
- [Intel RealSense with Docker](https://github.com/IntelRealSense/librealsense/blob/master/doc/docker.md)

---

**Last Updated**: November 26, 2025
