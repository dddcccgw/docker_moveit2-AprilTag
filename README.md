# ViperX-300S Visual Servoing Control System

A visual servoing control system for ViperX-300S robotic arm based on ROS2 Humble, integrating VLPoint vision processing and MoveIt2 motion planning.

## Quick Start

### 1. Build and Start Container

```bash
cd docker
./build.sh    # Build Docker image
./run.sh      # Start container
```

### 2. Enter Container (Open Multiple Terminals)

Each program requires a separate terminal window:

```bash
docker exec -it viperx300s_robot bash
```

### 3. Launch System (Run in 4 terminals sequentially)

**Terminal 1 - VLPoint Controller:**
```bash
ros2 launch vlpoint controller.launch.py
```

**Terminal 2 - VLPoint Worker:**
```bash
ros2 launch vlpoint worker.launch.py
```

**Terminal 3 - Visual Servoing:**
```bash
ros2 launch vlservo vlservoing.launch.py
```

**Terminal 4 - MoveIt2:**
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=vx300s hardware_type:=actual
```

### 4. Stop Container

```bash
cd docker
./stop.sh
```

---

## Testing and Verification

### Check Arm Connection

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s
```

### View Topics

```bash
ros2 topic list
ros2 topic echo /joint_states
```

## Development and Building

Rebuild packages inside the container:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Troubleshooting

### Cannot Access USB Devices

```bash
# Add user to dialout group on host
sudo usermod -aG dialout $USER
# Check devices
ls -l /dev/ttyUSB*
```

### RViz2 Not Displaying

```bash
# Allow X11 on host
xhost +local:docker
```

### Arm Not Responding

Check if joint states are being published:
```bash
ros2 topic echo /joint_states
```

---

## System Architecture

- **OS**: Ubuntu 22.04 + ROS2 Humble
- **Motion Planning**: MoveIt2
- **Vision Processing**: VLPoint + Visual Servoing
- **Hardware**: ViperX-300S + RealSense Camera
