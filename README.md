# ViperX-300S AprilTag Grasping System

An automated robotic grasping system for ViperX-300S based on ROS2 Humble, using AprilTag visual markers for world frame calibration and target detection, integrated with MoveIt2 motion planning.

## 🎯 System Overview

This system enables autonomous object grasping through 4 automated steps:

1. **World Frame Calibration** - Uses 3 AprilTags (ID: 0,1,2) to establish a fixed world coordinate system
2. **Target Detection** - Detects AprilTag (ID: 3) on target object and transforms to world frame
3. **Motion Planning** - Plans robot trajectory using MoveIt2 with collision avoidance
4. **Grasp Execution** - Executes complete grasp sequence (approach → grasp → retreat → home)

## 🚀 Quick Start

### 1. Build and Start Docker Container

```bash
cd docker
./build.sh    # Build Docker image (first time only)
./run.sh      # Start container
```

### 2. Enter Container

```bash
docker exec -it viperx300s_robot bash
```

### 3. Build ROS2 Workspace (First Time Only)

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch System

**Option A: Fully Automated Pipeline (Recommended)**

Single launch file that runs the complete 4-step pipeline automatically:

```bash
# With simulation (no physical robot needed):
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=fake \
  auto_start:=true

# With real robot:
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=actual \
  auto_start:=true
```

> **Tip:** When you set `hardware_type:=fake`, the launch file automatically enables the Interbotix simulator (`use_sim:=true`) so no `/dev/ttyUSB0` device is required.

**Option B: Manual Step-by-Step Control**

Launch pipeline without auto-start, then trigger manually:

```bash
# Terminal 1: Launch the pipeline system
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=fake \
  auto_start:=false

# Terminal 2: Trigger pipeline when ready
ros2 service call /start_pipeline std_srvs/srv/Trigger

# Monitor pipeline state:
ros2 topic echo /pipeline_state
```

**Option C: Individual Components (Advanced)**

Run each step separately for debugging:

```bash
# Terminal 1: World frame calibration
ros2 run apriltag_robot_control world_frame_calibration.py

# Terminal 2: Target detection
ros2 run apriltag_robot_control target_detector.py

# Terminal 3: Grasp controller
ros2 run apriltag_robot_control moveit_grasp_controller.py

# Terminal 4: MoveIt2 + Robot control
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
  robot_model:=vx300s \
  hardware_type:=fake
```

### 5. Stop Container

```bash
cd docker
./stop.sh
```

---

## 📋 Hardware Setup

### AprilTag Placement

Place AprilTags at these exact positions to establish the world coordinate system:

| Tag ID | Position (meters) | Purpose |
|--------|------------------|---------|
| 0 | (0.0, 0.0, 0.0) | World frame origin |
| 1 | (0.15, 0.0, 0.0) | X-axis reference (15cm from origin) |
| 2 | (0.0, 0.0, 0.15) | Z-axis reference (15cm above origin) |
| 3 | On target object | Object to be grasped |

- **Tag Size**: 6.25 cm (0.0625 m)
- **Tag Family**: tag36h11
- **Camera**: Intel RealSense D435 must see at least Tag 0

### Hardware Requirements

- ✅ Interbotix ViperX-300S robotic arm
- ✅ Intel RealSense D435 depth camera
- ✅ 4x AprilTag markers (tag36h11 family, 6.25cm size)
- ✅ USB connections for robot and camera

---

## 🔍 Monitoring & Debugging

### Monitor Pipeline Status

```bash
# Watch pipeline state changes (INIT → CALIBRATING → DETECTING → PLANNING → EXECUTING → DONE)
ros2 topic echo /pipeline_state

# Check if world frame is calibrated
ros2 topic echo /calibration_status

# View detected target position
ros2 topic echo /target_pose

# View planned grasp position
ros2 topic echo /grasp_pose
```

### Verify Camera Detection

```bash
# List RealSense devices
rs-enumerate-devices

# Test camera stream
ros2 run rqt_image_view rqt_image_view
```

### Test Robot Connection

```bash
# Check USB device
ls -l /dev/ttyUSB*

# Test basic robot control (without MoveIt2)
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s

# Check joint states
ros2 topic echo /joint_states
```

### View TF Frames

```bash
# List all coordinate frames
ros2 run tf2_ros tf2_echo world camera_link
ros2 run tf2_ros tf2_echo world target

# Visualize TF tree
ros2 run tf2_tools view_frames
```

---

## ⚙️ Configuration

### Launch Parameters

Customize system behavior with these parameters:

```bash
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \              # Robot model (vx300s, wx250s, etc.)
  hardware_type:=fake \              # 'actual' for real robot, 'fake' for simulation
  tag_size:=0.0625 \                 # AprilTag size in meters (default: 6.25cm)
  auto_start:=true \                 # Auto-start pipeline on launch
  approach_height:=0.15 \            # Height above target for approach (meters)
  grasp_height:=0.02 \               # Final grasp height above target (meters)
  retreat_height:=0.15 \             # Retreat height after grasp (meters)
  use_rviz:=true                     # Launch RViz for visualization
```

### Saved Data

The system saves calibration data for reproducibility:

- `apriltag_map.json` - World frame calibration with all tag positions
- `world_calibration.json` - Camera pose and intrinsics

---

## 🛠️ Development

### Rebuild ROS2 Packages

After modifying code:

```bash
cd /workspace/ros2_ws
colcon build --packages-select apriltag_robot_control --symlink-install
source install/setup.bash
```

### Code Structure

```
ros2_ws/src/apriltag_robot_control/
├── scripts/
│   ├── apriltag_pipeline.py              # Orchestrated pipeline (state machine)
│   ├── world_frame_calibration.py        # Step 1: Build world frame
│   ├── target_detector.py                # Step 2: Detect target
│   ├── moveit_grasp_controller.py        # Step 3-4: Plan & execute grasp
│   └── moveit2_interface.py              # MoveIt2 interface utilities
├── launch/
│   ├── pipeline.launch.py                # Complete automated pipeline
│   ├── full_system.launch.py             # Full system with MoveIt
│   └── apriltag_grasp.launch.py          # Basic grasp demo
└── config/
    └── (configuration files)
```

---

## 🐛 Troubleshooting

### Docker Issues

**Cannot access USB devices:**
```bash
# On host machine, add user to dialout group
sudo usermod -aG dialout $USER
# Log out and log back in

# Check device permissions
ls -l /dev/ttyUSB* /dev/video*
```

**RViz2 not displaying:**
```bash
# On host machine, allow X11 connections
xhost +local:docker

# Set display inside container (usually automatic)
echo $DISPLAY
```

### Camera Issues

**Camera not detected:**
```bash
# Check RealSense devices
rs-enumerate-devices

# Check video devices
ls -l /dev/video*

# Test camera in RViz
ros2 launch realsense2_camera rs_launch.py
```

### Robot Issues

**Arm not responding:**
```bash
# Check USB connection
ls -l /dev/ttyUSB*

# Should show: /dev/ttyUSB0 with rw permissions

# Test direct control first
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s
```

**MoveIt2 crashes with hardware_type:=actual:**

See `test_moveit2.md` for detailed solutions. Quick fix:
```bash
# Start with simulation mode first to verify software works
hardware_type:=fake
```

### AprilTag Detection Issues

**Tags not detected:**
- ✅ Ensure proper lighting (avoid shadows and glare)
- ✅ Verify tag size parameter matches physical tags (0.0625m = 6.25cm)
- ✅ Check that camera can see at least Tag 0 clearly
- ✅ Ensure tags are printed clearly (high contrast, not blurry)

**Unstable detection:**
- System uses temporal filtering - wait 2-3 seconds for stabilization
- Ensure tags are not moving or vibrating
- Check camera focus

---

## 📚 Additional Documentation

- `test_moveit2.md` - MoveIt2 integration testing and troubleshooting
- `ros2_ws/src/apriltag_robot_control/README.md` - Detailed API documentation
- `docker/` - Docker environment setup details

---

## 🏗️ System Architecture

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Motion Planning**: MoveIt2 with ros2_control
- **Vision**: AprilTag detection + Intel RealSense D435
- **Robot**: Interbotix ViperX-300S (Dynamixel servos)
- **Container**: Docker with X11 forwarding

### Technology Stack

```
┌─────────────────────────────────────────┐
│         Application Layer               │
│  (apriltag_robot_control package)       │
└─────────────────────────────────────────┘
           ↓              ↓
┌──────────────────┐  ┌─────────────────┐
│   Vision Layer   │  │  Motion Layer   │
│  - AprilTag      │  │  - MoveIt2      │
│  - RealSense     │  │  - ros2_control │
└──────────────────┘  └─────────────────┘
           ↓              ↓
┌─────────────────────────────────────────┐
│           ROS2 Humble                    │
│  (TF2, Topics, Services, Actions)       │
└─────────────────────────────────────────┘
           ↓              ↓
┌──────────────────┐  ┌─────────────────┐
│  Intel D435      │  │  ViperX-300S    │
│  Camera          │  │  Robot Arm      │
└──────────────────┘  └─────────────────┘
```

---

## 📄 License

BSD-3-Clause

## 🤝 Contributing

For detailed development guidelines, see the package-specific README in `ros2_ws/src/apriltag_robot_control/README.md`.
