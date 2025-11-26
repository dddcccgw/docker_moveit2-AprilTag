# 🚀 Quick Start Guide

## Prerequisites
- Intel RealSense D435 camera connected
- ROS 2 Humble installed
- Python dependencies installed

---

## ⚡ Quick Start (3 Steps)

### 1️⃣ Build ROS 2 Workspace

```bash
cd ~/AprilTag_ROS2__Intel-D435/ros2_ws
colcon build --packages-select apriltag_detector
```

### 2️⃣ Source Environment

```bash
source ~/AprilTag_ROS2__Intel-D435/ros2_ws/install/setup.bash
```

**Tip**: Add this to `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/AprilTag_ROS2__Intel-D435/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 3️⃣ Run Programs

**Option A: ROS 2 Commands (Recommended)**
```bash
# Multi-tag tracking & map frame
ros2 run apriltag_detector apriltag_map

# Camera position validation
ros2 run apriltag_detector camera_validator

# Hand-eye calibration data recording
ros2 run apriltag_detector record_calibration
```

**Option B: Standalone Scripts**
```bash
cd ~/AprilTag_ROS2__Intel-D435/scripts

# Multi-tag tracking
python3 my_camera_apriltag.py

# Position validation
python3 camera_position_validation.py

# Calibration data recording
python3 record_calibration_data.py
```

---

## 🔧 Common Commands

### Rebuild Package

```bash
cd ~/AprilTag_ROS2__Intel-D435/ros2_ws
colcon build --packages-select apriltag_detector
source install/setup.bash
```

### Check Package Installation

```bash
ros2 pkg executables apriltag_detector
```

Should show:
```
apriltag_detector apriltag_map
apriltag_detector camera_validator
apriltag_detector record_calibration
```

---

## ❓ Troubleshooting

### Issue: `No executable found`

**Solution**:
```bash
cd ~/AprilTag_ROS2__Intel-D435/ros2_ws
colcon build --packages-select apriltag_detector
source install/setup.bash
```

### Issue: Camera not found

**Solution**:
```bash
rs-enumerate-devices  # Check camera
# Reconnect USB cable (use USB 3.0 port)
```

### Issue: `ModuleNotFoundError`

**Solution**:
```bash
pip install opencv-python numpy pyrealsense2 dt-apriltags scipy
```

---

## 💾 Output Files

- `apriltag_map.json` - Tag positions in map frame
- `data/camera_poses.npy` - Calibration camera poses
- `data/robot_poses.npy` - Calibration robot poses

---

For detailed documentation, see [README.md](README.md)

**Updated**: November 26, 2025  
**Status**: ✅ Fully operational
