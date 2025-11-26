# AprilTag ROS 2 Intel D435 Setup Guide

This document explains how to use this project as a ROS 2 Humble package.

## Project Structure

```
apriltag_ros2_intel_d435/                    # Main ROS 2 Metapackage
├── CMakeLists.txt                          # ROS 2 metapackage CMake config
├── package.xml                             # ROS 2 metapackage definition
├── apriltag_detector/                      # Main Python package
│   ├── apriltag_detector/
│   │   ├── __init__.py
│   │   ├── apriltag_map.py
│   │   ├── camera_position_validator.py
│   │   └── record_calibration_data.py
│   ├── package.xml                         # ROS 2 Python package definition
│   ├── setup.py                            # Python package setup
│   ├── pyproject.toml                      # Python build configuration
│   └── README.md                           # Package documentation
│
├── apriltag/                               # C++ AprilTag library (optional)
├── apriltag_msgs/                          # Custom ROS 2 messages (optional)
├── apriltag_ros/                           # Legacy ROS 2 node (optional)
└── image_pipeline/                         # Image processing utilities
```

## Installation

### Prerequisites

```bash
# Install ROS 2 Humble
# See: https://docs.ros.org/en/humble/Installation.html

# Install build tools
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential
```

### Build the Workspace

```bash
# Clone or navigate to the workspace
cd ~/AprilTag_ROS2_intel-D435

# Install Python dependencies
pip3 install opencv-python numpy pyrealsense2 dt-apriltags scipy

# Build with colcon (ROS 2 way)
colcon build --symlink-install

# Source the setup script
source install/setup.bash
```

### Verify Installation

```bash
# Check if packages are found
ros2 pkg list | grep apriltag

# Should output:
# apriltag_detector
# apriltag_ros2_intel_d435
```

## Running the Packages

### Method 1: Using ROS 2 Commands

```bash
# Build (if not done)
colcon build --packages-select apriltag_detector

# Source workspace
source install/setup.bash

# Run the AprilTag map builder
ros2 run apriltag_detector apriltag_map

# Run the camera validator
ros2 run apriltag_detector camera_validator

# Run calibration data recorder
ros2 run apriltag_detector record_calibration
```

### Method 2: Direct Python Commands

```bash
# After installing with pip
apriltag_map
camera_validator
record_calibration

# Or using Python modules
python3 -m apriltag_detector.apriltag_map
python3 -m apriltag_detector.camera_position_validator
python3 -m apriltag_detector.record_calibration_data
```

## ROS 2 Package Information

### Main Package: `apriltag_ros2_intel_d435` (Metapackage)
- **Type**: Metapackage
- **Dependencies**: ament_cmake
- **Purpose**: Aggregates all AprilTag-related packages

### Sub-package: `apriltag_detector` (Python)
- **Type**: Python package (ament_python)
- **Dependencies**: 
  - ROS 2: rclpy, sensor_msgs, geometry_msgs, cv_bridge, image_transport
  - Python: numpy, opencv-python, pyrealsense2, dt-apriltags, scipy
- **Executables**:
  - `apriltag_map` - Map building node
  - `camera_validator` - Position validation node
  - `record_calibration` - Calibration data recording node

## Configuration

Edit `/apriltag_detector/apriltag_detector/*.py` to adjust:

```python
# Camera settings
TAG_SIZE = 0.0625              # meters
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
FPS = 30

# Target tags
TARGET_TAG_IDS = [0, 1, 2]
MAP_ORIGIN_TAG_ID = 0

# Validation settings (camera_position_validator only)
EXPECTED_TAG_POSITIONS = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.197, -0.001, -0.021]),
    2: np.array([-0.001, -0.101, -0.004])
}
EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])
POSITION_TOLERANCE = 0.015  # 1.5cm
```

## Troubleshooting

### Package Not Found
```bash
# Make sure you've sourced the setup script
source ~/AprilTag_ROS2_intel-D435/install/setup.bash

# Verify package is recognized
ros2 pkg list
```

### Build Errors
```bash
# Clean and rebuild
rm -rf build install log
colcon build --symlink-install

# Check for Python dependencies
pip3 install --upgrade opencv-python numpy pyrealsense2 dt-apriltags scipy
```

### Camera Not Detected
```bash
# Check Intel RealSense connection
lsusb | grep Intel

# Verify pyrealsense2 can access camera
python3 -c "import pyrealsense2; print('✓ pyrealsense2 OK')"
```

## Next Steps

1. **Launch Configuration**: Create ROS 2 launch files in `launch/` directory
2. **Services/Topics**: Add ROS 2 services and topics for integration
3. **Nodes**: Convert to full ROS 2 nodes (using rclpy.Node)
4. **Tests**: Add unit tests using pytest and ros2 testing framework
5. **Documentation**: Generate ROS 2 documentation with rosdoc2

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [AprilTag Library](https://april.eecs.umich.edu/software/apriltag/)
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [ament_python](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)

## Support

For issues or questions:
1. Check the package README.md
2. Review troubleshooting section
3. Check Intel RealSense documentation
4. Open an issue on GitHub

---

**Last Updated**: November 24, 2025
**ROS 2 Distribution**: Humble
**Python Version**: 3.10+
