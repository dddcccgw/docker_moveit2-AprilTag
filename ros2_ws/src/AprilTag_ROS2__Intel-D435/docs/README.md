# 🧭 AprilTag Multi-Tag Tracking & Map Frame Estimation

This is a ROS2 Package/standalone Python implementation for **multi-AprilTag detection**, **6DoF pose estimation**, and **map coordinate frame construction** using an Intel RealSense D435 RGB-D camera.


---

## � Screenshots & Visual Guide

### 🔧 Physical Setup
![Physical Setup](setup.png)
*Hardware configuration showing Intel RealSense D435 camera mounted on tripod with three AprilTags (ID: 0, 1, 2) mounted on a vertical board with grid paper for precise positioning*

### 🎯 Program: `my_camera_apriltag.py` - Multi-Tag Tracking & Map Frame
![AprilTag Map Frame Visualization](my_camera_apriltag.png)
*Real-time multi-tag tracking showing 3 AprilTags with map frame coordinate system. Features include:*
- *Live camera feed with detected tags (ID 0, 1, 2) highlighted with colored borders*
- *3D coordinate axes (RGB) overlaid on each tag showing orientation*
- *Real-time position data displayed: Tag 0 at origin (0.000, 0.000, 0.000), Tag 1 at (0.197, -0.002, -0.013), Tag 2 at (-0.002, -0.100, -0.003)*
- *Rotation angles (Roll, Pitch, Yaw) for each tag displayed in terminal*
- *Detection status: "Detected: 3/3 tags"*
- *Map frame established with Tag 0 as the origin*

### 📊 Program: `record_calibration_data.py` - Hand-Eye Calibration
![Calibration Data Recording](record_calibration_data.png)
*Interactive hand-eye calibration data collection showing synchronized camera-tag poses and robot end-effector positions for computing camera-to-robot transformation*

### ✅ Program: `camera_position_validation.py` - Position Validator
![Position Validation Pass](camera-position_validator.png)
*Position validation showing ALL CHECKS PASSED with detailed error metrics:*
- *Camera position validated: [-0.0814, 0.0493, -0.5819] m with 14.41 mm error ✓*
- *Tag 0 [ORIGIN]: Perfect match at (0.000, 0.000, 0.000) m*
- *Tag 1: Actual [0.1978, -0.0019, -0.0141] vs Expected [0.1970, -0.0010, -0.0210] - Error: 7.50mm ✓*
- *Tag 2: Actual [-0.0016, -0.1010, -0.0040] vs Expected [-0.0010, -0.1010, -0.0040] - Error: 2.50mm ✓*
- *Green ✓ indicators show all positions within 15mm tolerance*
- *Real-time validation overlay with color-coded pass/fail status*

---

## �📦 Features

### ✨ Advanced Capabilities
- **Multi-Tag Tracking**: Simultaneously detect and track multiple AprilTags (configurable IDs)
- **Map Frame Construction**: Establish a global coordinate system using one tag as the origin
- **Relative Position Calculation**: Compute 3D positions of all tags relative to the map origin
- **Temporal Filtering**: Smooth pose estimation with moving average filter (reduces jitter)
- **Pose Validation**: Automatic validation of detected poses for reliability
- **Real-time Visualization**: 3D coordinate axes for each tag, on-screen position display, color-coded tags
- **Data Export**: Save final map data to JSON file with all tag positions and orientations
- **ROS 2 Integration**: TF broadcasting, console scripts, and native ROS 2 entry points

### 🎯 Technical Highlights
- Intel RealSense D435 RGB-D integration
- Image preprocessing for robust detection (histogram equalization, Gaussian blur)
- Configurable detection parameters for accuracy vs. speed trade-off
- Comprehensive error handling and validation
- Clean terminal output with periodic status updates

---



## 📋 Main Programs

---

### 1️⃣ apriltag_map — Multi-Tag Tracking & Map Frame Construction
**Key Features:**
- Detects multiple AprilTags and builds a unified map coordinate frame
- Uses a specified tag as the origin; computes all tag positions in the map frame
- Temporal filtering (smooths coordinates), auto-saves JSON, can publish ROS 2 TF

**How to Run:**
- ROS 2:
  ```bash
  ros2 run apriltag_detector apriltag_map
  ```
- Standalone script:
  ```bash
  python3 scripts/my_camera_apriltag.py
  ```

---

### 2️⃣ camera_position_validator — Camera/Tag Position Validation
**Key Features:**
- Real-time comparison of detected camera/tag positions with ground truth
- Customizable error tolerance (default: 15mm)
- Terminal output with detailed errors, ✓/✗ pass/fail indicators, color visualization

**How to Run:**
- ROS 2:
  ```bash
  ros2 run apriltag_detector camera_validator
  ```
- Standalone script:
  ```bash
  python3 scripts/camera_position_validation.py
  ```

---

### 3️⃣ record_calibration_data — Hand-Eye Calibration Data Collection
**Key Features:**
- Interactive collection of camera-tag and robot end-effector pose pairs
- Supports batch sampling, auto-saves data (.npy/.json)
- Designed for hand-eye calibration workflows

**How to Run:**
- ROS 2:
  ```bash
  ros2 run apriltag_detector record_calibration
  ```
- Standalone script:
  ```bash
  python3 scripts/record_calibration_data.py
  ```

---

## 🚀 Installation

### Hardware Requirements
- **Intel RealSense D435** RGB-D Camera
- **Robot Arm** (optional, for hand-eye calibration)
- **AprilTags** - Printed on flat, rigid surface (foam board or acrylic recommended)

### Software Requirements
- **Python 3.8+**
- **Intel RealSense SDK 2.0**

### Python Dependencies
```bash
pip install opencv-python numpy pyrealsense2 dt-apriltags scipy
```

### Quick Install

**Method 1: Using setup script (recommended)**
```bash
~/AprilTag_ROS2_intel-D435/setup_apriltag_detector.sh
```

**Method 2: Manual installation**
```bash
cd ~/AprilTag_ROS2_intel-D435/apriltag_detector
pip3 install -e .
```

**Method 3: ROS 2 workspace build**
```bash
cd ~/AprilTag_ROS2_intel-D435
colcon build --base-paths apriltag_detector --symlink-install
source install/setup.bash
```

---

## ⚙️ Configuration

### AprilTag Size
Edit the `TAG_SIZE` constant at the top of module files (default: 6.25cm):
```python
TAG_SIZE = 0.0625  # meters (6.25 cm)
```

### Camera Resolution
Adjust frame width and height:
```python
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
```

### Detection Parameters
Fine-tune in `Detector` initialization:
```python
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=2.0,      # Lower = better accuracy, slower
    quad_sigma=0.8,         # Blur before detection
    refine_edges=1,
    decode_sharpening=0.25  # Sharpening factor
)
```

### Expected Positions (for camera_position_validator)
Edit expected tag and camera positions:
```python
EXPECTED_TAG_POSITIONS = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.197, -0.001, -0.021]),
    2: np.array([-0.001, -0.101, -0.004])
}
EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])
POSITION_TOLERANCE = 0.015  # 1.5cm
```

---

## 📚 Coordinate System

### Map Frame Reference
- **Origin**: AprilTag specified by `MAP_ORIGIN_TAG_ID` (default: Tag 0)
- **X-axis**: Points right (when viewing Tag 0)
- **Y-axis**: Points up
- **Z-axis**: Points toward camera direction

All other tag positions are reported relative to this origin.

---

## 💾 Output Files

### apriltag_map.json
```json
{
  "metadata": {
    "map_origin_tag_id": 0,
    "tag_size": 0.0625,
    "timestamp": "2025-11-24 10:30:45",
    "camera_intrinsics": {
      "fx": 603.59, "fy": 603.08,
      "cx": 329.98, "cy": 246.51
    }
  },
  "tags": {
    "0": {
      "position": [0.0, 0.0, 0.0],
      "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      "rotation_euler_deg": [0.0, 0.0, 0.0],
      "is_origin": true
    },
    "1": {
      "position": [0.1973, -0.0019, -0.0139],
      "rotation_matrix": [...],
      "rotation_euler_deg": [-2.65, 9.13, -0.49],
      "is_origin": false
    }
  }
}
```

### calibration_data.json
```json
{
  "metadata": {
    "tag_size": 0.0625,
    "num_samples": 5
  },
  "samples": [
    {
      "sample": 1,
      "camera_pose": [...],
      "robot_pose": [...]
    }
  ]
}
```

---

## 📂 Project Structure

```
AprilTag_ROS2_intel-D435/
├── apriltag_detector/                          # Main ROS 2 package
│   ├── apriltag_detector/
│   │   ├── __init__.py
│   │   ├── apriltag_map.py                     # Map frame detection
│   │   ├── camera_position_validator.py        # Position validation
│   │   └── record_calibration_data.py          # Calibration data recording
│   ├── resource/
│   ├── package.xml                             # ROS 2 manifest
│   ├── setup.py                                # Python setup + console scripts
│   ├── setup.cfg
│   ├── pyproject.toml
│   ├── QUICK_START.md
│   └── test_package.py
├── scripts/                                    # Standalone scripts (optional)
│   ├── my_camera_apriltag.py                   # Standalone map tracking
│   ├── camera_position_validation.py           # Standalone validator
│   └── record_calibration_data.py              # Standalone calibration
├── apriltag/                                   # AprilTag library
├── apriltag_msgs/                              # ROS message definitions
├── apriltag_ros/                               # Additional ROS integration
├── camera_info/                                # Camera calibration files
├── data/                                       # Generated calibration data
│   ├── camera_poses.npy
│   └── robot_poses.npy
├── apriltag_map.json                           # Generated map data (output)
├── CMakeLists.txt
├── package.xml
├── README.md                                   # This file
├── SETUP_GUIDE.md
├── ROS2_SETUP.md
├── DOCKER_SETUP.md
├── Makefile
└── LICENSE.md
```

---

## ▶️ Detailed Usage Guide

### 🗺️ Basic Multi-Tag Tracking (Complete Workflow)

**Step 1: Connect Camera and Prepare Tags**
```bash
# Check if camera is detected
rs-enumerate-devices

# Navigate to project directory
cd ~/AprilTag_ROS2_intel-D435
```

**Step 2: Run Detection**
```bash
# Via ROS 2 (Recommended)
source install/setup.bash
ros2 run apriltag_detector apriltag_map

# Or standalone Python
cd scripts
python3 my_camera_apriltag.py
```

**Step 3: Monitor Real-Time Output**

**Terminal Output Example:**
```
📷 Camera Intrinsics:
   fx: 603.59, fy: 603.08, cx: 329.98, cy: 246.51
✅ AprilTag detector initialized! Press Q to quit
🗺️  Map Origin: Tag 0

============================================================
🗺️  Map Frame Status (Origin: Tag 0)
============================================================
Frame: 45
Detected: 3/3 tags

Tag 0 [ORIGIN]: (0.000, 0.000, 0.000) m
         Rotation: R=0.0° P=0.0° Y=0.0°

Tag 1: (0.197, -0.002, -0.013) m
         Rotation: R=-2.6° P=9.1° Y=-0.5°

Tag 2: (-0.002, -0.100, -0.003) m
         Rotation: R=-2.5° P=-0.3° Y=0.3°

FPS: 29.8
```

**Step 4: Exit and Save**
```bash
# Press 'Q' to exit
# Program will save: apriltag_map.json
```

**Step 5: View Output**
```bash
cat apriltag_map.json | python3 -m json.tool
```

---

### 🎯 Camera Position Validation (Complete Workflow)

**Step 1: Measure Physical Setup**

Use a ruler/caliper to measure:
- Position of Tag 0 (set as origin at 0,0,0)
- Positions of Tags 1, 2, etc. relative to Tag 0
- Camera position relative to Tag 0

Record values like:
```python
EXPECTED_TAG_POSITIONS = {
    0: np.array([0.0, 0.0, 0.0]),           # Origin
    1: np.array([0.197, -0.001, -0.021]),   # 19.7cm right, 0.1cm back, 2.1cm down
    2: np.array([-0.001, -0.101, -0.004])   # Tag 2 position
}
EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])
POSITION_TOLERANCE = 0.015  # 15mm tolerance
```

**Step 2: Update Script**

Edit `camera_position_validator.py` or `scripts/camera_position_validation.py`:
```python
# Line ~30: Update these values with your measurements
EXPECTED_TAG_POSITIONS = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.197, -0.001, -0.021]),
    2: np.array([-0.001, -0.101, -0.004])
}
EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])
POSITION_TOLERANCE = 0.015  # 1.5cm
```

**Step 3: Run Validation**
```bash
# Via ROS 2
ros2 run apriltag_detector camera_validator

# Or standalone
python3 scripts/camera_position_validation.py
```

**Step 4: Review Results**

**Terminal Output - PASS Example:**
```
==================================================================
🎯 Camera Position Validation
==================================================================
Camera Position: [-0.0881,  0.0598, -0.5885] m
Expected:        [-0.0880,  0.0600, -0.5890] m
Error: 0.71 mm - ✓ PASS

==================================================================
Tag Position Validation
==================================================================
Tag 0 [ORIGIN]:
  Actual:   [ 0.0000,  0.0000,  0.0000] m
  Expected: [ 0.0000,  0.0000,  0.0000] m
  Error: 0.00 mm - ✓ PASS

Tag 1:
  Actual:   [ 0.1971, -0.0015, -0.0213] m
  Expected: [ 0.1970, -0.0010, -0.0210] m
  Error: 0.58 mm - ✓ PASS

Tag 2:
  Actual:   [-0.0020, -0.1003, -0.0033] m
  Expected: [-0.0010, -0.1010, -0.0040] m
  Error: 2.50 mm - ✓ PASS

==================================================================
Overall: ✓✓✓ ALL CHECKS PASSED ✓✓✓
==================================================================
```

**Terminal Output - FAIL Example (with adjustments needed):**
```
==================================================================
🎯 Camera Position Validation
==================================================================
Camera Position: [-0.120,  0.080, -0.590] m
Expected:        [-0.088,  0.060, -0.589] m
Error: 38.5 mm - ✗ FAIL (exceeds 15mm tolerance)

⚠️  Camera position is 38.5mm away from expected!
   Check camera mounting and physical setup.
```

**Step 5: Adjust if Needed**

If validation fails:
1. **Small errors (< 20mm)**: Likely measurement uncertainty - increase tolerance
2. **Large errors (> 20mm)**: 
   - Check if camera is properly mounted
   - Verify tag positions are flat and secure
   - Re-measure physical setup
   - Check camera calibration parameters

---

### 🤖 Hand-Eye Calibration (Complete Workflow)

**Step 1: Setup**
```bash
# Mount camera on robot or fixed workspace
# Place single AprilTag as calibration target
# Connect robot via API (if available)
```

**Step 2: Run Data Collection**
```bash
# Via ROS 2
ros2 run apriltag_detector record_calibration

# Or standalone
python3 scripts/record_calibration_data.py
```

**Step 3: Collect Pose Pairs**

**Interactive Session Example:**
```
📷 Camera Intrinsics:
   fx: 603.59, fy: 603.08, cx: 329.98, cy: 246.51
✅ Tag detection ready. Press SPACE to capture, 'Q' to quit.

---
🤖 Pose Pair Capture
---
Pose 1:
Robot base to end-effector [x, y, z, roll, pitch, yaw] in meters/radians:
> 0.3, 0.2, 0.5, 0, 0, 0

Camera detected! Saving pose pair 1...
✓ Sample 1 saved

---
Pose 2:
Robot base to end-effector [x, y, z, roll, pitch, yaw]:
> 0.4, 0.2, 0.5, 0.1, 0, 0

Camera detected! Saving pose pair 2...
✓ Sample 2 saved

... (repeat 10-20 times for accuracy) ...
```

**Step 4: Exit and Save**
```bash
# Press 'Q' to finish
# Program creates:
#   - data/camera_poses.npy
#   - data/robot_poses.npy
#   - data/calibration_data.json
```

**Step 5: Process Calibration Data**

Use your hand-eye calibration algorithm (OpenCV, SciPy, etc.):
```python
import numpy as np
import cv2

# Load collected data
camera_poses = np.load('data/camera_poses.npy')
robot_poses = np.load('data/robot_poses.npy')

# Extract rotation/translation components
# (format depends on your pose representation)
R_gripper2base, t_gripper2base = extract_robot_poses(robot_poses)
R_target2cam, t_target2cam = extract_camera_poses(camera_poses)

# Run calibration
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

print(f"Camera to Gripper Transformation:\n")
print(f"Rotation:\n{R_cam2gripper}")
print(f"Translation: {t_cam2gripper}")
```

---

## ▶️ Quick Start

### 1. Basic Multi-Tag Tracking
```bash
# Connect Intel RealSense D435 camera
ros2 run apriltag_detector apriltag_map

# Or use standalone script:
cd scripts
python3 my_camera_apriltag.py

# Press 'Q' to exit and save map data
```

**Output:** `apriltag_map.json` with all tag positions in map frame

### 2. Camera Position Validation
```bash
# First, measure and update expected positions in the script
ros2 run apriltag_detector camera_validator

# Or standalone:
python3 camera_position_validation.py
```

**Example output:**
```
==================================================================
🎯 Camera Position Validation
==================================================================
Camera Position: [-0.0881,  0.0598, -0.5885] m
Expected:        [-0.0880,  0.0600, -0.5890] m
Error: 0.71 mm - ✓ PASS

Tag 0: [ORIGIN] ✓ PASS
Tag 1: Error 0.58 mm ✓ PASS
Tag 2: Error 2.50 mm ✓ PASS

==================================================================
Overall: ✓✓✓ ALL CHECKS PASSED ✓✓✓
==================================================================
```

### 3. Hand-Eye Calibration Data Collection
```bash
ros2 run apriltag_detector record_calibration

# Interactive mode:
# 1. Position robot end-effector at different poses
# 2. Press Enter to capture camera-tag + robot pose
# 3. Repeat 10-20 times for better accuracy
# 4. Press 'q' to save data
```

**Output:** `data/camera_poses.npy`, `data/robot_poses.npy`

---

## 🔧 Troubleshooting

### AprilTag Not Detected
1. Ensure tags are clearly visible and flat
2. Reduce `quad_decimate` value (e.g., 1.5) for higher accuracy
3. Increase lighting in environment
4. Check camera focus setting

### Large Position Errors
1. Verify camera calibration parameters
2. Ensure tag size setting matches physical tags
3. Increase temporal filtering window
4. Verify tag mounting is flat and secure

### Camera Connection Issues
1. Verify Intel D435 is connected
2. Check USB connection (USB 3.0 recommended)
3. Validate `pyrealsense2` driver is installed
4. Run: `rs-enumerate-devices` to test

### Jittery Position Estimates
1. Increase temporal filter window size
2. Reduce camera motion during tracking
3. Ensure stable tag mounting
4. Improve lighting conditions

### Camera Position Validation Failures
- **All checks failing**: Update `EXPECTED_TAG_POSITIONS` with measured values
- **Inconsistent results**: Increase `POSITION_TOLERANCE` tolerance if needed
- **Unknown camera position**: Run `apriltag_map` first to determine position
- **Single tag fails**: Check if tag is flat; even small tilts affect accuracy

---

## 📊 Use Cases

### 🗺️ Map Frame Construction
- Robot localization using tags as landmarks
- Multi-robot coordination with shared map frame
- Scene understanding and spatial mapping

### 🤖 Hand-Eye Calibration
- Eye-in-hand setup (camera on robot end-effector)
- Eye-to-hand setup (fixed camera observing robot)
- Robot-camera coordinate system alignment

### ✅ Quality Assurance
- Physical setup verification against design specs
- Post-calibration accuracy validation
- Positioning error diagnosis in multi-tag systems

---

## 🧪 Tested Environment

| Component | Version |
|-----------|---------|
| **Camera** | Intel RealSense D435 |
| **RealSense SDK** | 2.55+ |
| **Python** | 3.10+ |
| **OpenCV** | 4.10+ |
| **dt-apriltags** | 1.0.4+ |
| **NumPy** | 1.24+ |
| **SciPy** | 1.11+ |
| **ROS 2** | Humble or later |

---

## 📚 Additional Resources

- [AprilTag Official](https://april.eecs.umich.edu/software/apriltag/)
- [RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Original AprilTag ROS 2](https://github.com/Tinker-Twins/AprilTag-ROS-2)
- [dt-apriltags Python Wrapper](https://github.com/duckietown/lib-dt-apriltags)

---

## 📄 License

BSD

---

## 👨‍💻 Author

**David Chen**  
📧 gwchen24@gmail.com  
💼 [github.com/dddcccgw](https://github.com/dddcccgw)

---

## 🙏 Attribution

Inspired by and extended from:
- [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2)
- [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag)
- [duckietown/lib-dt-apriltags](https://github.com/duckietown/lib-dt-apriltags)
