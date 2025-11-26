# AprilTag Robot Control

ROS2 package for AprilTag-based robot manipulation with ViperX-300S robot arm.

## Overview

This package provides a **complete automated pipeline** for vision-guided robotic grasping:

### Pipeline Steps (Fully Automated)

1. **Step 1: World Frame Calibration** 
   - Uses 3 AprilTags (ID: 0, 1, 2) at known positions to establish world coordinate frame
   - Computes camera pose in world frame using multi-tag fusion
   - Saves calibration data for reproducibility

2. **Step 2: Target Detection**
   - Detects AprilTag (ID: 3) on target object
   - Transforms target pose to world frame
   - Applies temporal filtering for stable detection

3. **Step 3: Motion Planning**
   - Plans approach/grasp/retreat trajectory using robot IK
   - Validates reachability before execution
   - Handles collision avoidance

4. **Step 4: Grasp Execution**
   - Executes 6-step grasp sequence:
     1. Open gripper
     2. Move to approach position (above target)
     3. Descend to grasp height
     4. Close gripper
     5. Retreat upward
     6. Return to home position

## AprilTag Setup

### Calibration Tags (Known Positions)

Place the following AprilTags at their designated positions:

| Tag ID | Position (m) | Description |
|--------|--------------|-------------|
| 0 | (0.0, 0.0, 0.0) | Origin of world frame |
| 1 | (0.15, 0.0, 0.0) | X-axis reference (15cm from origin) |
| 2 | (0.0, 0.0, 0.15) | Z-axis reference (15cm above origin) |

### Target Tag

| Tag ID | Description |
|--------|-------------|
| 3 | Target object to be grasped |

**Tag Size**: 6.25 cm (tag_size = 0.0625 m)

**Tag Family**: tag36h11

## Hardware Requirements

- Intel RealSense D435 camera
- Interbotix ViperX-300S robot arm
- AprilTags (4x) from tag36h11 family

## Installation

```bash
# Inside Docker container
cd /workspace/ros2_ws
colcon build --packages-select apriltag_robot_control
source install/setup.bash
```

## Quick Start

### Docker Environment (Recommended)

```bash
# 1. Start Docker container
cd docker/
./run.sh

# 2. Inside container, build the package
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select apriltag_robot_control --symlink-install
source install/setup.bash

# 3. Run the complete pipeline
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=fake \
  auto_start:=true
```

### Manual Control (Step by Step)

```bash
# Launch pipeline without auto-start
ros2 launch apriltag_robot_control pipeline.launch.py auto_start:=false

# In another terminal, trigger pipeline manually:
ros2 service call /start_pipeline std_srvs/srv/Trigger

# Reset pipeline to restart:
ros2 service call /reset_pipeline std_srvs/srv/Trigger
```

### Monitor Pipeline State

```bash
# Watch pipeline state in real-time
ros2 topic echo /pipeline_state

# Check calibration status
ros2 topic echo /calibration_status

# View target pose
ros2 topic echo /target_pose
```

## ROS2 Services & Topics

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/start_pipeline` | std_srvs/Trigger | Start the automated pipeline |
| `/reset_pipeline` | std_srvs/Trigger | Reset pipeline to initial state |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pipeline_state` | std_msgs/String | Current state (INIT, CALIBRATING, DETECTING, PLANNING, EXECUTING, DONE, ERROR) |
| `/calibration_status` | std_msgs/Bool | World frame calibration status |
| `/target_pose` | geometry_msgs/PoseStamped | Target position in world frame |
| `/grasp_pose` | geometry_msgs/PoseStamped | Computed grasp position |

### TF Frames

The pipeline publishes these TF transforms:

```
world                    (origin at AprilTag 0)
├── camera_link         (Intel D435 camera)
└── target              (AprilTag 3 on object)
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/calibration_status` | std_msgs/Bool | World frame calibration status |
| `/target_pose` | geometry_msgs/PoseStamped | Target position in world frame |
| `/grasp_pose` | geometry_msgs/PoseStamped | Computed grasp position |

## TF Frames

The system publishes the following transforms:

```
world
├── camera_link (camera position in world)
└── target (AprilTag 3 position in world)
```

## Complete Workflow

### Setup

1. **Place Calibration Tags** at known positions:
   - Tag 0: (0, 0, 0) - origin
   - Tag 1: (0.15m, 0, 0) - X-axis reference  
   - Tag 2: (0, 0, 0.15m) - Z-axis reference

2. **Point Intel D435 Camera** so it can see tag 0 (minimum)

3. **Place Target Object** with AprilTag 3 attached

### Execution

**Automatic Mode** (Recommended):
```bash
ros2 launch apriltag_robot_control pipeline.launch.py auto_start:=true
```

The pipeline automatically:
- ✓ Detects calibration tags and builds world frame
- ✓ Saves calibration to `world_calibration.json`
- ✓ Waits for stable target detection
- ✓ Plans motion trajectory
- ✓ Executes complete grasp sequence
- ✓ Returns to home position

**Manual Mode**:
```bash
# Launch without auto-start
ros2 launch apriltag_robot_control pipeline.launch.py auto_start:=false

# Trigger when ready
ros2 service call /start_pipeline std_srvs/srv/Trigger
```

### State Machine

```
INIT → CALIBRATING → DETECTING → PLANNING → EXECUTING → DONE
         ↑                                                 ↓
         └─────────────────── RESET ─────────────────────┘
                                ↓
                             ERROR
```

Monitor state changes:
```bash
ros2 topic echo /pipeline_state
```

## Configuration Parameters

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_model` | vx300s | Robot model (vx300s, wx250s, etc.) |
| `hardware_type` | fake | actual (real robot) or fake (simulation) |
| `tag_size` | 0.0625 | AprilTag size in meters (6.25cm) |
| `auto_start` | false | Auto-start pipeline on launch |
| `approach_height` | 0.15 | Height above target for approach (m) |
| `grasp_height` | 0.02 | Height above target for grasp (m) |
| `retreat_height` | 0.15 | Height to retreat after grasp (m) |

### Example: Custom Configuration

```bash
ros2 launch apriltag_robot_control pipeline.launch.py \
  robot_model:=vx300s \
  hardware_type:=actual \
  tag_size:=0.05 \
  approach_height:=0.20 \
  grasp_height:=0.03 \
  retreat_height:=0.20 \
  auto_start:=true
```

### Calibration Output

The pipeline saves calibration data to `world_calibration.json`:

```json
{
  "timestamp": "2025-11-26 12:00:00",
  "camera_pose": {
    "translation": [0.3, -0.2, 0.5],
    "rotation_matrix": [[...], [...], [...]]
  },
  "camera_intrinsics": {
    "fx": 604.89, "fy": 604.45,
    "cx": 332.80, "cy": 247.85
  }
}
```

## Troubleshooting

### Camera not detected
```bash
rs-enumerate-devices  # Check RealSense camera
```

### Robot not connecting
- Ensure USB cable is connected
- Check permissions: `ls -la /dev/ttyUSB*`
- Run: `sudo chmod 666 /dev/ttyUSB0`

### Tags not detected
- Ensure proper lighting
- Check tag size parameter matches physical tags
- Verify tag family is tag36h11

## License

BSD-3-Clause
