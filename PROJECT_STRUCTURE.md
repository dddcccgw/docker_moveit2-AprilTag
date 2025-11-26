# ViperX-300S-VLA Project Structure

## 📁 Directory Overview

```
viperx300s-VLA/
│
├── 📂 docker/                          # Docker containerization
│   ├── Dockerfile                     # Container image definition
│   ├── docker-compose.yml             # Service configuration
│   ├── build.sh                       # Build container script
│   ├── run.sh                         # Start container script
│   └── stop.sh                        # Stop container script
│
├── 📂 ros2_ws/                         # ROS2 workspace
│   ├── src/                           # Source packages
│   │   │
│   │   ├── 📂 apriltag_robot_control/       # 🌟 MAIN PACKAGE
│   │   │   ├── scripts/                     # Python executables
│   │   │   │   ├── apriltag_pipeline.py             # Complete automated pipeline
│   │   │   │   ├── world_frame_calibration.py       # Step 1: World frame calibration
│   │   │   │   ├── target_detector.py               # Step 2: Target detection
│   │   │   │   ├── moveit_grasp_controller.py       # Step 3-4: Planning & execution
│   │   │   │   ├── moveit2_interface.py             # MoveIt2 interface utilities
│   │   │   │   └── apriltag_grasp_demo.py           # Demo application
│   │   │   │
│   │   │   ├── launch/                      # Launch files
│   │   │   │   ├── pipeline.launch.py               # Automated pipeline launcher
│   │   │   │   ├── full_system.launch.py            # Full system with MoveIt
│   │   │   │   └── apriltag_grasp.launch.py         # Basic grasp demo
│   │   │   │
│   │   │   ├── config/                      # Configuration files
│   │   │   ├── CMakeLists.txt               # Build configuration
│   │   │   ├── package.xml                  # Package metadata
│   │   │   └── README.md                    # Detailed package documentation
│   │   │
│   │   ├── 📂 AprilTag_ROS2__Intel-D435/    # AprilTag detection package
│   │   │   ├── scripts/                     # AprilTag detection nodes
│   │   │   ├── config/                      # Camera calibration files
│   │   │   └── launch/                      # AprilTag detection launchers
│   │   │
│   │   ├── 📂 interbotix_ros_manipulators/  # Robot arm control
│   │   │   ├── interbotix_xsarm_control/    # Low-level arm control
│   │   │   ├── interbotix_xsarm_moveit/     # MoveIt2 integration
│   │   │   ├── interbotix_xsarm_descriptions/ # Robot URDF models
│   │   │   └── ...
│   │   │
│   │   ├── 📂 interbotix_ros_core/          # Core Interbotix drivers
│   │   │   ├── interbotix_xs_sdk/           # Dynamixel motor control
│   │   │   └── interbotix_xs_msgs/          # ROS message definitions
│   │   │
│   │   └── 📂 interbotix_ros_toolboxes/     # Utility toolboxes
│   │       ├── interbotix_xs_toolbox/       # Robot control utilities
│   │       └── interbotix_perception_toolbox/ # Vision utilities
│   │
│   ├── build/                         # Compiled binaries (auto-generated)
│   ├── install/                       # Installed packages (auto-generated)
│   └── log/                           # Build logs (auto-generated)
│
├── 📂 build/                           # Main workspace build (auto-generated)
├── 📂 install/                         # Main workspace install (auto-generated)
├── 📂 log/                             # Main workspace logs (auto-generated)
│
├── 📄 README.md                        # Main project documentation (START HERE!)
├── 📄 PROJECT_STRUCTURE.md             # This file - project structure guide
├── 📄 test_moveit2.md                  # MoveIt2 testing and troubleshooting
├── 📄 apriltag_map.json                # Saved world frame calibration data
└── 📄 .gitignore                       # Git ignore rules
```

---

## 🎯 Key Components Explained

### 1. Docker Environment (`docker/`)

**Purpose**: Provides a consistent, reproducible development environment

- **Dockerfile**: Installs ROS2, MoveIt2, RealSense drivers, AprilTag libraries
- **docker-compose.yml**: Configures USB device access, X11 display forwarding
- **Scripts**: Simplify container management

**Why Docker?**
- ✅ No dependency conflicts
- ✅ Easy USB device passthrough
- ✅ Reproducible across machines
- ✅ Isolated from host system

### 2. Main Package (`ros2_ws/src/apriltag_robot_control/`)

**Purpose**: Core logic for AprilTag-based robotic grasping

#### Scripts Breakdown:

| Script | Function | Step |
|--------|----------|------|
| `apriltag_pipeline.py` | **Orchestrator** - State machine that coordinates all steps | All |
| `world_frame_calibration.py` | Detects Tags 0,1,2 and builds world coordinate system | 1 |
| `target_detector.py` | Detects Tag 3 and transforms to world frame | 2 |
| `moveit_grasp_controller.py` | Plans trajectory and executes grasp sequence | 3-4 |
| `moveit2_interface.py` | MoveIt2 API wrapper for motion planning | 3 |
| `apriltag_grasp_demo.py` | Standalone demo application | - |

#### Launch Files:

| Launch File | Use Case | When to Use |
|-------------|----------|-------------|
| `pipeline.launch.py` | **Recommended** - Complete automated system | Production use, demos |
| `full_system.launch.py` | Robot + MoveIt + AprilTag demo | Testing integration |
| `apriltag_grasp.launch.py` | Basic grasp without full pipeline | Simple testing |

### 3. AprilTag Detection (`AprilTag_ROS2__Intel-D435/`)

**Purpose**: Detect AprilTag markers from Intel RealSense camera

- Publishes tag detections with 6DOF pose (position + orientation)
- Handles camera calibration and image processing
- Uses `tag36h11` family

### 4. Robot Control (`interbotix_ros_manipulators/`)

**Purpose**: Low-level and high-level robot arm control

- **xsarm_control**: Direct motor control via Dynamixel SDK
- **xsarm_moveit**: MoveIt2 integration for advanced motion planning
- **xsarm_descriptions**: URDF models for visualization and kinematics

### 5. Core Drivers (`interbotix_ros_core/`)

**Purpose**: Hardware interface for Dynamixel motors

- Direct communication with robot servos
- Joint state publishing
- Motor configuration and control

---

## 🔄 Data Flow

```
                 ┌─────────────────┐
                 │  Intel D435     │
                 │  Camera         │
                 └────────┬────────┘
                          │ (RGB-D Images)
                          ↓
                 ┌─────────────────┐
                 │  AprilTag       │
                 │  Detection      │
                 └────────┬────────┘
                          │ (Tag Poses)
           ┌──────────────┼──────────────┐
           ↓              ↓              ↓
  ┌────────────┐  ┌──────────────┐  ┌──────────────┐
  │ Tag 0,1,2  │  │   Tag 3      │  │  TF2         │
  │ (World)    │  │   (Target)   │  │  (Transforms)│
  └─────┬──────┘  └──────┬───────┘  └──────────────┘
        │                 │
        └────────┬────────┘
                 ↓
        ┌────────────────┐
        │  apriltag_     │
        │  pipeline.py   │  (State Machine)
        └────────┬───────┘
                 │
     ┌───────────┼───────────┐
     ↓           ↓           ↓
┌─────────┐ ┌─────────┐ ┌─────────┐
│Calibrate│ │ Detect  │ │  Plan   │
│ (Step 1)│ │(Step 2) │ │(Step 3) │
└─────────┘ └─────────┘ └────┬────┘
                              ↓
                     ┌────────────────┐
                     │     MoveIt2    │
                     │  Motion Plan   │
                     └────────┬───────┘
                              ↓
                     ┌────────────────┐
                     │  Execute Grasp │
                     │    (Step 4)    │
                     └────────┬───────┘
                              ↓
                     ┌────────────────┐
                     │  ViperX-300S   │
                     │  Robot Arm     │
                     └────────────────┘
```

---

## 🗂️ File Types

### Auto-Generated (Don't Edit)

- `build/`, `install/`, `log/` - Build artifacts
- `*egg-info/` - Python package metadata
- `CMakeFiles/`, `CMakeCache.txt` - CMake build files

### Configuration Files

- `package.xml` - ROS package metadata and dependencies
- `CMakeLists.txt` - Build instructions
- `docker-compose.yml` - Container configuration
- `apriltag_map.json` - Saved calibration data (runtime generated)

### Source Code

- `*.py` - Python scripts (main logic)
- `*.launch.py` - ROS2 launch files
- `*.yaml` - Configuration parameters

### Documentation

- `README.md` files - User documentation
- `*.md` files - Guides and notes

---

## 🔧 Typical Workflows

### Starting from Scratch

```bash
1. cd docker && ./build.sh           # Build container (once)
2. ./run.sh                          # Start container
3. docker exec -it viperx300s_robot bash
4. cd /workspace/ros2_ws
5. colcon build --symlink-install    # Build packages (once)
6. source install/setup.bash
7. ros2 launch apriltag_robot_control pipeline.launch.py ...
```

### Modifying Code

```bash
1. Edit files in /workspace/ros2_ws/src/apriltag_robot_control/
2. cd /workspace/ros2_ws
3. colcon build --packages-select apriltag_robot_control --symlink-install
4. source install/setup.bash
5. Re-run your launch command
```

### Adding New Dependencies

```bash
1. Edit package.xml (add <depend>package_name</depend>)
2. Edit CMakeLists.txt (add find_package(), ament_target_dependencies())
3. Rebuild: colcon build --packages-select apriltag_robot_control
```

---

## 📊 ROS2 Topics & Services

### Published Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/pipeline_state` | String | apriltag_pipeline | Current state (INIT, CALIBRATING, DETECTING, etc.) |
| `/calibration_status` | Bool | world_frame_calibration | World frame calibration complete |
| `/target_pose` | PoseStamped | target_detector | Target position in world frame |
| `/grasp_pose` | PoseStamped | moveit_grasp_controller | Computed grasp position |
| `/joint_states` | JointState | robot control | Current joint positions |
| `/tf` | TFMessage | various | Coordinate transformations |

### Services

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/start_pipeline` | Trigger | apriltag_pipeline | Start automated pipeline |
| `/reset_pipeline` | Trigger | apriltag_pipeline | Reset to initial state |

### TF Frames

| Frame | Parent | Description |
|-------|--------|-------------|
| `world` | - | Origin (AprilTag 0) |
| `camera_link` | `world` | Camera position |
| `target` | `world` | Target object (Tag 3) |
| `base_link` | `world` | Robot base |
| `ee_gripper_link` | `base_link` | End effector |

---

## 🎓 Learning Path for New Developers

### Beginner

1. ✅ Read main `README.md`
2. ✅ Understand the 4-step pipeline
3. ✅ Run system in simulation mode
4. ✅ Monitor topics with `ros2 topic echo`

### Intermediate

1. ✅ Read `apriltag_robot_control/README.md`
2. ✅ Understand each script's purpose
3. ✅ Modify launch parameters
4. ✅ Run with real hardware

### Advanced

1. ✅ Read source code in `scripts/`
2. ✅ Understand state machine in `apriltag_pipeline.py`
3. ✅ Modify grasp trajectories
4. ✅ Add new features

---

## 🔗 Related Documentation

- **Main README**: Overall system guide (start here!)
- **Package README**: `ros2_ws/src/apriltag_robot_control/README.md` - Detailed API
- **MoveIt2 Testing**: `test_moveit2.md` - Troubleshooting guide
- **Docker Config**: `docker/docker-compose.yml` - Container setup

---

**Last Updated**: November 26, 2025
