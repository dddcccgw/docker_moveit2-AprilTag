# 📁 Project Structure Guide

This document explains the organized structure of the AprilTag_ROS2_intel-D435 project.

## Directory Layout

```
AprilTag_ROS2_intel-D435/
├── 📄 README.md                    # Main project documentation
├── � STRUCTURE.md                 # This file
├── 📄 .gitignore                   # Git ignore rules
├── �📁 docs/                        # All additional documentation
├── 📁 docker_config/               # Docker deployment files
├── 📁 images/                      # Screenshots and diagrams
├── 📁 config/                      # Configuration files (JSON, etc.)
├── 📁 scripts/                     # Shell scripts and standalone Python scripts
├── 📁 camera_info/                 # Camera calibration files
├── 📁 data/                        # Generated calibration data
├── 📄 CMakeLists.txt               # Top-level build file (legacy)
├── 📄 package.xml                  # Top-level package manifest (legacy)
├── 📄 Makefile                     # Build shortcuts
└── 📁 ros2_ws/                     # ROS 2 workspace (MAIN)
    ├── 📁 src/                     # ROS 2 source packages
    │   ├── apriltag/               # AprilTag C library
    │   ├── apriltag_detector/      # Main Python package
    │   ├── apriltag_msgs/          # ROS 2 message definitions
    │   ├── apriltag_ros/           # Additional ROS integration
    │   ├── image_pipeline/         # Image processing packages
    │   └── vision_opencv/          # OpenCV-ROS bridge
    ├── 📁 build/                   # Build artifacts (gitignored)
    ├── 📁 install/                 # Installed packages (gitignored)
    └── 📁 log/                     # Build logs (gitignored)
```

## Quick Reference

### 📚 Documentation
- **Main README**: `README.md` (in root)
- **All docs**: `docs/` directory
  - Setup guides: `SETUP_GUIDE.md`, `ROS2_SETUP.md`, `DOCKER_SETUP.md`
  - Project index: `PROJECT_INDEX.md`
  - Technical summary: `APRILTAG_DETECTOR_SUMMARY.md`

### 🐳 Docker Deployment
- All Docker files: `docker_config/` directory
- Run: `cd docker_config && docker-compose up`

### 🖼️ Images
- All screenshots and diagrams: `images/` directory
- Referenced in README with `images/` prefix

### ⚙️ Configuration
- JSON config files: `config/` directory
  - `apriltag_map.json` - Generated map data
  - `target_location_tag3.json` - Target definitions

### 🔧 Scripts
- Installation scripts: `scripts/install.sh`, `scripts/uninstall.sh`
- Setup scripts: `scripts/setup_apriltag_detector.sh`
- Standalone Python: `scripts/*.py`

### 📦 Main Package
- ROS 2 package: `apriltag_detector/`
- Source code: `apriltag_detector/apriltag_detector/*.py`
- Run via: `ros2 run apriltag_detector <command>`

## Path Updates

If you're updating any scripts or documentation:

- **Image references**: Use `images/filename.png`
- **Config files**: Use `config/filename.json`
- **Documentation**: Use `docs/filename.md`
- **Scripts**: Use `scripts/filename.sh` or `scripts/filename.py`
- **Docker**: Use `docker_config/docker-compose.yml`

## Build Artifacts

The following directories are generated during build and are **gitignored**:
- `build/` - CMake/colcon build output
- `install/` - Installed packages
- `log/` - Build and runtime logs

To rebuild:
```bash
rm -rf build/ install/ log/
colcon build
```

## Quick Start

### Building the ROS 2 Workspace

```bash
cd ~/AprilTag_ROS2_intel-D435/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running the Programs

```bash
# After sourcing the workspace
ros2 run apriltag_detector apriltag_map
ros2 run apriltag_detector camera_validator
ros2 run apriltag_detector record_calibration
```

### General Usage

1. **Read**: `README.md` in root directory
2. **Setup**: Follow `docs/SETUP_GUIDE.md`
3. **Docker**: Use `docker_config/docker-compose.yml`
4. **Build**: Work in `ros2_ws/` directory

---

*This structure was organized on Nov 25, 2025 to improve project clarity and maintainability.*
