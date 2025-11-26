#!/bin/bash

# Verify ROS2 Humble Package Setup
# This script checks if the project is properly configured as a ROS2 Humble package

set -e

echo "========================================"
echo "ROS2 Humble Package Verification"
echo "========================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

passed=0
failed=0

# Check function
check() {
    local condition=$1
    local message=$2
    local error_msg=$3
    
    if eval "$condition"; then
        echo -e "${GREEN}✓${NC} $message"
        ((passed++))
    else
        echo -e "${RED}✗${NC} $message"
        if [ -n "$error_msg" ]; then
            echo "  → $error_msg"
        fi
        ((failed++))
    fi
}

# Check ROS 2 distribution
echo "1️⃣  ROS 2 Distribution Checks:"
echo "---"

if command -v ros2 &> /dev/null; then
    ROS2_DISTRO=$(ros2 --version 2>/dev/null | grep -oP 'humble|iron|jazzy' || echo "unknown")
    if [ "$ROS2_DISTRO" = "humble" ]; then
        echo -e "${GREEN}✓${NC} ROS 2 Humble is installed"
        ((passed++))
    else
        echo -e "${YELLOW}⚠${NC}  Found ROS 2 $ROS2_DISTRO (expected Humble)"
        ((failed++))
    fi
else
    echo -e "${YELLOW}⚠${NC}  ROS 2 not found in PATH"
    echo "  → Please source /opt/ros/humble/setup.bash"
    ((failed++))
fi

echo ""

# Check package.xml files
echo "2️⃣  Package Configuration Files:"
echo "---"

check "[ -f package.xml ]" "Root package.xml exists"
check "[ -f apriltag_detector/package.xml ]" "apriltag_detector/package.xml exists"

echo ""

# Check package.xml content
echo "3️⃣  Package Metadata:"
echo "---"

if [ -f package.xml ]; then
    PACKAGE_NAME=$(grep -oP '(?<=<name>)[^<]+' package.xml | head -1)
    BUILD_TOOL=$(grep -oP '(?<=<buildtool_depend>)[^<]+' package.xml | head -1)
    
    echo "Root Package:"
    echo "  • Name: $PACKAGE_NAME"
    echo "  • Build Tool: $BUILD_TOOL"
    
    check "grep -q 'ament_cmake' package.xml" "  ✓ Uses ament_cmake"
    check "grep -q '<metapackage' package.xml" "  ✓ Configured as metapackage"
fi

if [ -f apriltag_detector/package.xml ]; then
    echo ""
    echo "SubPackage (apriltag_detector):"
    SUBPKG_NAME=$(grep -oP '(?<=<name>)[^<]+' apriltag_detector/package.xml | head -1)
    BUILD_TOOL=$(grep -oP '(?<=<buildtool_depend>)[^<]+' apriltag_detector/package.xml | head -1)
    echo "  • Name: $SUBPKG_NAME"
    echo "  • Build Tool: $BUILD_TOOL"
    
    check "grep -q 'ament_python' apriltag_detector/package.xml" "  ✓ Uses ament_python"
    check "grep -q '<build_type>ament_python</build_type>' apriltag_detector/package.xml" "  ✓ Build type set to ament_python"
fi

echo ""

# Check setup files
echo "4️⃣  Python Package Setup:"
echo "---"

check "[ -f apriltag_detector/setup.py ]" "setup.py exists"
check "[ -f apriltag_detector/setup.cfg ]" "setup.cfg exists"
check "[ -f apriltag_detector/pyproject.toml ]" "pyproject.toml exists"

if [ -f apriltag_detector/setup.py ]; then
    check "grep -q 'ament_python' apriltag_detector/setup.py || grep -q 'entry_points' apriltag_detector/setup.py" "  ✓ Contains ROS2 entry points"
fi

echo ""

# Check CMakeLists.txt
echo "5️⃣  CMake Configuration:"
echo "---"

check "[ -f CMakeLists.txt ]" "Root CMakeLists.txt exists"
check "grep -q 'ament_cmake_core_register_metapackage' CMakeLists.txt" "  ✓ Registered as metapackage"

echo ""

# Check Python dependencies
echo "6️⃣  Python Dependencies:"
echo "---"

dependencies=("numpy" "opencv-python" "pyrealsense2" "dt-apriltags" "scipy")

for dep in "${dependencies[@]}"; do
    python3 -c "import ${dep//-/_}" 2>/dev/null && {
        echo -e "${GREEN}✓${NC} $dep is installed"
        ((passed++))
    } || {
        echo -e "${YELLOW}⚠${NC}  $dep is not installed"
        echo "  → Install with: pip3 install $dep"
        ((failed++))
    }
done

echo ""

# Summary
echo "========================================"
echo "Summary:"
echo "========================================"
echo -e "Passed: ${GREEN}$passed${NC}"
echo -e "Failed: ${RED}$failed${NC}"
echo ""

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}✅ All checks passed!${NC}"
    echo ""
    echo "Your project is properly configured as a ROS2 Humble package."
    echo ""
    echo "Next steps:"
    echo "1. Build the workspace:"
    echo "   colcon build --symlink-install"
    echo ""
    echo "2. Source the workspace:"
    echo "   source install/setup.bash"
    echo ""
    echo "3. Run a package:"
    echo "   ros2 run apriltag_detector apriltag_map"
    exit 0
else
    echo -e "${RED}❌ Some checks failed!${NC}"
    echo ""
    echo "Please fix the issues above and try again."
    exit 1
fi
