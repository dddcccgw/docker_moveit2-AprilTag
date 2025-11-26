#!/bin/bash
# ViperX-300S Container Startup Script
# 
# This script:
# 1. Configures X11 display forwarding for GUI applications (RViz2)
# 2. Starts the Docker container in background
# 3. Sets up USB device symlinks for robot hardware
# 4. Optionally enters the container shell

set -e  # Exit on error

# Change to docker directory
cd "$(dirname "$0")"

CONTAINER_NAME="viperx300s_robot"

# Check if container is already running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "════════════════════════════════════════════════════════════"
    echo "  Container is already running!"
    echo "════════════════════════════════════════════════════════════"
    echo ""
    
    # Verify hardware connections
    echo "Checking hardware connections..."
    docker exec $CONTAINER_NAME bash -c '
        # Check robot arm connection
        if [ -e "/dev/ttyUSB0" ]; then
            echo "  ✓ Robot arm detected: /dev/ttyUSB0"
            if [ ! -e "/dev/ttyDXL" ]; then
                ln -sf /dev/ttyUSB0 /dev/ttyDXL
                echo "  ✓ Created symlink: /dev/ttyDXL -> /dev/ttyUSB0"
            fi
        else
            echo "  ⚠ Robot arm NOT detected: /dev/ttyUSB0"
            echo "    (Use hardware_type:=fake for simulation)"
        fi
        
        # Check camera connection
        if command -v rs-enumerate-devices &> /dev/null; then
            if rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense"; then
                echo "  ✓ RealSense camera detected"
            else
                echo "  ⚠ RealSense camera NOT detected"
            fi
        fi
    ' 2>/dev/null || true
    
    echo ""
    echo "Entering container..."
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

echo "════════════════════════════════════════════════════════════"
echo "  Starting ViperX-300S Container"
echo "════════════════════════════════════════════════════════════"
echo ""

# Configure X11 for GUI applications (RViz2, MoveIt2 visualization)
echo "Configuring X11 display forwarding..."
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || true
    chmod 644 $XAUTH
fi

# Allow Docker containers to connect to X server
xhost +local:docker > /dev/null 2>&1 || echo "⚠ Warning: Could not configure xhost (GUI may not work)"

echo "Starting container in background..."
echo ""

# Start container
docker compose up -d

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✓ Container started successfully!"
echo "════════════════════════════════════════════════════════════"
echo ""

# Setup hardware device symlinks
echo "Setting up hardware connections..."
docker exec $CONTAINER_NAME bash -c '
    # Create symlink for robot arm
    if [ -e "/dev/ttyUSB0" ]; then
        ln -sf /dev/ttyUSB0 /dev/ttyDXL
        echo "  ✓ Robot arm: /dev/ttyUSB0 -> /dev/ttyDXL"
    else
        echo "  ⚠ Robot arm NOT detected (use hardware_type:=fake for simulation)"
    fi
    
    # Check camera
    if command -v rs-enumerate-devices &> /dev/null; then
        if rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense"; then
            echo "  ✓ RealSense camera detected"
        else
            echo "  ⚠ RealSense camera NOT detected"
        fi
    fi
' 2>/dev/null || true

echo ""
echo "Container Status:"
echo "  Name: $CONTAINER_NAME"
echo "  Image: viperx300s-ros2:humble"
echo ""

# Ask if user wants to enter container
read -p "Enter container now? (Y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    echo ""
    echo "Entering container..."
    echo "════════════════════════════════════════════════════════════"
    docker exec -it $CONTAINER_NAME bash
else
    echo ""
    echo "Container is running in background."
    echo ""
    echo "Useful commands:"
    echo "  Enter container:  ./run.sh  (or docker exec -it $CONTAINER_NAME bash)"
    echo "  View logs:        docker compose logs -f"
    echo "  Stop container:   ./stop.sh"
    echo ""
    echo "Quick Start Guide: ../README.md"
    echo "════════════════════════════════════════════════════════════"
fi
