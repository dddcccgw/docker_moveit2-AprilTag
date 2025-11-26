#!/bin/bash

# AprilTag ROS2 Docker Setup Script
# This script builds and runs the Docker container for AprilTag ROS2 project

set -e

echo "========================================"
echo "AprilTag ROS2 Humble Docker Setup"
echo "========================================"

# Check if docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed!"
    echo "Please install Docker from: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo "⚠️  docker-compose is not installed!"
    echo "Installing docker-compose..."
    sudo apt-get update
    sudo apt-get install -y docker-compose
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Build Docker image
echo ""
echo "📦 Building Docker image..."
docker-compose build

# Start container
echo ""
echo "🚀 Starting container..."
docker-compose up -d

# Get container ID
CONTAINER_ID=$(docker-compose ps -q apriltag_ros2)
CONTAINER_NAME="apriltag_humble"

if [ -z "$CONTAINER_ID" ]; then
    echo "❌ Failed to start container"
    exit 1
fi

echo "✅ Container started successfully!"
echo ""
echo "Container ID: $CONTAINER_ID"
echo "Container Name: $CONTAINER_NAME"
echo ""
echo "========================================"
echo "Next Steps:"
echo "========================================"
echo ""
echo "1. Enter the container:"
echo "   docker exec -it $CONTAINER_NAME bash"
echo ""
echo "2. Or attach to running container:"
echo "   docker attach $CONTAINER_NAME"
echo ""
echo "3. Inside the container, verify installation:"
echo "   ros2 --version"
echo "   ros2 pkg list | grep apriltag"
echo ""
echo "4. Run AprilTag detector:"
echo "   source /root/ws/install/setup.bash"
echo "   ros2 run apriltag_detector apriltag_map"
echo ""
echo "========================================"
echo ""
