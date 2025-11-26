#!/bin/bash
# ViperX-300S Docker Image Build Script
# 
# This script builds the Docker image containing:
# - ROS2 Humble
# - MoveIt2
# - Intel RealSense SDK
# - AprilTag detection libraries
# - Interbotix robot drivers

set -e  # Exit on error

# Change to docker directory
cd "$(dirname "$0")"

echo "════════════════════════════════════════════════════════════"
echo "  Building ViperX-300S AprilTag Grasping System"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "This may take 10-20 minutes on first build..."
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "✗ Error: Docker is not installed"
    echo "  Please install Docker first: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if docker compose is available
if ! docker compose version &> /dev/null; then
    echo "✗ Error: Docker Compose is not available"
    echo "  Please update Docker to a version that includes Compose V2"
    exit 1
fi

# Build the image
echo "Building Docker image..."
echo ""
docker compose build

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✓ Build completed successfully!"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "Image: viperx300s-ros2:humble"
echo ""
echo "Next steps:"
echo "  1. Start container:  ./run.sh"
echo "  2. Or manually:      docker compose up -d"
echo "  3. Enter container:  docker exec -it viperx300s_robot bash"
echo ""
echo "Quick start guide: ../README.md"
echo "════════════════════════════════════════════════════════════"
