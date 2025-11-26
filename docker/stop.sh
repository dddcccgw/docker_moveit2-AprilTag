#!/bin/bash
# ViperX-300S Container Stop Script
# 
# This script gracefully stops the Docker container and cleans up resources

set -e  # Exit on error

# Change to docker directory
cd "$(dirname "$0")"

CONTAINER_NAME="viperx300s_robot"

echo "════════════════════════════════════════════════════════════"
echo "  Stopping ViperX-300S Container"
echo "════════════════════════════════════════════════════════════"
echo ""

# Check if container is running
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "ℹ Container is not running"
    echo ""
    # Check if container exists but is stopped
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        echo "Removing stopped container..."
        docker compose down
    else
        echo "No container found. Nothing to do."
    fi
    exit 0
fi

echo "Stopping container gracefully..."
docker compose down

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✓ Container stopped successfully!"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "To start again: ./run.sh"
echo "════════════════════════════════════════════════════════════"
