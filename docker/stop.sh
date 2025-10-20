#!/bin/bash

# 切換到 docker 目錄
cd "$(dirname "$0")"

echo "======================================"
echo "Stopping ViperX-300S Container..."
echo "======================================"

# 停止容器
docker compose down

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================"
    echo "✓ Container stopped successfully!"
    echo "======================================"
    echo ""
else
    echo ""
    echo "======================================"
    echo "✗ Failed to stop container."
    echo "======================================"
    exit 1
fi
