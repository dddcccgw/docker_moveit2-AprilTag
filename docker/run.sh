#!/bin/bash

# 切換到 docker 目錄
cd "$(dirname "$0")"

CONTAINER_NAME="viperx300s_robot"

# 檢查容器是否已經在執行
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "======================================"
    echo "Container is already running!"
    echo "======================================"
    echo ""
    
    # 確保 ttyDXL 符號連結存在
    docker exec $CONTAINER_NAME bash -c '
        if [ -e "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyDXL" ]; then
            ln -sf /dev/ttyUSB0 /dev/ttyDXL
            echo "✓ Created symlink: /dev/ttyDXL -> /dev/ttyUSB0"
        fi
    ' 2>/dev/null
    
    echo "Entering container..."
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

echo "======================================"
echo "Starting ViperX-300S Container..."
echo "======================================"

# 設定 X11 授權 (允許 Docker 容器使用 GUI)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod 644 $XAUTH
fi

# 允許本地 X server 接受連線
xhost +local:docker > /dev/null 2>&1

# 啟動容器 (背景執行)
docker compose up -d

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================"
    echo "✓ Container started successfully!"
    echo "======================================"
    echo ""
    
    # 在容器內創建 ttyDXL 符號連結 (如果 ttyUSB0 存在)
    docker exec $CONTAINER_NAME bash -c '
        if [ -e "/dev/ttyUSB0" ]; then
            ln -sf /dev/ttyUSB0 /dev/ttyDXL
            echo "✓ Created symlink: /dev/ttyDXL -> /dev/ttyUSB0"
        fi
    ' 2>/dev/null
    
    # 詢問是否要進入容器
    read -p "Do you want to enter the container now? (Y/n): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
        echo "Entering container..."
        docker exec -it $CONTAINER_NAME bash
    else
        echo ""
        echo "Container is running in background."
        echo ""
        echo "To enter the container later:"
        echo "  ./run.sh"
        echo "  or: docker exec -it $CONTAINER_NAME bash"
        echo ""
        echo "To view logs:"
        echo "  docker compose logs -f"
        echo ""
        echo "To stop:"
        echo "  ./stop.sh"
        echo ""
    fi
else
    echo ""
    echo "======================================"
    echo "✗ Failed to start container."
    echo "======================================"
    exit 1
fi
