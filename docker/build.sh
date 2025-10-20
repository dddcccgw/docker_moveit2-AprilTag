#!/bin/bash

# 切換到 docker 目錄
cd "$(dirname "$0")"

echo "======================================"
echo "Building ViperX-300S Docker Image..."
echo "======================================"

# 使用 docker compose 建置映像
docker compose build

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================"
    echo "✓ Build completed successfully!"
    echo "======================================"
    echo ""
    echo "Next steps:"
    echo "  1. Run:  ./run.sh"
    echo "  2. Or:   docker compose up -d"
    echo ""
else
    echo ""
    echo "======================================"
    echo "✗ Build failed. Please check errors above."
    echo "======================================"
    exit 1
fi
