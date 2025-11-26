#!/bin/bash
# AprilTag Detector 環境設置腳本

echo "🚀 設置 AprilTag Detector 環境..."

# 確保已安裝依賴
echo "📦 檢查依賴..."
pip3 install -q opencv-python numpy pyrealsense2 dt-apriltags scipy

# 安裝 apriltag_detector 套件
echo "🔧 安裝 apriltag_detector 套件..."
cd "$(dirname "$0")/apriltag_detector"
pip3 install -e . -q --no-deps

echo ""
echo "✅ 設置完成！"
echo ""
echo "可用的命令："
echo "  1️⃣  apriltag_map              - 構建 AprilTag 地圖"
echo "  2️⃣  camera_validator          - 驗證攝像頭位置"
echo "  3️⃣  record_calibration        - 記錄校準數據"
echo ""
echo "或使用 Python 模塊："
echo "  python3 -m apriltag_detector.apriltag_map"
echo "  python3 -m apriltag_detector.camera_position_validator"
echo "  python3 -m apriltag_detector.record_calibration_data"
echo ""
