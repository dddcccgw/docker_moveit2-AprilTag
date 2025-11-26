# 🚀 apriltag_detector 快速參考

## 一行命令快速開始

### 方法 1：使用設置腳本（推薦）
```bash
# 1. 構建
cd ~/AprilTag_ROS2_intel-D435 && colcon build --packages-select apriltag_detector && source install/setup.bash

# 2. 運行地圖構建
ros2 run apriltag_detector apriltag_map

# 3. 運行驗證（新終端）
ros2 run apriltag_detector camera_validator

# 4. 運行校準數據記錄（新終端）
ros2 run apriltag_detector record_calibration
```

## 檔案位置速查

| 組件 | 路徑 |
|------|------|
| 套件根目錄 | `/home/david/AprilTag_ROS2_intel-D435/apriltag_detector/` |
| 源代碼 | `/home/david/AprilTag_ROS2_intel-D435/apriltag_detector/apriltag_detector/*.py` |
| package.xml | `/home/david/AprilTag_ROS2_intel-D435/apriltag_detector/package.xml` |
| setup.py | `/home/david/AprilTag_ROS2_intel-D435/apriltag_detector/setup.py` |
| 文檔 | `/home/david/AprilTag_ROS2_intel-D435/apriltag_detector/README.md` |

## 三個模塊一覽

| 模塊 | 命令 | 輸出 | 用途 |
|------|------|------|------|
| **apriltag_map** | `ros2 run apriltag_detector apriltag_map` | apriltag_map.json | 建立地圖 |
| **camera_validator** | `ros2 run apriltag_detector camera_validator` | 驗證報告 | 驗證位置 |
| **record_calibration** | `ros2 run apriltag_detector record_calibration` | data/*.npy | 記錄校準數據 |

## 主要類

- `AprilTagDetector` - 地圖構建
- `CameraPositionValidator` - 位置驗證  
- `CalibrationDataRecorder` - 數據記錄
- `TemporalFilter` - 穩定性濾波

## 關鍵參數

```python
TAG_SIZE = 0.0625           # 標籤大小（米）
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
MAP_ORIGIN_TAG_ID = 0       # 原點標籤 ID
POSITION_TOLERANCE = 0.015  # 誤差容限（米）
```

## 調試技巧

- 🔍 **沒看到標籤？** 降低 `quad_decimate` 參數
- 📏 **誤差太大？** 檢查 `TAG_SIZE` 設置
- 🎥 **攝像頭問題？** 確保 Intel D435 已連接（USB 3.0）
- 🔗 **導入錯誤？** `pip3 install pyrealsense2 dt-apriltags opencv-python scipy`

## 輸出示例

```
=== Map Frame Status ===
Tag 0 [ORIGIN]: (0.000, 0.000, 0.000) m
Tag 1: (0.197, -0.001, -0.021) m
Tag 2: (-0.001, -0.101, -0.004) m
```

---

詳見 `README.md` 和 `APRILTAG_DETECTOR_SUMMARY.md`
