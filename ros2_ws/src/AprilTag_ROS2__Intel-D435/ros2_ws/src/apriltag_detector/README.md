# AprilTag Detector Package

一個 ROS 2 Python 套件，用於使用 Intel RealSense D435 攝像頭進行 AprilTag 檢測和攝像頭位置驗證。

## 功能

### 1. **apriltag_map** - AprilTag 地圖構建
- 檢測多個 AprilTag 並創建統一的地圖坐標系
- 將所有標籤位置轉換到一個統一的地圖框架中
- 支持臨時濾波以提高穩定性
- 將地圖數據保存為 JSON 格式

**運行方式：**
```bash
ros2 run apriltag_detector apriltag_map
```

### 2. **camera_position_validator** - 攝像頭位置驗證
- 驗證檢測到的攝像頭位置是否符合預期
- 支持誤差檢查和實時反饋
- 顯示攝像頭和標籤位置的驗證結果
- 公差可配置（默認 1.5cm）

**運行方式：**
```bash
ros2 run apriltag_detector camera_validator
```

### 3. **record_calibration_data** - 校準數據記錄
- 記錄攝像頭和機器人位姿對進行手眼校準
- 支持批量採樣和數據保存
- 輸出為 `.npy` 和 `.json` 格式

**運行方式：**
```bash
ros2 run apriltag_detector record_calibration
```

## 安裝

### 依賴項

#### ROS 2 依賴
```bash
sudo apt-get install python3-pip
```

#### Python 依賴
```bash
pip3 install opencv-python numpy pyrealsense2 dt-apriltags scipy
```

### 構建和安裝

**方法 1：使用設置腳本（推薦）**
```bash
~/AprilTag_ROS2_intel-D435/setup_apriltag_detector.sh
```

**方法 2：手動安裝**
```bash
cd ~/AprilTag_ROS2_intel-D435/apriltag_detector
pip3 install -e .
```

## 配置

### AprilTag 大小
編輯各個文件頂部的 `TAG_SIZE` 常數（默認為 6.25cm）：
```python
TAG_SIZE = 0.0625  # meters (6.25 cm)
```

### 攝像頭解析度
調整幀寬度和高度：
```python
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
```

### 檢測參數

在 `Detector` 初始化中調整：
```python
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=2.0,      # 降低以提高精度
    quad_sigma=0.8,         # 降低以進行更清晰的檢測
    refine_edges=1,
    decode_sharpening=0.25  # 增加銳化
)
```

### 預期位置和誤差容限（camera_position_validator）

編輯文件中的預期位置：
```python
EXPECTED_TAG_POSITIONS = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.197, -0.001, -0.021]),
    2: np.array([-0.001, -0.101, -0.004])
}

EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])
POSITION_TOLERANCE = 0.015  # 1.5cm
```

## 坐標系說明

### 地圖坐標系（Map Frame）
- **原點**：由 `MAP_ORIGIN_TAG_ID` 指定的 AprilTag（默認 Tag 0）
- **X 軸**：指向右（看著 Tag 0 時）
- **Y 軸**：指向上
- **Z 軸**：指向攝像頭方向

所有其他標籤位置相對於此原點報告。

## 輸出文件

### apriltag_map.json
```json
{
  "metadata": {
    "map_origin_tag_id": 0,
    "tag_size": 0.0625,
    "timestamp": "2025-11-24 10:30:45"
  },
  "tags": {
    "0": {
      "position": [0.0, 0.0, 0.0],
      "rotation_matrix": [[1, 0, 0], ...],
      "rotation_euler_deg": [0, 0, 0],
      "is_origin": true
    }
  }
}
```

### calibration_data.json
```json
{
  "metadata": {
    "tag_size": 0.0625,
    "num_samples": 5
  },
  "samples": [
    {
      "sample": 1,
      "camera_pose": [...],
      "robot_pose": [...]
    }
  ]
}
```

## 文件結構

```
apriltag_detector/
├── apriltag_detector/
│   ├── __init__.py
│   ├── apriltag_map.py              # 地圖構建模塊
│   ├── camera_position_validator.py # 位置驗證模塊
│   └── record_calibration_data.py   # 校準數據記錄模塊
├── resource/
│   └── apriltag_detector
├── package.xml                      # ROS 2 套件定義
├── setup.py                         # Python 設置配置
└── setup.cfg                        # setuptools 配置
```

## 故障排除

### 沒有檢測到 AprilTag
1. 確保標籤清晰可見
2. 調整 `quad_decimate` 參數（降低以提高精度）
3. 增加照明
4. 檢查攝像頭焦點設置

### 位置誤差過大
1. 檢查攝像頭校準參數
2. 確保標籤尺寸設置正確
3. 增加臨時濾波窗口大小
4. 驗證標籤物理位置

### 攝像頭連接問題
1. 確認 Intel D435 已連接
2. 檢查 USB 連接（推薦使用 USB 3.0）
3. 驗證 pyrealsense2 驅動已安裝

## 許可證

BSD

## 作者

David

## 相關資源

- [AprilTag](https://april.eecs.umich.edu/software/apriltag/)
- [RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
