# AprilTag ROS2 Intel D435 - 完整使用指南

## 📋 目錄

1. [快速開始](#快速開始)
2. [Docker 使用](#docker-使用)
3. [本機安裝](#本機安裝)
4. [驗證安裝](#驗證安裝)
5. [常見問題](#常見問題)

---

## 快速開始

### 使用 Docker（推薦）

```bash
# 1. 進入專案目錄
cd /home/david/AprilTag_ROS2_intel-D435

# 2. 執行 Docker 設置腳本
bash docker-setup.sh

# 3. 進入容器
docker exec -it apriltag_humble bash

# 4. 驗證安裝
source /root/ws/install/setup.bash
ros2 pkg list | grep apriltag
```

### 在本機運行（無 Docker）

```bash
# 1. 進入專案目錄
cd /home/david/AprilTag_ROS2_intel-D435

# 2. 安裝 Python 依賴
pip3 install numpy opencv-python pyrealsense2 dt-apriltags scipy

# 3. 編譯工作區
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 4. Source 設置文件
source install/setup.bash

# 5. 運行 AprilTag 檢測器
ros2 run apriltag_detector apriltag_map
```

---

## Docker 使用

### 優點

✅ 隔離的環境，不影響系統
✅ 一致的開發環境
✅ 易於部署和分享
✅ 容易清理

### 構建映像

```bash
# 方法 1: 使用 docker-compose（推薦）
docker-compose build

# 方法 2: 直接使用 Docker
docker build -t apriltag_ros2:humble .
```

### 運行容器

```bash
# 方法 1: 使用 docker-compose
docker-compose up -d

# 方法 2: 使用 docker-compose 交互式
docker-compose up -it

# 方法 3: 直接使用 Docker
docker run -it \
  --name apriltag_humble \
  --device=/dev/bus/usb:/dev/bus/usb \
  --privileged \
  --net=host \
  -v $(pwd):/root/ws/src/apriltag_ros2_intel_d435/ \
  apriltag_ros2:humble
```

### 容器內操作

```bash
# 進入運行中的容器
docker exec -it apriltag_humble bash

# 查看容器日誌
docker logs apriltag_humble

# 停止容器
docker-compose stop

# 重啟容器
docker-compose restart

# 移除容器
docker-compose down
```

### 在容器內運行應用

```bash
# 進入容器後
docker exec -it apriltag_humble bash

# Source ROS 2 setup
source /root/ws/install/setup.bash

# 列出所有 ROS 2 包
ros2 pkg list

# 列出 AprilTag 相關的包
ros2 pkg list | grep apriltag

# 運行 AprilTag 地圖構建器
ros2 run apriltag_detector apriltag_map

# 運行相機驗證器
ros2 run apriltag_detector camera_validator

# 運行校準數據記錄器
ros2 run apriltag_detector record_calibration
```

---

## 本機安裝

### 系統要求

- Ubuntu 22.04（ROS 2 Humble 推薦環境）
- ROS 2 Humble 已安裝
- Python 3.10 或更高版本
- CMake 3.8 或更高版本
- Colcon 構建工具

### 安裝步驟

#### 1. 安裝 ROS 2 Humble

```bash
# 如果還未安裝，參考官方文檔
# https://docs.ros.org/en/humble/Installation.html

# Ubuntu 安裝示例
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2. 安裝必要的依賴

```bash
# 系統包
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake

# Python 包
pip3 install --upgrade \
  numpy \
  opencv-python \
  pyrealsense2 \
  dt-apriltags \
  scipy

# Intel RealSense SDK（如需使用 D435 相機）
sudo apt-get install -y librealsense2-dev
```

#### 3. 編譯專案

```bash
# 進入專案目錄
cd /home/david/AprilTag_ROS2_intel-D435

# Source ROS 2
source /opt/ros/humble/setup.bash

# 編譯整個工作區
colcon build --symlink-install

# 或只編譯特定包
colcon build --packages-select apriltag_detector --symlink-install
```

#### 4. Source 環境

```bash
# Source 構建輸出
source /home/david/AprilTag_ROS2_intel-D435/install/setup.bash

# 驗證
ros2 pkg list | grep apriltag
```

---

## 驗證安裝

### 檢查清單

```bash
# 1. 檢查 ROS 2 安裝
source /opt/ros/humble/setup.bash
ros2 pkg list | head

# 2. 檢查 AprilTag 包
ros2 pkg list | grep apriltag
# 應該輸出:
# apriltag_detector
# apriltag_ros2_intel_d435

# 3. 檢查 Python 依賴
python3 -c "import cv2, numpy, pyrealsense2, dt_apriltags, scipy; print('✓ All dependencies OK')"

# 4. 檢查 Intel RealSense 相機（如已連接）
python3 -c "import pyrealsense2 as rs; ctx = rs.context(); print(f'Found {len(ctx.query_devices())} RealSense devices')"

# 5. 運行一個簡單的測試
source install/setup.bash
ros2 run apriltag_detector apriltag_map --help
```

### 驗證腳本

提供了自動驗證腳本 `verify-ros2-setup.sh`

```bash
bash verify-ros2-setup.sh
```

此腳本會檢查：
- ✓ ROS 2 Humble 安裝
- ✓ package.xml 配置
- ✓ Python 依賴
- ✓ 構建工具配置

---

## 常見問題

### Q1: 相機無法連接

**症狀**: `pyrealsense2` 導入失敗或無法檢測相機

**解決方案**:

```bash
# 1. 檢查 USB 連接
lsusb | grep Intel

# 2. 安裝正確的驅動程式
sudo apt-get install librealsense2-dev

# 3. 添加用戶到 plugdev 組
sudo usermod -a -G plugdev $USER
sudo usermod -a -G video $USER
newgrp video

# 4. 在 Docker 中，確保設備權限
docker run --privileged --device=/dev/bus/usb:/dev/bus/usb ...
```

### Q2: "Package not found" 錯誤

**症狀**: `ros2 run apriltag_detector ...` 失敗

**解決方案**:

```bash
# 1. 確保已 source 設置文件
source install/setup.bash

# 2. 檢查包是否存在
ros2 pkg list | grep apriltag

# 3. 重新構建
rm -rf build install log
colcon build --symlink-install

# 4. Source 新的設置
source install/setup.bash
```

### Q3: Docker 構建失敗

**症狀**: `docker-compose build` 或 `docker build` 失敗

**解決方案**:

```bash
# 1. 檢查 Docker 是否在運行
docker version

# 2. 檢查網絡連接
docker run --rm alpine ping -c 1 8.8.8.8

# 3. 清理 Docker 資源
docker system prune -a

# 4. 重新構建（無緩存）
docker-compose build --no-cache
```

### Q4: Python 依賴安裝失敗

**症狀**: `pip3 install` 出現錯誤

**解決方案**:

```bash
# 1. 升級 pip
pip3 install --upgrade pip setuptools wheel

# 2. 安裝系統依賴
sudo apt-get install -y python3-dev python3-pip python3-venv

# 3. 逐個安裝包，查看哪個失敗
pip3 install numpy
pip3 install opencv-python
pip3 install pyrealsense2
pip3 install dt-apriltags
pip3 install scipy

# 4. 檢查是否安裝成功
python3 -c "import cv2; print(cv2.__version__)"
```

### Q5: ROS 2 通信問題

**症狀**: 無法發現節點或主題

**解決方案**:

```bash
# 1. 檢查 ROS_DOMAIN_ID（在同一網絡上應相同）
echo $ROS_DOMAIN_ID

# 2. 確保 ROS 2 daemon 運行
ros2 daemon start

# 3. 列出可用主題
ros2 topic list

# 4. 查看節點
ros2 node list

# 5. 檢查環境變量
printenv | grep ROS
```

### Q6: 容器時間同步問題

**症狀**: 容器內時間與主機不同步

**解決方案**:

編輯 `docker-compose.yml`：

```yaml
volumes:
  - /etc/timezone:/etc/timezone:ro
  - /etc/localtime:/etc/localtime:ro
```

---

## 項目結構

```
apriltag_ros2_intel_d435/                      # 主 ROS 2 Metapackage
├── Dockerfile                                 # Docker 構建配置
├── docker-compose.yml                         # Docker Compose 配置
├── docker-setup.sh                            # Docker 自動設置腳本
├── verify-ros2-setup.sh                       # ROS 2 驗證腳本
├── package.xml                                # ROS 2 metapackage 定義
├── CMakeLists.txt                             # ROS 2 metapackage CMake
├── apriltag_detector/                         # Python 應用包
│   ├── apriltag_detector/                     # Python 模塊
│   ├── package.xml                            # ROS 2 Python 包定義
│   ├── setup.py                               # Python 安裝配置
│   ├── setup.cfg                              # Setup 配置
│   ├── pyproject.toml                         # Python 項目配置
│   └── README.md                              # 包文檔
├── apriltag/                                  # C++ AprilTag 庫
├── apriltag_msgs/                             # 自定義 ROS 2 消息
├── apriltag_ros/                              # ROS 2 節點實現
├── camera_info/                               # 相機校準數據
├── data/                                      # 數據文件
├── build/                                     # 構建輸出
├── install/                                   # 安裝輸出
└── log/                                       # 構建日誌
```

---

## 開發工作流程

### 本機開發

```bash
# 編輯代碼...
cd apriltag_detector

# 重新構建
cd ..
colcon build --packages-select apriltag_detector --symlink-install

# Source 並測試
source install/setup.bash
ros2 run apriltag_detector apriltag_map
```

### Docker 開發

```bash
# 啟動容器
docker-compose up -d

# 編輯代碼（主機上）
# 容器會看到掛載的文件變化

# 在容器內重新構建
docker exec -it apriltag_humble bash
cd /root/ws
colcon build --packages-select apriltag_detector --symlink-install
source install/setup.bash

# 測試
ros2 run apriltag_detector apriltag_map
```

---

## 下一步

1. **創建啟動文件**: 在 `launch/` 目錄中創建 ROS 2 launch 文件
2. **添加服務**: 實現 ROS 2 服務用於相機控制
3. **節點轉換**: 將腳本轉換為完整的 ROS 2 節點（使用 `rclpy.Node`）
4. **測試**: 添加單元測試和集成測試
5. **文檔**: 生成 API 文檔

---

## 資源

- [ROS 2 Humble 文檔](https://docs.ros.org/en/humble/)
- [AprilTag 庫](https://april.eecs.umich.edu/software/apriltag/)
- [Intel RealSense D435 文檔](https://www.intelrealsense.com/depth-camera-d435/)
- [Colcon 用戶指南](https://colcon.readthedocs.io/)
- [Docker 文檔](https://docs.docker.com/)

---

**最後更新**: 2025年11月24日
**ROS 2 版本**: Humble
**Python 版本**: 3.10+
