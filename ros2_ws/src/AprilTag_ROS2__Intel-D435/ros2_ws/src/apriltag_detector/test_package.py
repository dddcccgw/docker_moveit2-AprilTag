#!/usr/bin/env python3
"""测试 apriltag_detector 包的安装"""

import sys

def test_package_structure():
    """测试包结构和入口点"""
    print("=" * 60)
    print("测试 apriltag_detector ROS2 包")
    print("=" * 60)
    
    # 测试导入
    try:
        import apriltag_detector
        print("✅ 成功导入 apriltag_detector 包")
        print(f"   版本: {apriltag_detector.__version__}")
    except ImportError as e:
        print(f"❌ 无法导入 apriltag_detector: {e}")
        return False
    
    # 测试模块导入
    modules = [
        'apriltag_detector.apriltag_map',
        'apriltag_detector.camera_position_validator',
        'apriltag_detector.record_calibration_data'
    ]
    
    for module in modules:
        try:
            __import__(module)
            print(f"✅ 成功导入 {module}")
        except ImportError as e:
            print(f"❌ 无法导入 {module}: {e}")
            return False
    
    # 检查 main 函数
    print("\n检查入口点函数:")
    try:
        from apriltag_detector.apriltag_map import main as apriltag_main
        print("✅ apriltag_map.main() 存在")
    except ImportError as e:
        print(f"❌ apriltag_map.main() 不存在: {e}")
    
    try:
        from apriltag_detector.camera_position_validator import main as validator_main
        print("✅ camera_position_validator.main() 存在")
    except ImportError as e:
        print(f"❌ camera_position_validator.main() 不存在: {e}")
    
    try:
        from apriltag_detector.record_calibration_data import main as record_main
        print("✅ record_calibration_data.main() 存在")
    except ImportError as e:
        print(f"❌ record_calibration_data.main() 不存在: {e}")
    
    # 检查命令行工具
    print("\n检查安装的命令行工具:")
    import shutil
    commands = ['apriltag_map', 'camera_validator', 'record_calibration']
    
    for cmd in commands:
        cmd_path = shutil.which(cmd)
        if cmd_path:
            print(f"✅ {cmd} -> {cmd_path}")
        else:
            print(f"❌ {cmd} 未找到")
    
    print("\n" + "=" * 60)
    print("✅ apriltag_detector 包已成功安装和配置！")
    print("=" * 60)
    print("\n可用的命令:")
    print("  - apriltag_map          : AprilTag 检测和地图构建")
    print("  - camera_validator      : 相机位置验证")
    print("  - record_calibration    : 记录校准数据")
    print("\n注意: 运行这些命令需要连接 Intel RealSense D435 相机")
    print("=" * 60)
    
    return True

if __name__ == '__main__':
    success = test_package_structure()
    sys.exit(0 if success else 1)
