from setuptools import setup, find_packages

package_name = 'apriltag_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pyrealsense2',
        'dt-apriltags',
        'scipy',
    ],
    zip_safe=True,
    maintainer='David',
    maintainer_email='david@example.com',
    description='AprilTag detection and camera position validation with Intel D435',
    long_description='Complete package for AprilTag detection, pose validation, and calibration using Intel RealSense D435 camera in ROS 2',
    long_description_content_type='text/markdown',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_map = apriltag_detector.apriltag_map:main',
            'camera_validator = apriltag_detector.camera_position_validator:main',
            'record_calibration = apriltag_detector.record_calibration_data:main',
        ],
    },
)
