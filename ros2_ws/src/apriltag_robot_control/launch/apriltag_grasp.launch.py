"""
Launch file for AprilTag-based robot manipulation demo.

This launch file starts all necessary nodes for:
1. World frame calibration using AprilTags 0, 1, 2
2. Target detection (AprilTag 3)
3. Robot control with ViperX-300S

Usage:
  ros2 launch apriltag_robot_control apriltag_grasp.launch.py
  
  With robot arm control:
  ros2 launch apriltag_robot_control apriltag_grasp.launch.py hardware_type:=actual
  
  Simulation mode:
  ros2 launch apriltag_robot_control apriltag_grasp.launch.py hardware_type:=fake
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('apriltag_robot_control')
    
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='vx300s',
        description='Robot model (vx300s)'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='vx300s',
        description='Robot namespace'
    )
    
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='fake',
        choices=['actual', 'fake'],
        description='Hardware type: actual (real robot) or fake (simulation)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.0625',
        description='AprilTag size in meters'
    )
    
    # Configuration file
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # AprilTag Grasp Demo Node (main node)
    apriltag_demo_node = Node(
        package='apriltag_robot_control',
        executable='apriltag_grasp_demo.py',
        name='apriltag_grasp_demo',
        output='screen',
        parameters=[{
            'tag_size': LaunchConfiguration('tag_size'),
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_name'),
            'use_moveit': False,
            'auto_execute': False,
        }],
    )
    
    return LaunchDescription([
        # Arguments
        robot_model_arg,
        robot_name_arg,
        hardware_type_arg,
        use_rviz_arg,
        tag_size_arg,
        
        # Nodes
        apriltag_demo_node,
    ])
