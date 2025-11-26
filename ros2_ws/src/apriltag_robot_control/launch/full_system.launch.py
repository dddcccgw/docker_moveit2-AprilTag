"""
Full System Launch for AprilTag-based Robot Manipulation

This launch file starts the complete system including:
1. ViperX-300S robot arm with MoveIt2
2. AprilTag detection and world calibration
3. Grasp control

Usage:
  # With real robot:
  ros2 launch apriltag_robot_control full_system.launch.py hardware_type:=actual
  
  # Simulation mode (no physical robot):
  ros2 launch apriltag_robot_control full_system.launch.py hardware_type:=fake
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('apriltag_robot_control')
    
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='vx300s',
        description='Robot model'
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
        description='Hardware type'
    )
    
    use_moveit_rviz_arg = DeclareLaunchArgument(
        'use_moveit_rviz',
        default_value='true',
        description='Launch MoveIt RViz'
    )
    
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.0625',
        description='AprilTag size in meters'
    )
    
    # Include MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_name'),
            'hardware_type': LaunchConfiguration('hardware_type'),
            'use_moveit_rviz': LaunchConfiguration('use_moveit_rviz'),
        }.items()
    )
    
    # AprilTag Grasp Demo Node (delayed start to allow robot to initialize)
    apriltag_demo_node = TimerAction(
        period=5.0,  # Wait 5 seconds for robot to initialize
        actions=[
            Node(
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
        ]
    )
    
    return LaunchDescription([
        # Arguments
        robot_model_arg,
        robot_name_arg,
        hardware_type_arg,
        use_moveit_rviz_arg,
        tag_size_arg,
        
        # Robot with MoveIt
        moveit_launch,
        
        # AprilTag demo (delayed)
        apriltag_demo_node,
    ])
