"""
Complete AprilTag Robot Pipeline Launch

Launches the orchestrated pipeline that performs:
1. World frame calibration (AprilTags 0,1,2)
2. Target detection (AprilTag 3)
3. Motion planning (MoveIt2/Interbotix)
4. Grasp execution

Usage:
  # With real robot:
  ros2 launch apriltag_robot_control pipeline.launch.py robot_model:=vx300s hardware_type:=actual

  # Simulation (no physical hardware):
  ros2 launch apriltag_robot_control pipeline.launch.py robot_model:=vx300s hardware_type:=fake
  
  # Auto-start pipeline:
  ros2 launch apriltag_robot_control pipeline.launch.py auto_start:=true
  
  # Manual control:
  ros2 service call /start_pipeline std_srvs/srv/Trigger
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('apriltag_robot_control')
    
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='vx300s',
        description='Robot model (vx300s, wx250s, etc.)'
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
        description='AprilTag size in meters (default: 6.25cm)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start pipeline on launch'
    )
    
    approach_height_arg = DeclareLaunchArgument(
        'approach_height',
        default_value='0.15',
        description='Height above target for approach (meters)'
    )
    
    grasp_height_arg = DeclareLaunchArgument(
        'grasp_height',
        default_value='0.02',
        description='Height above target for final grasp (meters)'
    )
    
    retreat_height_arg = DeclareLaunchArgument(
        'retreat_height',
        default_value='0.15',
        description='Height to retreat after grasp (meters)'
    )
    
    # Include robot control launch (Interbotix + MoveIt)
    use_sim_value = PythonExpression([
        "'true' if '",
        LaunchConfiguration('hardware_type'),
        "' == 'fake' else 'false'"
    ])

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_name'),
            'hardware_type': LaunchConfiguration('hardware_type'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_sim': use_sim_value,
        }.items()
    )
    
    # AprilTag Pipeline Node (delayed start to allow robot to initialize)
    pipeline_node = TimerAction(
        period=10.0,  # Wait 10 seconds for robot to initialize
        actions=[
            Node(
                package='apriltag_robot_control',
                executable='apriltag_pipeline.py',
                name='apriltag_pipeline',
                output='screen',
                parameters=[{
                    'tag_size': LaunchConfiguration('tag_size'),
                    'robot_model': LaunchConfiguration('robot_model'),
                    'robot_name': LaunchConfiguration('robot_name'),
                    'auto_start': LaunchConfiguration('auto_start'),
                    'save_calibration': True,
                    'calibration_file': 'world_calibration.json',
                    'approach_height': LaunchConfiguration('approach_height'),
                    'grasp_height': LaunchConfiguration('grasp_height'),
                    'retreat_height': LaunchConfiguration('retreat_height'),
                    'world_frame': 'world',
                    'camera_frame': 'camera_link',
                }],
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        robot_model_arg,
        robot_name_arg,
        hardware_type_arg,
        use_rviz_arg,
        tag_size_arg,
        auto_start_arg,
        approach_height_arg,
        grasp_height_arg,
        retreat_height_arg,
        
        # Robot control
        robot_launch,
        
        # Pipeline node (delayed)
        pipeline_node,
    ])
