#!/usr/bin/env python3
"""
AprilTag Robot Pipeline - Complete Orchestration

This node orchestrates the complete workflow:
Step 1: Use 3 AprilTags (id:0,1,2) to build world frame map and locate camera
Step 2: Get the location of AprilTag (id:3) in world frame
Step 3: Plan motion to AprilTag 3 using MoveIt2
Step 4: Execute grasp of object with AprilTag 3

State Machine:
  INIT -> CALIBRATING -> DETECTING -> PLANNING -> EXECUTING -> DONE
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import pyrealsense2 as rs
from dt_apriltags import Detector
import time
import json
from enum import Enum
from pathlib import Path

from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


class PipelineState(Enum):
    """State machine states for the pipeline."""
    INIT = 0
    CALIBRATING = 1
    CALIBRATED = 2
    DETECTING = 3
    TARGET_DETECTED = 4
    PLANNING = 5
    PLANNED = 6
    EXECUTING = 7
    DONE = 8
    ERROR = 9


class AprilTagPipeline(Node):
    """
    Complete pipeline orchestration for AprilTag-based robot manipulation.
    """
    
    def __init__(self):
        super().__init__('apriltag_pipeline')
        
        # Parameters
        self.declare_parameter('tag_size', 0.0625)  # 6.25 cm
        self.declare_parameter('robot_model', 'vx300s')
        self.declare_parameter('robot_name', 'vx300s')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'vx300s/base_link')
        self.declare_parameter('ee_frame', 'vx300s/ee_gripper_link')
        self.declare_parameter('planning_group', 'interbotix_arm')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('save_calibration', True)
        self.declare_parameter('calibration_file', 'world_calibration.json')
        self.declare_parameter('approach_height', 0.15)
        self.declare_parameter('grasp_height', 0.02)
        self.declare_parameter('retreat_height', 0.15)
        
        # Get parameters
        self.tag_size = self.get_parameter('tag_size').value
        self.robot_model = self.get_parameter('robot_model').value
        self.robot_name = self.get_parameter('robot_name').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.planning_group = self.get_parameter('planning_group').value
        self.auto_start = self.get_parameter('auto_start').value
        self.save_calibration = self.get_parameter('save_calibration').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.approach_height = self.get_parameter('approach_height').value
        self.grasp_height = self.get_parameter('grasp_height').value
        self.retreat_height = self.get_parameter('retreat_height').value
        
        # Known calibration tag positions
        self.calibration_tags = {
            0: np.array([0.0, 0.0, 0.0]),      # Origin
            1: np.array([0.15, 0.0, 0.0]),     # X-axis (15cm)
            2: np.array([0.0, 0.0, 0.15])      # Z-axis (15cm)
        }
        self.target_tag_id = 3
        
        # State
        self.state = PipelineState.INIT
        self.T_world_to_camera = None
        self.target_pose_world = None
        self.robot_initialized = False
        self.moveit_initialized = False
        self.moveit_interface = None
        
        # Filters for stable detection
        self.camera_pose_filter = TemporalFilter(window_size=10)
        self.target_pose_filter = TemporalFilter(window_size=5)
        
        # Initialize camera
        self._init_camera()
        
        # Initialize AprilTag detector
        self._init_detector()
        
        # Initialize robot (Interbotix API)
        self.robot = None
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.state_pub = self.create_publisher(String, 'pipeline_state', 10)
        self.calibration_pub = self.create_publisher(Bool, 'calibration_status', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.grasp_pose_pub = self.create_publisher(PoseStamped, 'grasp_pose', 10)
        
        # Services
        self.start_srv = self.create_service(
            Trigger, 'start_pipeline', self.start_pipeline_callback,
            callback_group=self.callback_group
        )
        self.reset_srv = self.create_service(
            Trigger, 'reset_pipeline', self.reset_pipeline_callback,
            callback_group=self.callback_group
        )
        
        # Main processing timer
        self.timer = self.create_timer(0.033, self.process_state)  # ~30 Hz
        
        # Robot initialization timer
        self.robot_init_timer = self.create_timer(3.0, self.try_init_robot)
        
        self.get_logger().info('='*60)
        self.get_logger().info('AprilTag Robot Pipeline Initialized')
        self.get_logger().info('='*60)
        self.get_logger().info('Pipeline Steps:')
        self.get_logger().info('  1. CALIBRATING: Use tags 0,1,2 to build world frame')
        self.get_logger().info('  2. DETECTING: Locate AprilTag 3 in world frame')
        self.get_logger().info('  3. PLANNING: Generate motion plan to target')
        self.get_logger().info('  4. EXECUTING: Execute grasp sequence')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Auto-start: {self.auto_start}')
        self.get_logger().info('Services: /start_pipeline, /reset_pipeline')
        self.get_logger().info('='*60)
        
        if self.auto_start:
            self.state = PipelineState.CALIBRATING
            self.get_logger().info('Auto-start enabled - beginning calibration...')
    
    def _init_camera(self):
        """Initialize RealSense camera."""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
        self.camera_matrix = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ])
        
        self.get_logger().info(f'Camera initialized: fx={intr.fx:.1f}, fy={intr.fy:.1f}')
    
    def _init_detector(self):
        """Initialize AprilTag detector."""
        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25
        )
        self.get_logger().info('AprilTag detector initialized (tag36h11)')
    
    def try_init_robot(self):
        """Try to initialize robot arm."""
        if self.robot_initialized:
            self.robot_init_timer.cancel()
            return
        
        try:
            from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
            import sys, os
            sys.path.insert(0, os.path.dirname(__file__))
            from moveit2_interface import MoveIt2Interface
            
            self.robot = InterbotixManipulatorXS(
                robot_model=self.robot_model,
                robot_name=self.robot_name,
                moving_time=2.0,
                accel_time=0.5,
                gripper_pressure=0.5,
                node=self
            )
            self.robot_initialized = True
            self.get_logger().info('✓ Robot arm initialized!')
            
            # Initialize MoveIt2 interface
            try:
                self.get_logger().info('Initializing MoveIt2 interface...')
                self.moveit_interface = MoveIt2Interface(
                    node=self,
                    group_name=self.planning_group,
                    base_link=self.base_frame,
                    ee_link=self.ee_frame
                )
                self.moveit_initialized = True
                self.get_logger().info('✓ MoveIt2 interface initialized!')
            except Exception as me:
                self.get_logger().warn(f'MoveIt2 init failed: {me}')
                self.get_logger().warn('Will use Interbotix API only')
                self.moveit_initialized = False
            
            self.robot.arm.go_to_home_pose()
            self.robot.gripper.release()
            self.robot_init_timer.cancel()
        except Exception as e:
            self.get_logger().debug(f'Robot init pending: {e}')
    
    def detect_tags(self):
        """Detect all AprilTags in current frame."""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            
            if not color_frame:
                return {}, None
            
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size
            )
            
            tags = {}
            for det in detections:
                if self._validate_pose(det.pose_R, det.pose_t):
                    tags[det.tag_id] = {
                        'R': det.pose_R,
                        't': det.pose_t.flatten(),
                        'center': det.center
                    }
            
            return tags, frame
        except Exception as e:
            self.get_logger().debug(f'Detection error: {e}')
            return {}, None
    
    def _validate_pose(self, R, t):
        """Validate pose detection."""
        if R is None or t is None:
            return False
        if abs(np.linalg.det(R) - 1.0) > 0.1:
            return False
        if np.any(np.isnan(t)) or np.any(np.isinf(t)):
            return False
        dist = np.linalg.norm(t)
        return 0.05 < dist < 5.0
    
    def calibrate_world_frame(self, detected_tags):
        """
        Step 1: Calibrate world frame using tags 0, 1, 2.
        Returns camera pose in world frame.
        """
        if 0 not in detected_tags:
            return None
        
        # Get origin tag (tag 0) pose: camera -> tag0
        origin = detected_tags[0]
        T_cam_to_tag0 = np.eye(4)
        T_cam_to_tag0[:3, :3] = origin['R']
        T_cam_to_tag0[:3, 3] = origin['t']
        
        # Tag0 is at world origin, so: world -> camera = inv(camera -> tag0)
        T_world_to_camera = np.linalg.inv(T_cam_to_tag0)
        
        # Refine with additional tags if available
        if len(detected_tags) > 1:
            T_world_to_camera = self._refine_with_multiple_tags(
                detected_tags, T_world_to_camera)
        
        return T_world_to_camera
    
    def _refine_with_multiple_tags(self, detected_tags, initial_T):
        """Refine camera pose using multiple calibration tags."""
        transforms = [initial_T]
        
        for tag_id, tag_data in detected_tags.items():
            if tag_id == 0 or tag_id not in self.calibration_tags:
                continue
            
            # Tag pose in camera frame
            T_cam_to_tag = np.eye(4)
            T_cam_to_tag[:3, :3] = tag_data['R']
            T_cam_to_tag[:3, 3] = tag_data['t']
            
            # Known tag position in world frame
            world_pos = self.calibration_tags[tag_id]
            T_world_to_tag = np.eye(4)
            T_world_to_tag[:3, 3] = world_pos
            
            # Compute camera pose: world -> camera = world -> tag @ inv(camera -> tag)
            T_world_to_camera = T_world_to_tag @ np.linalg.inv(T_cam_to_tag)
            transforms.append(T_world_to_camera)
        
        # Average transforms
        avg_t = np.mean([T[:3, 3] for T in transforms], axis=0)
        avg_R = np.mean([T[:3, :3] for T in transforms], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        result = np.eye(4)
        result[:3, :3] = avg_R
        result[:3, 3] = avg_t
        
        return result
    
    def compute_target_world_pose(self, detected_tags):
        """
        Step 2: Compute target (tag 3) pose in world frame.
        """
        if self.target_tag_id not in detected_tags:
            return None
        
        if self.T_world_to_camera is None:
            return None
        
        target = detected_tags[self.target_tag_id]
        T_cam_to_target = np.eye(4)
        T_cam_to_target[:3, :3] = target['R']
        T_cam_to_target[:3, 3] = target['t']
        
        # Transform to world frame
        T_world_to_target = self.T_world_to_camera @ T_cam_to_target
        
        return T_world_to_target
    
    def save_calibration_data(self):
        """Save calibration data to JSON file."""
        if self.T_world_to_camera is None:
            return
        
        data = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'camera_pose': {
                'translation': self.T_world_to_camera[:3, 3].tolist(),
                'rotation_matrix': self.T_world_to_camera[:3, :3].tolist(),
            },
            'calibration_tags': {
                str(k): v.tolist() for k, v in self.calibration_tags.items()
            },
            'camera_intrinsics': {
                'fx': self.camera_params[0],
                'fy': self.camera_params[1],
                'cx': self.camera_params[2],
                'cy': self.camera_params[3],
            }
        }
        
        filepath = Path(self.calibration_file)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info(f'✓ Calibration saved to: {filepath.absolute()}')
    
    def plan_grasp_motion(self):
        """
        Step 3: Plan motion to target using MoveIt2 or robot IK.
        Returns True if planning succeeded.
        """
        if not self.robot_initialized or self.target_pose_world is None:
            return False
        
        T = self.target_pose_world
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        if self.moveit_initialized:
            self.get_logger().info('Planning grasp motion with MoveIt2...')
            
            # Create approach pose
            approach_pose = self.moveit_interface.create_pose(
                x, y, z + self.approach_height,
                pitch=np.pi/2  # Gripper pointing down
            )
            
            # Test planning to approach position
            trajectory = self.moveit_interface.plan_to_pose_goal(approach_pose)
            
            if trajectory:
                self.get_logger().info('✓ MoveIt2 planning succeeded')
                return True
            else:
                self.get_logger().warn('MoveIt2 planning failed, will use direct IK')
                return True  # Still allow execution with Interbotix fallback
        else:
            self.get_logger().info('Planning grasp motion with Interbotix IK...')
            # Planning is implicit in Interbotix API - IK happens in set_ee_pose_components
            return True
    
    def execute_grasp_sequence(self):
        """
        Step 4: Execute complete grasp sequence.
        """
        if not self.robot_initialized or self.target_pose_world is None:
            return False
        
        T = self.target_pose_world
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        self.get_logger().info('='*60)
        self.get_logger().info('EXECUTING GRASP SEQUENCE')
        self.get_logger().info(f'Target: ({x:.3f}, {y:.3f}, {z:.3f})')
        self.get_logger().info('='*60)
        
        try:
            # Step 1: Open gripper
            self.get_logger().info('[1/6] Opening gripper...')
            self.robot.gripper.release()
            time.sleep(0.5)
            
            # Step 2: Move to approach position (above target)
            approach_z = z + self.approach_height
            self.get_logger().info(f'[2/6] Moving to approach (z={approach_z:.3f})...')
            
            success, _ = self.robot.arm.set_ee_pose_components(
                x=x, y=y, z=approach_z, pitch=np.pi/2, moving_time=2.0
            )
            
            if not success:
                self.get_logger().warn('Approach position unreachable, trying alternative...')
                # Try with slight offset
                success, _ = self.robot.arm.set_ee_pose_components(
                    x=x*0.95, y=y*0.95, z=approach_z, pitch=np.pi/2, moving_time=2.0
                )
                if not success:
                    self.get_logger().error('Cannot reach approach position')
                    return False
            
            time.sleep(0.5)
            
            # Step 3: Descend to grasp position
            grasp_z = z + self.grasp_height
            descent_distance = approach_z - grasp_z
            self.get_logger().info(f'[3/6] Descending {descent_distance:.3f}m to grasp height...')
            
            success = self.robot.arm.set_ee_cartesian_trajectory(
                z=-descent_distance, moving_time=1.5
            )
            
            if not success:
                self.get_logger().warn('Cartesian descent failed, using IK...')
                success, _ = self.robot.arm.set_ee_pose_components(
                    x=x, y=y, z=grasp_z, pitch=np.pi/2, moving_time=1.5
                )
            
            time.sleep(0.3)
            
            # Step 4: Close gripper
            self.get_logger().info('[4/6] Closing gripper...')
            self.robot.gripper.grasp()
            time.sleep(1.0)
            
            # Step 5: Lift up
            self.get_logger().info(f'[5/6] Lifting {self.retreat_height:.3f}m...')
            self.robot.arm.set_ee_cartesian_trajectory(
                z=self.retreat_height, moving_time=1.5
            )
            time.sleep(0.5)
            
            # Step 6: Return to home
            self.get_logger().info('[6/6] Returning to home position...')
            self.robot.arm.go_to_home_pose(moving_time=2.0)
            time.sleep(0.5)
            
            self.get_logger().info('='*60)
            self.get_logger().info('✓ GRASP SEQUENCE COMPLETED!')
            self.get_logger().info('='*60)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Grasp execution failed: {e}')
            return False
    
    def process_state(self):
        """Main state machine processing."""
        try:
            # Publish current state
            state_msg = String()
            state_msg.data = self.state.name
            self.state_pub.publish(state_msg)
            
            # Detect tags in current frame
            detected_tags, frame = self.detect_tags()
            
            # State machine
            if self.state == PipelineState.INIT:
                # Waiting for start command or auto-start
                pass
            
            elif self.state == PipelineState.CALIBRATING:
                # Looking for calibration tags 0, 1, 2
                cal_tags = {k: v for k, v in detected_tags.items() if k in self.calibration_tags}
                
                if 0 in cal_tags:
                    T = self.calibrate_world_frame(cal_tags)
                    if T is not None:
                        self.T_world_to_camera = self.camera_pose_filter.update(T)
                        self._publish_camera_tf()
                        
                        # Transition to CALIBRATED after stable detection
                        if self.camera_pose_filter.is_stable():
                            self.state = PipelineState.CALIBRATED
                            if self.save_calibration:
                                self.save_calibration_data()
                            self.get_logger().info('✓ Step 1 Complete: World frame calibrated')
                            self.get_logger().info(f'  Camera at: {self.T_world_to_camera[:3, 3]}')
                            self.state = PipelineState.DETECTING
                
                # Publish calibration status
                cal_msg = Bool()
                cal_msg.data = (self.T_world_to_camera is not None)
                self.calibration_pub.publish(cal_msg)
            
            elif self.state == PipelineState.DETECTING:
                # Looking for target tag 3
                if self.T_world_to_camera is not None and self.target_tag_id in detected_tags:
                    T_target = self.compute_target_world_pose(detected_tags)
                    if T_target is not None:
                        self.target_pose_world = self.target_pose_filter.update(T_target)
                        self._publish_target_tf()
                        self._publish_target_pose()
                        self._publish_grasp_pose()
                        
                        # Transition to TARGET_DETECTED after stable detection
                        if self.target_pose_filter.is_stable():
                            self.state = PipelineState.TARGET_DETECTED
                            self.get_logger().info('✓ Step 2 Complete: Target detected')
                            self.get_logger().info(f'  Target at: {self.target_pose_world[:3, 3]}')
                            self.state = PipelineState.PLANNING
            
            elif self.state == PipelineState.PLANNING:
                # Plan motion to target
                if self.plan_grasp_motion():
                    self.state = PipelineState.PLANNED
                    self.get_logger().info('✓ Step 3 Complete: Motion planned')
                    self.state = PipelineState.EXECUTING
                else:
                    self.get_logger().warn('Planning failed, retrying...')
                    time.sleep(1.0)
            
            elif self.state == PipelineState.EXECUTING:
                # Execute grasp
                if self.execute_grasp_sequence():
                    self.state = PipelineState.DONE
                    self.get_logger().info('✓ Step 4 Complete: Grasp executed')
                    self.get_logger().info('')
                    self.get_logger().info('='*60)
                    self.get_logger().info('✓✓✓ PIPELINE COMPLETE ✓✓✓')
                    self.get_logger().info('='*60)
                else:
                    self.state = PipelineState.ERROR
                    self.get_logger().error('Execution failed')
            
            elif self.state == PipelineState.DONE:
                # Pipeline complete, do nothing
                pass
            
            elif self.state == PipelineState.ERROR:
                # Error state, wait for reset
                pass
        
        except Exception as e:
            self.get_logger().error(f'State machine error: {e}')
            self.state = PipelineState.ERROR
    
    def _publish_camera_tf(self):
        """Publish camera TF transform."""
        if self.T_world_to_camera is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.camera_frame
        
        T = self.T_world_to_camera
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        
        rot = Rotation.from_matrix(T[:3, :3])
        q = rot.as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_target_tf(self):
        """Publish target TF transform."""
        if self.target_pose_world is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = 'target'
        
        T = self.target_pose_world
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        
        rot = Rotation.from_matrix(T[:3, :3])
        q = rot.as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_target_pose(self):
        """Publish target pose message."""
        if self.target_pose_world is None:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        
        T = self.target_pose_world
        msg.pose.position.x = float(T[0, 3])
        msg.pose.position.y = float(T[1, 3])
        msg.pose.position.z = float(T[2, 3])
        
        rot = Rotation.from_matrix(T[:3, :3])
        q = rot.as_quat()
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        
        self.target_pose_pub.publish(msg)
    
    def _publish_grasp_pose(self):
        """Publish computed grasp pose."""
        if self.target_pose_world is None:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        
        T = self.target_pose_world
        # Grasp position: slightly above target
        msg.pose.position.x = float(T[0, 3])
        msg.pose.position.y = float(T[1, 3])
        msg.pose.position.z = float(T[2, 3] + self.grasp_height)
        
        # Gripper orientation: pointing down
        grasp_rot = Rotation.from_euler('xyz', [0, np.pi, 0])
        q = grasp_rot.as_quat()
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        
        self.grasp_pose_pub.publish(msg)
    
    def start_pipeline_callback(self, request, response):
        """Service callback to start the pipeline."""
        if self.state in [PipelineState.INIT, PipelineState.ERROR, PipelineState.DONE]:
            self.state = PipelineState.CALIBRATING
            self.get_logger().info('Pipeline started - beginning calibration...')
            response.success = True
            response.message = 'Pipeline started'
        else:
            response.success = False
            response.message = f'Pipeline already running (state: {self.state.name})'
        
        return response
    
    def reset_pipeline_callback(self, request, response):
        """Service callback to reset the pipeline."""
        self.state = PipelineState.INIT
        self.T_world_to_camera = None
        self.target_pose_world = None
        self.camera_pose_filter.reset()
        self.target_pose_filter.reset()
        
        if self.robot_initialized:
            try:
                self.robot.gripper.release()
                self.robot.arm.go_to_home_pose()
            except:
                pass
        
        self.get_logger().info('Pipeline reset to INIT state')
        response.success = True
        response.message = 'Pipeline reset'
        
        return response
    
    def destroy_node(self):
        """Clean up resources."""
        try:
            self.pipeline.stop()
        except:
            pass
        
        if self.robot_initialized:
            try:
                self.robot.gripper.release()
                self.robot.arm.go_to_sleep_pose()
            except:
                pass
        
        super().destroy_node()


class TemporalFilter:
    """Moving average filter for pose smoothing with stability detection."""
    
    def __init__(self, window_size=5, stability_threshold=0.01):
        self.window_size = window_size
        self.stability_threshold = stability_threshold
        self.buffer = []
        self.stable_count = 0
        self.required_stable = 3
    
    def update(self, T):
        """Add new transform and return filtered result."""
        self.buffer.append(T.copy())
        
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        
        # Average translations
        avg_t = np.mean([m[:3, 3] for m in self.buffer], axis=0)
        
        # Average rotations (simplified - use SVD to project back to SO(3))
        avg_R = np.mean([m[:3, :3] for m in self.buffer], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        result = np.eye(4)
        result[:3, :3] = avg_R
        result[:3, 3] = avg_t
        
        # Check stability
        if len(self.buffer) >= 2:
            variance = np.std([m[:3, 3] for m in self.buffer], axis=0)
            if np.max(variance) < self.stability_threshold:
                self.stable_count += 1
            else:
                self.stable_count = 0
        
        return result
    
    def is_stable(self):
        """Check if measurements are stable."""
        return self.stable_count >= self.required_stable
    
    def reset(self):
        """Reset filter state."""
        self.buffer = []
        self.stable_count = 0


def main(args=None):
    rclpy.init(args=args)
    
    node = AprilTagPipeline()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
