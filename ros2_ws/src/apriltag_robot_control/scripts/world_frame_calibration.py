#!/usr/bin/env python3
"""
World Frame Calibration Node

This node uses 3 AprilTags (id:0,1,2) at known positions to establish a world
coordinate frame and compute the camera's position in that frame.

AprilTag positions (known):
  - Tag 0: (0, 0, 0) - Origin
  - Tag 1: (0.15m, 0, 0) - X-axis reference
  - Tag 2: (0, 0, 0.15m) - Z-axis reference (note: in our setup Z is vertical)

The node publishes TF transforms:
  - world -> camera_link (camera position in world frame)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import pyrealsense2 as rs
from dt_apriltags import Detector

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool
import json
import time


class WorldFrameCalibrationNode(Node):
    """
    ROS2 Node for world frame calibration using AprilTags.
    """
    
    def __init__(self):
        super().__init__('world_frame_calibration')
        
        # Declare parameters
        self.declare_parameter('tag_size', 0.0625)  # 6.25 cm
        self.declare_parameter('calibration_tags', [0, 1, 2])
        self.declare_parameter('origin_tag_id', 0)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        self.tag_size = self.get_parameter('tag_size').value
        self.calibration_tags = self.get_parameter('calibration_tags').value
        self.origin_tag_id = self.get_parameter('origin_tag_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Known tag positions in world frame (meters)
        # Tag 0: Origin, Tag 1: X-axis, Tag 2: Z-axis
        self.known_tag_positions = {
            0: np.array([0.0, 0.0, 0.0]),
            1: np.array([0.15, 0.0, 0.0]),
            2: np.array([0.0, 0.0, 0.15])
        }
        
        # Initialize RealSense camera
        self._init_realsense()
        
        # Initialize AprilTag detector
        self._init_detector()
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publishers
        self.calibration_status_pub = self.create_publisher(
            Bool, 'calibration_status', 10)
        self.camera_pose_pub = self.create_publisher(
            PoseStamped, 'camera_pose_world', 10)
        
        # State
        self.is_calibrated = False
        self.T_world_to_camera = None
        self.temporal_filter = TemporalFilter(window_size=10)
        
        # Create timer for main loop
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'World Frame Calibration Node initialized')
        self.get_logger().info(f'Looking for tags: {self.calibration_tags}')
        self.get_logger().info(f'Origin tag: {self.origin_tag_id}')
    
    def _init_realsense(self):
        """Initialize RealSense camera."""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.frame_rate)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.frame_rate)
        
        try:
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            # Get camera intrinsics
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
            self.camera_matrix = np.array([
                [intr.fx, 0, intr.ppx],
                [0, intr.fy, intr.ppy],
                [0, 0, 1]
            ])
            
            self.get_logger().info(f'RealSense camera initialized')
            self.get_logger().info(f'Camera intrinsics: fx={intr.fx:.2f}, fy={intr.fy:.2f}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RealSense: {e}')
            raise
    
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
        self.get_logger().info('AprilTag detector initialized')
    
    def detect_tags(self):
        """Detect AprilTags in current frame."""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            return {}
        
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        detections = self.detector.detect(
            gray, 
            estimate_tag_pose=True,
            camera_params=self.camera_params, 
            tag_size=self.tag_size
        )
        
        detected_tags = {}
        for det in detections:
            if det.tag_id in self.calibration_tags:
                if self._validate_pose(det.pose_R, det.pose_t):
                    detected_tags[det.tag_id] = {
                        'R': det.pose_R,
                        't': det.pose_t,
                        'center': det.center
                    }
        
        return detected_tags
    
    def _validate_pose(self, R, t):
        """Validate detected pose."""
        if R is None or t is None:
            return False
        
        det = np.linalg.det(R)
        if abs(det - 1.0) > 0.1:
            return False
        
        if np.any(np.isnan(t)) or np.any(np.isinf(t)):
            return False
        
        distance = np.linalg.norm(t)
        if distance < 0.05 or distance > 5.0:
            return False
        
        return True
    
    def compute_camera_pose(self, detected_tags):
        """
        Compute camera pose in world frame using detected AprilTags.
        
        Uses the known positions of calibration tags to establish the
        world-to-camera transformation.
        """
        if self.origin_tag_id not in detected_tags:
            return None
        
        # Get origin tag pose (camera -> tag0)
        origin_data = detected_tags[self.origin_tag_id]
        R_cam_to_tag0 = origin_data['R']
        t_cam_to_tag0 = origin_data['t'].flatten()
        
        # Build transformation matrix: camera -> tag0
        T_cam_to_tag0 = np.eye(4)
        T_cam_to_tag0[:3, :3] = R_cam_to_tag0
        T_cam_to_tag0[:3, 3] = t_cam_to_tag0
        
        # Tag0 is at world origin, so T_world_to_tag0 = I
        # Therefore: T_world_to_cam = T_tag0_to_cam = inv(T_cam_to_tag0)
        T_world_to_camera = np.linalg.inv(T_cam_to_tag0)
        
        # Refine using other tags if available
        if len(detected_tags) > 1:
            T_world_to_camera = self._refine_with_multiple_tags(
                detected_tags, T_world_to_camera)
        
        return T_world_to_camera
    
    def _refine_with_multiple_tags(self, detected_tags, initial_T):
        """
        Refine camera pose using multiple detected tags.
        Uses weighted average based on detection quality.
        """
        transforms = [initial_T]
        
        for tag_id, tag_data in detected_tags.items():
            if tag_id == self.origin_tag_id:
                continue
            
            if tag_id not in self.known_tag_positions:
                continue
            
            # Get tag pose in camera frame
            R_cam_to_tag = tag_data['R']
            t_cam_to_tag = tag_data['t'].flatten()
            
            T_cam_to_tag = np.eye(4)
            T_cam_to_tag[:3, :3] = R_cam_to_tag
            T_cam_to_tag[:3, 3] = t_cam_to_tag
            
            # Known tag position in world frame
            world_pos = self.known_tag_positions[tag_id]
            
            # T_world_to_tag (tag at known position, assume same orientation as origin)
            T_world_to_tag = np.eye(4)
            T_world_to_tag[:3, 3] = world_pos
            
            # T_world_to_cam = T_world_to_tag @ inv(T_cam_to_tag)
            T_world_to_camera = T_world_to_tag @ np.linalg.inv(T_cam_to_tag)
            transforms.append(T_world_to_camera)
        
        # Average the transforms (simplified - better methods exist)
        avg_t = np.mean([T[:3, 3] for T in transforms], axis=0)
        avg_R = np.mean([T[:3, :3] for T in transforms], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        result = np.eye(4)
        result[:3, :3] = avg_R
        result[:3, 3] = avg_t
        
        return result
    
    def timer_callback(self):
        """Main processing loop."""
        try:
            detected_tags = self.detect_tags()
            
            if not detected_tags:
                # Publish calibration status
                status_msg = Bool()
                status_msg.data = False
                self.calibration_status_pub.publish(status_msg)
                return
            
            # Compute camera pose
            T_world_to_camera = self.compute_camera_pose(detected_tags)
            
            if T_world_to_camera is None:
                return
            
            # Apply temporal filtering
            T_world_to_camera = self.temporal_filter.update(T_world_to_camera)
            self.T_world_to_camera = T_world_to_camera
            self.is_calibrated = True
            
            # Publish TF: world -> camera_link
            self._publish_camera_tf(T_world_to_camera)
            
            # Publish camera pose
            self._publish_camera_pose(T_world_to_camera)
            
            # Publish calibration status
            status_msg = Bool()
            status_msg.data = True
            self.calibration_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def _publish_camera_tf(self, T):
        """Publish camera transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.camera_frame
        
        # Extract translation
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        
        # Extract rotation as quaternion
        rot = Rotation.from_matrix(T[:3, :3])
        quat = rot.as_quat()  # [x, y, z, w]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_camera_pose(self, T):
        """Publish camera pose as PoseStamped."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.world_frame
        
        pose_msg.pose.position.x = T[0, 3]
        pose_msg.pose.position.y = T[1, 3]
        pose_msg.pose.position.z = T[2, 3]
        
        rot = Rotation.from_matrix(T[:3, :3])
        quat = rot.as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.camera_pose_pub.publish(pose_msg)
    
    def destroy_node(self):
        """Clean up resources."""
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()


class TemporalFilter:
    """Simple moving average filter for pose smoothing."""
    
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.buffer = []
    
    def update(self, T):
        """Add new transform and return filtered result."""
        self.buffer.append(T.copy())
        
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        
        # Average translations
        avg_t = np.mean([m[:3, 3] for m in self.buffer], axis=0)
        
        # Average rotations (simplified)
        avg_R = np.mean([m[:3, :3] for m in self.buffer], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        result = np.eye(4)
        result[:3, :3] = avg_R
        result[:3, 3] = avg_t
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WorldFrameCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
