#!/usr/bin/env python3
"""
Target Detector Node

This node detects AprilTag (id:3) and publishes its position in the world frame.
It subscribes to the calibration status and camera pose, then transforms the
detected tag position to the world frame.

Published Topics:
  - /target_pose (geometry_msgs/PoseStamped): Target position in world frame
  - /target_detected (std_msgs/Bool): Whether target is currently detected

TF Transforms published:
  - world -> target (position of AprilTag id:3)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import pyrealsense2 as rs
from dt_apriltags import Detector

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class TargetDetectorNode(Node):
    """
    ROS2 Node for detecting target AprilTag and publishing its world position.
    """
    
    def __init__(self):
        super().__init__('target_detector')
        
        # Declare parameters
        self.declare_parameter('tag_size', 0.0625)  # 6.25 cm
        self.declare_parameter('target_tag_id', 3)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('target_frame', 'target')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('grasp_offset_z', 0.05)  # Offset above target for grasping
        
        # Get parameters
        self.tag_size = self.get_parameter('tag_size').value
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.grasp_offset_z = self.get_parameter('grasp_offset_z').value
        
        # Initialize RealSense camera
        self._init_realsense()
        
        # Initialize AprilTag detector
        self._init_detector()
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped, 'target_pose', 10)
        self.grasp_pose_pub = self.create_publisher(
            PoseStamped, 'grasp_pose', 10)
        self.target_detected_pub = self.create_publisher(
            Bool, 'target_detected', 10)
        self.marker_pub = self.create_publisher(
            Marker, 'target_marker', 10)
        
        # Subscribers
        self.calibration_status_sub = self.create_subscription(
            Bool, 'calibration_status', self.calibration_status_callback, 10)
        
        # State
        self.is_calibrated = False
        self.target_detected = False
        self.target_pose_world = None
        self.temporal_filter = TemporalFilter(window_size=5)
        
        # Create timer for main loop
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Target Detector Node initialized')
        self.get_logger().info(f'Looking for target tag ID: {self.target_tag_id}')
    
    def _init_realsense(self):
        """Initialize RealSense camera."""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.frame_rate)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.frame_rate)
        
        try:
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
            
            self.get_logger().info('RealSense camera initialized')
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
    
    def calibration_status_callback(self, msg):
        """Handle calibration status updates."""
        self.is_calibrated = msg.data
    
    def detect_target(self):
        """Detect target AprilTag in current frame."""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            return None
        
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
        
        for det in detections:
            if det.tag_id == self.target_tag_id:
                if self._validate_pose(det.pose_R, det.pose_t):
                    return {
                        'R': det.pose_R,
                        't': det.pose_t,
                        'center': det.center
                    }
        
        return None
    
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
    
    def transform_to_world(self, tag_data):
        """Transform tag pose from camera frame to world frame."""
        try:
            # Get camera -> world transform from TF
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Build world -> camera transform matrix
            t = transform.transform.translation
            r = transform.transform.rotation
            
            T_world_to_camera = np.eye(4)
            rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
            T_world_to_camera[:3, :3] = rot.as_matrix()
            T_world_to_camera[:3, 3] = [t.x, t.y, t.z]
            
            # Build camera -> tag transform matrix
            T_camera_to_tag = np.eye(4)
            T_camera_to_tag[:3, :3] = tag_data['R']
            T_camera_to_tag[:3, 3] = tag_data['t'].flatten()
            
            # Compute world -> tag
            T_world_to_tag = T_world_to_camera @ T_camera_to_tag
            
            return T_world_to_tag
            
        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')
            return None
    
    def timer_callback(self):
        """Main processing loop."""
        try:
            # Detect target tag
            target_data = self.detect_target()
            
            if target_data is None:
                self.target_detected = False
                self._publish_detection_status(False)
                return
            
            # Check if world frame calibration is available
            if not self.is_calibrated:
                self.get_logger().warn('World frame not calibrated yet', throttle_duration_sec=2.0)
                return
            
            # Transform to world frame
            T_world_to_target = self.transform_to_world(target_data)
            
            if T_world_to_target is None:
                return
            
            # Apply temporal filtering
            T_world_to_target = self.temporal_filter.update(T_world_to_target)
            self.target_pose_world = T_world_to_target
            self.target_detected = True
            
            # Publish target TF
            self._publish_target_tf(T_world_to_target)
            
            # Publish target pose
            self._publish_target_pose(T_world_to_target)
            
            # Publish grasp pose (slightly above target)
            self._publish_grasp_pose(T_world_to_target)
            
            # Publish visualization marker
            self._publish_marker(T_world_to_target)
            
            # Publish detection status
            self._publish_detection_status(True)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def _publish_target_tf(self, T):
        """Publish target transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.target_frame
        
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        
        rot = Rotation.from_matrix(T[:3, :3])
        quat = rot.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_target_pose(self, T):
        """Publish target pose as PoseStamped."""
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
        
        self.target_pose_pub.publish(pose_msg)
        
        self.get_logger().info(
            f'Target at world: ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})',
            throttle_duration_sec=2.0
        )
    
    def _publish_grasp_pose(self, T):
        """Publish grasp pose (slightly above target with downward orientation)."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.world_frame
        
        # Position: above target
        pose_msg.pose.position.x = T[0, 3]
        pose_msg.pose.position.y = T[1, 3]
        pose_msg.pose.position.z = T[2, 3] + self.grasp_offset_z
        
        # Orientation: gripper pointing down (Z-axis pointing down)
        # For a top-down grasp, we want the gripper to point down
        # Roll = 0, Pitch = 180 deg (π), Yaw = 0
        grasp_rot = Rotation.from_euler('xyz', [0, np.pi, 0])
        quat = grasp_rot.as_quat()
        
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.grasp_pose_pub.publish(pose_msg)
    
    def _publish_marker(self, T):
        """Publish visualization marker."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = T[0, 3]
        marker.pose.position.y = T[1, 3]
        marker.pose.position.z = T[2, 3]
        
        rot = Rotation.from_matrix(T[:3, :3])
        quat = rot.as_quat()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Tag size
        marker.scale.x = self.tag_size
        marker.scale.y = self.tag_size
        marker.scale.z = 0.01
        
        # Green color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.5e9)
        
        self.marker_pub.publish(marker)
    
    def _publish_detection_status(self, detected):
        """Publish detection status."""
        msg = Bool()
        msg.data = detected
        self.target_detected_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up resources."""
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()


class TemporalFilter:
    """Simple moving average filter for pose smoothing."""
    
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = []
    
    def update(self, T):
        """Add new transform and return filtered result."""
        self.buffer.append(T.copy())
        
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        
        avg_t = np.mean([m[:3, 3] for m in self.buffer], axis=0)
        avg_R = np.mean([m[:3, :3] for m in self.buffer], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        result = np.eye(4)
        result[:3, :3] = avg_R
        result[:3, 3] = avg_t
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
