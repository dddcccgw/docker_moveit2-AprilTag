#!/usr/bin/env python3
"""
AprilTag Grasp Demo

This is the main demo script that orchestrates the complete workflow:
1. Calibrate world frame using AprilTags 0, 1, 2
2. Detect target object (AprilTag 3)
3. Plan and execute grasp using ViperX-300S

This script can be run standalone or as a ROS2 node.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import pyrealsense2 as rs
from dt_apriltags import Detector
import time
import json

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros


class AprilTagGraspDemo(Node):
    """
    Main demo node that combines all functionality.
    """
    
    def __init__(self):
        super().__init__('apriltag_grasp_demo')
        
        # Parameters
        self.declare_parameter('tag_size', 0.0625)
        self.declare_parameter('robot_model', 'vx300s')
        self.declare_parameter('robot_name', 'vx300s')
        self.declare_parameter('use_moveit', False)  # Use Interbotix API by default
        self.declare_parameter('auto_execute', False)  # Auto execute when target detected
        
        self.tag_size = self.get_parameter('tag_size').value
        self.robot_model = self.get_parameter('robot_model').value
        self.robot_name = self.get_parameter('robot_name').value
        self.use_moveit = self.get_parameter('use_moveit').value
        self.auto_execute = self.get_parameter('auto_execute').value
        
        # Known tag positions in world frame
        self.calibration_tags = {
            0: np.array([0.0, 0.0, 0.0]),      # Origin
            1: np.array([0.15, 0.0, 0.0]),     # X-axis (15cm)
            2: np.array([0.0, 0.0, 0.15])      # Z-axis (15cm)
        }
        self.target_tag_id = 3
        
        # Initialize camera
        self._init_camera()
        
        # Initialize AprilTag detector
        self._init_detector()
        
        # Initialize robot (optional, can be external)
        self.robot = None
        self.robot_initialized = False
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # State
        self.world_calibrated = False
        self.T_world_to_camera = None
        self.target_pose_world = None
        self.target_detected = False
        
        # Temporal filters
        self.camera_pose_filter = TemporalFilter(window_size=10)
        self.target_pose_filter = TemporalFilter(window_size=5)
        
        # Callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.calibration_pub = self.create_publisher(Bool, 'calibration_status', 10)
        self.target_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, 'grasp_pose', 10)
        
        # Services
        self.calibrate_srv = self.create_service(
            Trigger, 'calibrate_world', self.calibrate_callback,
            callback_group=self.callback_group
        )
        self.detect_target_srv = self.create_service(
            Trigger, 'detect_target', self.detect_target_callback,
            callback_group=self.callback_group
        )
        self.execute_grasp_srv = self.create_service(
            Trigger, 'execute_demo_grasp', self.execute_grasp_callback,
            callback_group=self.callback_group
        )
        
        # Timer for continuous detection
        self.timer = self.create_timer(0.033, self.main_loop)  # ~30 Hz
        
        # Try to initialize robot
        self.init_robot_timer = self.create_timer(3.0, self.try_init_robot)
        
        self.get_logger().info('AprilTag Grasp Demo initialized')
        self.get_logger().info('Services available:')
        self.get_logger().info('  - /calibrate_world')
        self.get_logger().info('  - /detect_target')
        self.get_logger().info('  - /execute_demo_grasp')
    
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
        
        self.get_logger().info('Camera initialized')
    
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
    
    def try_init_robot(self):
        """Try to initialize robot arm."""
        if self.robot_initialized:
            self.init_robot_timer.cancel()
            return
        
        try:
            from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
            
            self.robot = InterbotixManipulatorXS(
                robot_model=self.robot_model,
                robot_name=self.robot_name,
                moving_time=2.0,
                accel_time=0.5,
                gripper_pressure=0.5,
                node=self
            )
            self.robot_initialized = True
            self.get_logger().info('Robot arm initialized!')
            self.robot.arm.go_to_home_pose()
            self.init_robot_timer.cancel()
        except Exception as e:
            self.get_logger().debug(f'Robot init pending: {e}')
    
    def detect_tags(self):
        """Detect all AprilTags in current frame."""
        frames = self.pipeline.wait_for_frames()
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
        Calibrate world frame using detected calibration tags.
        """
        if 0 not in detected_tags:
            return None
        
        # Get origin tag (tag 0) pose
        origin = detected_tags[0]
        T_cam_to_origin = np.eye(4)
        T_cam_to_origin[:3, :3] = origin['R']
        T_cam_to_origin[:3, 3] = origin['t']
        
        # World to camera = inverse of camera to origin (since origin is at world origin)
        T_world_to_camera = np.linalg.inv(T_cam_to_origin)
        
        return T_world_to_camera
    
    def compute_target_world_pose(self, detected_tags):
        """
        Compute target pose in world frame.
        """
        if self.target_tag_id not in detected_tags:
            return None
        
        if self.T_world_to_camera is None:
            return None
        
        target = detected_tags[self.target_tag_id]
        T_cam_to_target = np.eye(4)
        T_cam_to_target[:3, :3] = target['R']
        T_cam_to_target[:3, 3] = target['t']
        
        # World to target = world to camera @ camera to target
        T_world_to_target = self.T_world_to_camera @ T_cam_to_target
        
        return T_world_to_target
    
    def main_loop(self):
        """Main processing loop."""
        try:
            detected_tags, frame = self.detect_tags()
            
            if not detected_tags:
                return
            
            # Update world calibration
            if 0 in detected_tags:
                T = self.calibrate_world_frame(detected_tags)
                if T is not None:
                    self.T_world_to_camera = self.camera_pose_filter.update(T)
                    self.world_calibrated = True
                    self._publish_camera_tf()
            
            # Publish calibration status
            cal_msg = Bool()
            cal_msg.data = self.world_calibrated
            self.calibration_pub.publish(cal_msg)
            
            # Detect and track target
            if self.world_calibrated and self.target_tag_id in detected_tags:
                T_target = self.compute_target_world_pose(detected_tags)
                if T_target is not None:
                    self.target_pose_world = self.target_pose_filter.update(T_target)
                    self.target_detected = True
                    self._publish_target_tf()
                    self._publish_target_pose()
                    self._publish_grasp_pose()
            else:
                self.target_detected = False
            
        except Exception as e:
            self.get_logger().error(f'Main loop error: {e}')
    
    def _publish_camera_tf(self):
        """Publish camera TF."""
        if self.T_world_to_camera is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_link'
        
        T = self.T_world_to_camera
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        
        rot = Rotation.from_matrix(T[:3, :3])
        q = rot.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_target_tf(self):
        """Publish target TF."""
        if self.target_pose_world is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'target'
        
        T = self.target_pose_world
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        
        rot = Rotation.from_matrix(T[:3, :3])
        q = rot.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_target_pose(self):
        """Publish target pose."""
        if self.target_pose_world is None:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        T = self.target_pose_world
        msg.pose.position.x = T[0, 3]
        msg.pose.position.y = T[1, 3]
        msg.pose.position.z = T[2, 3]
        
        rot = Rotation.from_matrix(T[:3, :3])
        q = rot.as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.target_pub.publish(msg)
    
    def _publish_grasp_pose(self):
        """Publish grasp pose (above target, gripper down)."""
        if self.target_pose_world is None:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        T = self.target_pose_world
        # Grasp position: slightly above target
        msg.pose.position.x = T[0, 3]
        msg.pose.position.y = T[1, 3]
        msg.pose.position.z = T[2, 3] + 0.05  # 5cm above
        
        # Gripper orientation: pointing down
        grasp_rot = Rotation.from_euler('xyz', [0, np.pi, 0])
        q = grasp_rot.as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.grasp_pub.publish(msg)
    
    def calibrate_callback(self, request, response):
        """Service callback for manual calibration trigger."""
        if self.world_calibrated:
            response.success = True
            response.message = "World frame already calibrated"
        else:
            response.success = False
            response.message = "Calibration tags not detected"
        return response
    
    def detect_target_callback(self, request, response):
        """Service callback for target detection status."""
        if self.target_detected:
            T = self.target_pose_world
            response.success = True
            response.message = f"Target at ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})"
        else:
            response.success = False
            response.message = "Target not detected"
        return response
    
    def execute_grasp_callback(self, request, response):
        """Service callback for grasp execution."""
        if not self.robot_initialized:
            response.success = False
            response.message = "Robot not initialized"
            return response
        
        if not self.target_detected or self.target_pose_world is None:
            response.success = False
            response.message = "No target detected"
            return response
        
        try:
            success = self._execute_grasp()
            response.success = success
            response.message = "Grasp completed" if success else "Grasp failed"
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def _execute_grasp(self):
        """Execute the grasp sequence."""
        T = self.target_pose_world
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        self.get_logger().info(f'Executing grasp at ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # 1. Open gripper
        self.robot.gripper.release()
        time.sleep(0.5)
        
        # 2. Move to approach position
        approach_z = z + 0.15
        self.get_logger().info('Moving to approach position...')
        theta, success = self.robot.arm.set_ee_pose_components(
            x=x, y=y, z=approach_z, pitch=np.pi/2
        )
        if not success:
            self.get_logger().error('Failed to reach approach position')
            return False
        time.sleep(0.5)
        
        # 3. Descend to grasp
        self.get_logger().info('Descending to grasp...')
        self.robot.arm.set_ee_cartesian_trajectory(z=-0.12, moving_time=1.5)
        time.sleep(0.3)
        
        # 4. Close gripper
        self.get_logger().info('Closing gripper...')
        self.robot.gripper.grasp()
        time.sleep(1.0)
        
        # 5. Lift up
        self.get_logger().info('Lifting...')
        self.robot.arm.set_ee_cartesian_trajectory(z=0.15, moving_time=1.5)
        time.sleep(0.5)
        
        # 6. Return home
        self.get_logger().info('Returning home...')
        self.robot.arm.go_to_home_pose()
        
        self.get_logger().info('Grasp completed!')
        return True
    
    def destroy_node(self):
        """Clean up."""
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
    """Moving average filter for poses."""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = []
    
    def update(self, T):
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
    
    node = AprilTagGraspDemo()
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
