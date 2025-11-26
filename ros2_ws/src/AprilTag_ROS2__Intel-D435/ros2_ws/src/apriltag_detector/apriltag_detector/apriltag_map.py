#!/usr/bin/env python3
"""
AprilTag Map Frame Detection Node

This node detects AprilTags using Intel RealSense D435 camera and publishes
their positions in a unified map coordinate frame.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation
import time
import traceback
import json
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ====== Configuration ======
TAG_SIZE = 0.0625  # meters (6.25 cm)
FPS = 30
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
TARGET_TAG_IDS = [0, 1, 2]  # Three target tags
MAP_ORIGIN_TAG_ID = 0  # Tag 0 as Map coordinate origin


class MapFramePublisher(Node):
    """Publishes the map->camera transform via TF2."""

    def __init__(self, node_name='map_frame_publisher', map_frame_id='map', child_frame_id='camera_link'):
        super().__init__(node_name)
        self.map_frame_id = map_frame_id
        self.child_frame_id = child_frame_id
        self.broadcaster = TransformBroadcaster(self)

    def publish_map_frame(self, rotation: np.ndarray, translation: np.ndarray):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame_id
        msg.child_frame_id = self.child_frame_id
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])

        quat = Rotation.from_matrix(rotation).as_quat()
        msg.transform.rotation.x = float(quat[0])
        msg.transform.rotation.y = float(quat[1])
        msg.transform.rotation.z = float(quat[2])
        msg.transform.rotation.w = float(quat[3])

        self.broadcaster.sendTransform(msg)


class AprilTagDetector:
    """Detects AprilTags and manages map frame transformations."""
    
    def __init__(self, tag_size=TAG_SIZE, fps=FPS, frame_width=FRAME_WIDTH, 
                 frame_height=FRAME_HEIGHT, map_origin_id=MAP_ORIGIN_TAG_ID,
                 tf_publisher: Optional[MapFramePublisher] = None):     # Added tf_publisher parameter
        """Initialize AprilTag detector and camera."""
        self.tag_size = tag_size
        self.fps = fps
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.map_origin_id = map_origin_id
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, frame_width, frame_height, 
                                  rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, frame_width, frame_height, 
                                  rs.format.z16, fps)
        
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        
        # Get camera intrinsics
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
        self.camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                                       [0, intr.fy, intr.ppy],
                                       [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))
        
        print(f"📷 Camera Intrinsics:\n   fx: {intr.fx:.2f}, fy: {intr.fy:.2f}, "
              f"cx: {intr.ppx:.2f}, cy: {intr.ppy:.2f}")
        
        # Initialize AprilTag detector
        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25
        )
        print("✅ AprilTag detector initialized!")
        print(f"🗺️  Map Origin: Tag {map_origin_id}")
        
        # Temporal filter for stability
        self.pose_filter = TemporalFilter(window_size=5)
        self.tag_data_storage = {}
        self.tf_publisher = tf_publisher                        # Added for TF2 publishing
    
    def detect_and_process(self):
        """Detect AprilTags in current frame and return processed data."""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return None, None
        
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Image preprocessing
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # AprilTag detection
        try:
            detections = self.detector.detect(gray, estimate_tag_pose=True,
                                             camera_params=self.camera_params, 
                                             tag_size=self.tag_size)
        except Exception as e:
            print(f"⚠️ Detection error: {e}")
            detections = []
        
        current_frame_tags = {}
        map_origin_data = None
        
        # Process detections
        for det in detections:
            if det.tag_id not in TARGET_TAG_IDS:
                continue
            
            if not self._validate_pose(det.pose_R, det.pose_t):
                continue
            
            # Apply temporal filtering
            filtered_R, filtered_t = self.pose_filter.update(det.tag_id, 
                                                            det.pose_R, det.pose_t)
            
            # Draw visualization
            corners = det.corners.astype(int)
            color = (0, 255, 0) if det.tag_id == self.map_origin_id else (255, 0, 0)
            cv2.polylines(frame, [corners.reshape((-1, 1, 2))], True, color, 2)
            
            center = tuple(det.center.astype(int))
            cv2.putText(frame, f"ID:{det.tag_id}", (center[0]-20, center[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Get depth
            depth = depth_frame.get_distance(int(center[0]), int(center[1]))
            
            # Draw coordinate axes
            rvec, _ = cv2.Rodrigues(filtered_R)
            frame = self._draw_pose_axes(frame, rvec, filtered_t)
            
            # Store tag data
            current_frame_tags[det.tag_id] = {
                'pose_R': filtered_R,
                'pose_t': filtered_t,
                'depth': depth,
                'center': center
            }
            
            if det.tag_id == self.map_origin_id:
                map_origin_data = current_frame_tags[det.tag_id]
        
        # Transform to map frame if origin found
        if map_origin_data is not None:
            map_origin_R = map_origin_data['pose_R']
            map_origin_t = map_origin_data['pose_t']
            
            for tag_id, data in current_frame_tags.items():
                if tag_id == self.map_origin_id:
                    data['map_position'] = np.array([0.0, 0.0, 0.0])
                    data['map_rotation'] = np.eye(3)
                else:
                    R_map, t_map = self._transform_to_map_frame(
                        data['pose_R'], data['pose_t'],
                        map_origin_R, map_origin_t
                    )
                    data['map_position'] = t_map.flatten()
                    data['map_rotation'] = R_map
                
                self.tag_data_storage[tag_id] = data
            
            # Draw info overlay
            frame = self._draw_map_info(frame, self.tag_data_storage)
            self._publish_map_frame(map_origin_R, map_origin_t)                 # Publish TF2 map frame
        else:
            cv2.putText(frame, f"Searching for Map Origin (Tag {self.map_origin_id})...", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        
        status_text = f"Detected: {len(current_frame_tags)}/{len(TARGET_TAG_IDS)} tags | FPS: {self.fps}"
        cv2.putText(frame, status_text, (10, self.frame_height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame, current_frame_tags
    
    def _validate_pose(self, pose_R, pose_t):
        """Validate detected pose reasonableness."""
        if pose_R is None or pose_t is None:
            return False
        
        det = np.linalg.det(pose_R)
        if abs(det - 1.0) > 0.1:
            return False
        
        if np.any(np.isnan(pose_t)) or np.any(np.isinf(pose_t)):
            return False
        
        distance = np.linalg.norm(pose_t)
        if distance < 0.05 or distance > 5.0:
            return False
        
        return True
    
    def _draw_pose_axes(self, frame, rvec, tvec, length=0.05):
        """Draw 3D coordinate axes on frame."""
        axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, 
                                     self.camera_matrix, self.dist_coeffs)
        imgpts = imgpts.astype(int)
        origin = tuple(imgpts[0].ravel())
        frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)
        frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)
        frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3)
        return frame
    
    def _transform_to_map_frame(self, tag_pose_R, tag_pose_t, map_origin_R, map_origin_t):
        """Transform tag pose from camera frame to map frame."""
        T_cam_to_map_origin = np.eye(4)
        T_cam_to_map_origin[:3, :3] = map_origin_R
        T_cam_to_map_origin[:3, 3] = map_origin_t.flatten()
        
        T_map_origin_to_cam = np.linalg.inv(T_cam_to_map_origin)
        
        T_cam_to_tag = np.eye(4)
        T_cam_to_tag[:3, :3] = tag_pose_R
        T_cam_to_tag[:3, 3] = tag_pose_t.flatten()
        
        T_map_origin_to_tag = T_map_origin_to_cam @ T_cam_to_tag
        
        R_map = T_map_origin_to_tag[:3, :3]
        t_map = T_map_origin_to_tag[:3, 3].reshape(3, 1)
        
        return R_map, t_map
    
    def _draw_map_info(self, frame, tag_data):
        """Draw map information overlay on frame."""
        y_offset = 30
        title = f"=== Map Frame (Origin: Tag {self.map_origin_id}) ==="
        cv2.putText(frame, title, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(frame, title, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y_offset += 30
        
        for tag_id in sorted(tag_data.keys()):
            data = tag_data[tag_id]
            
            if tag_id == self.map_origin_id:
                text = f"Tag {tag_id}: [ORIGIN] X=0.000 Y=0.000 Z=0.000 m"
                color = (0, 255, 0)
            else:
                pos = data['map_position']
                text = f"Tag {tag_id}: X={pos[0]:6.3f} Y={pos[1]:6.3f} Z={pos[2]:6.3f} m"
                color = (255, 255, 255)
            
            cv2.putText(frame, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(frame, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_offset += 25
        
        return frame

    def _publish_map_frame(self, map_origin_R, map_origin_t):
        """Send the map -> camera transform over TF2."""
        if self.tf_publisher is None:
            return

        T_cam_to_map_origin = np.eye(4)
        T_cam_to_map_origin[:3, :3] = map_origin_R
        T_cam_to_map_origin[:3, 3] = map_origin_t.flatten()

        T_map_origin_to_cam = np.linalg.inv(T_cam_to_map_origin)
        rotation = T_map_origin_to_cam[:3, :3]
        translation = T_map_origin_to_cam[:3, 3]

        self.tf_publisher.publish_map_frame(rotation, translation)
    
    def save_map_data(self, output_file='apriltag_map.json'):
        """Save map data to JSON file."""
        if not self.tag_data_storage:
            print("⚠️ No tag data to save")
            return
        
        map_data = {
            'metadata': {
                'map_origin_tag_id': self.map_origin_id,
                'tag_size': self.tag_size,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            },
            'tags': {}
        }
        
        for tag_id in sorted(self.tag_data_storage.keys()):
            data = self.tag_data_storage[tag_id]
            pos = data['map_position']
            rot = data['map_rotation']
            euler = Rotation.from_matrix(rot).as_euler('xyz', degrees=True)
            
            map_data['tags'][str(tag_id)] = {
                'position': pos.tolist(),
                'rotation_matrix': rot.tolist(),
                'rotation_euler_deg': euler.tolist(),
                'is_origin': (tag_id == self.map_origin_id)
            }
        
        try:
            with open(output_file, 'w') as f:
                json.dump(map_data, f, indent=2)
            print(f"💾 Map data saved to: {output_file}")
        except Exception as e:
            print(f"⚠️ Failed to save map data: {e}")
    
    def cleanup(self):
        """Clean up resources."""
        self.pipeline.stop()
        cv2.destroyAllWindows()


class TemporalFilter:
    """Simple moving average filter for pose smoothing."""
    
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = {}
    
    def update(self, tag_id, pose_R, pose_t):
        """Add new pose and return filtered pose."""
        if tag_id not in self.buffer:
            self.buffer[tag_id] = {'R': [], 't': []}
        
        self.buffer[tag_id]['R'].append(pose_R)
        self.buffer[tag_id]['t'].append(pose_t)
        
        if len(self.buffer[tag_id]['R']) > self.window_size:
            self.buffer[tag_id]['R'].pop(0)
            self.buffer[tag_id]['t'].pop(0)
        
        avg_t = np.mean(self.buffer[tag_id]['t'], axis=0)
        avg_R = np.mean(self.buffer[tag_id]['R'], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        return avg_R, avg_t


def main():
    rclpy.init()
    tf_node = MapFramePublisher()
    detector = AprilTagDetector(tf_publisher=tf_node)
    
    print("✅ Press Q to quit")
    print("="*70)

    try:
        frame_count = 0
        last_print_time = time.time()
        
        while True:
            frame, current_frame_tags = detector.detect_and_process()
            
            if frame is None:
                continue
            
            # Periodic terminal output
            if detector.tag_data_storage:
                current_time = time.time()
                if current_time - last_print_time >= 2.0:
                    print(f"\n{'='*60}")
                    print(f"🗺️  Map Frame Status (Origin: Tag {detector.map_origin_id})")
                    print(f"{'='*60}")
                    for tag_id in sorted(detector.tag_data_storage.keys()):
                        data = detector.tag_data_storage[tag_id]
                        pos = data['map_position']
                        euler = Rotation.from_matrix(data['map_rotation']).as_euler('xyz', degrees=True)
                        
                        if tag_id == detector.map_origin_id:
                            print(f"Tag {tag_id} [ORIGIN]: (0.000, 0.000, 0.000) m")
                        else:
                            print(f"Tag {tag_id}: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) m")
                            print(f"         Rotation: R={euler[0]:6.1f}° P={euler[1]:6.1f}° Y={euler[2]:6.1f}°")
                    
                    last_print_time = current_time
            
            cv2.imshow("AprilTag Map Frame - Press Q to quit", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            frame_count += 1

    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user (Ctrl+C)")
    
    except Exception as e:
        print("\n❌ Error occurred:")
        traceback.print_exc()
    
    finally:
        detector.cleanup()
        detector.save_map_data()
        tf_node.destroy_node()
        rclpy.shutdown()
        print("\n🛑 Program terminated")


if __name__ == '__main__':
    main()
