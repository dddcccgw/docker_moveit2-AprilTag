#!/usr/bin/env python3
"""
Record Calibration Data Node

Records camera and robot poses for hand-eye calibration.
"""

import numpy as np
import cv2
import json
from pathlib import Path
import pyrealsense2 as rs
from dt_apriltags import Detector
import traceback


TAG_SIZE = 0.0625  # meters
FPS = 30
FRAME_WIDTH, FRAME_HEIGHT = 640, 480


class CalibrationDataRecorder:
    """Records calibration data from AprilTag detections."""
    
    def __init__(self, tag_size=TAG_SIZE, fps=FPS, frame_width=FRAME_WIDTH,
                 frame_height=FRAME_HEIGHT, data_dir='data'):
        """Initialize calibration data recorder."""
        self.tag_size = tag_size
        self.fps = fps
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)
        
        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, frame_width, frame_height,
                                  rs.format.bgr8, fps)
        
        self.profile = self.pipeline.start(self.config)
        
        # Get camera intrinsics
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
        
        print(f"📷 Camera Intrinsics:\n   fx: {intr.fx:.2f}, fy: {intr.fy:.2f}, "
              f"cx: {intr.ppx:.2f}, cy: {intr.ppy:.2f}")
        
        # Initialize detector
        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25
        )
        print("✅ AprilTag detector initialized!")
        
        # Data storage
        self.camera_poses = []
        self.robot_poses = []
        self.calibration_data = []
    
    def detect_tag_pose(self, display=True):
        """Detect tag pose in current frame."""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return None, None
        
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        try:
            detections = self.detector.detect(gray, estimate_tag_pose=True,
                                             camera_params=self.camera_params,
                                             tag_size=self.tag_size)
        except Exception as e:
            print(f"⚠️ Detection error: {e}")
            detections = []
        
        tag_pose = None
        
        if detections:
            det = detections[0]
            if det.pose_R is not None and det.pose_t is not None:
                # Convert to 4x4 transform matrix
                T_cam_tag = np.eye(4)
                T_cam_tag[:3, :3] = det.pose_R
                T_cam_tag[:3, 3] = det.pose_t.flatten()
                tag_pose = T_cam_tag
                
                # Draw visualization
                corners = det.corners.astype(int)
                cv2.polylines(frame, [corners.reshape((-1, 1, 2))], True, (0, 255, 0), 2)
                center = tuple(det.center.astype(int))
                cv2.putText(frame, f"ID:{det.tag_id}", (center[0]-20, center[1]-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, "✓ Detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Searching for tag...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        
        if display:
            cv2.imshow("Calibration Data Recording - Press SPACE to capture, Q to quit", frame)
        
        return tag_pose, frame
    
    def record_pose_pair(self, camera_pose, robot_pose, sample_num):
        """Record a pair of camera and robot poses."""
        self.camera_poses.append(camera_pose)
        self.robot_poses.append(robot_pose)
        
        self.calibration_data.append({
            'sample': sample_num,
            'camera_pose': camera_pose.tolist(),
            'robot_pose': robot_pose.tolist()
        })
        
        print(f"✅ Sample {sample_num} recorded!")
        print(f"   Camera pose (Camera -> Tag):")
        print(f"   Position: {camera_pose[:3, 3]}")
        print(f"   Robot pose (Base -> EE): {robot_pose[:3, 3]}")
    
    def save_data(self):
        """Save calibration data to disk."""
        if not self.camera_poses:
            print("⚠️ No data to save")
            return
        
        # Save as numpy arrays
        camera_poses_array = np.array(self.camera_poses)
        robot_poses_array = np.array(self.robot_poses)
        
        camera_file = self.data_dir / 'camera_poses.npy'
        robot_file = self.data_dir / 'robot_poses.npy'
        
        np.save(camera_file, camera_poses_array)
        np.save(robot_file, robot_poses_array)
        
        print(f"💾 Saved camera poses to: {camera_file}")
        print(f"💾 Saved robot poses to: {robot_file}")
        
        # Also save as JSON for readability
        json_file = self.data_dir / 'calibration_data.json'
        with open(json_file, 'w') as f:
            json.dump({
                'metadata': {
                    'tag_size': self.tag_size,
                    'num_samples': len(self.calibration_data)
                },
                'samples': self.calibration_data
            }, f, indent=2)
        
        print(f"💾 Saved calibration data to: {json_file}")
    
    def cleanup(self):
        """Clean up resources."""
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main():
    """Main entry point for calibration data recording."""
    recorder = CalibrationDataRecorder()
    
    print("\n" + "="*70)
    print("📊 Calibration Data Recorder")
    print("="*70)
    print("\nInstructions:")
    print("  1. Position robot arm with camera looking at AprilTag")
    print("  2. Get robot pose (Base -> End Effector)")
    print("  3. Press SPACE to capture camera pose")
    print("  4. Repeat for different configurations")
    print("  5. Press Q to finish and save data")
    print("\n" + "="*70 + "\n")
    
    sample_count = 0
    
    try:
        while True:
            camera_pose, frame = recorder.detect_tag_pose(display=True)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord(' '):
                if camera_pose is not None:
                    # Prompt user for robot pose
                    robot_pose_input = input("\nEnter robot pose (4x4 matrix as comma-separated values, "
                                            "or 'skip' to use identity): ")
                    
                    if robot_pose_input.lower() == 'skip':
                        robot_pose = np.eye(4)
                        print("⚠️ Using identity matrix as placeholder")
                    else:
                        try:
                            values = [float(x.strip()) for x in robot_pose_input.split(',')]
                            if len(values) == 16:
                                robot_pose = np.array(values).reshape(4, 4)
                            else:
                                print("❌ Invalid number of values. Expected 16 for 4x4 matrix")
                                continue
                        except ValueError:
                            print("❌ Invalid input format")
                            continue
                    
                    sample_count += 1
                    recorder.record_pose_pair(camera_pose, robot_pose, sample_count)
                else:
                    print("⚠️ No tag detected. Try again.")
    
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user (Ctrl+C)")
    
    except Exception as e:
        print("\n❌ Error occurred:")
        traceback.print_exc()
    
    finally:
        recorder.cleanup()
        recorder.save_data()
        print("\n🛑 Program terminated")
        print(f"\n📊 Recorded {sample_count} pose pairs")


if __name__ == '__main__':
    main()
