import cv2
import numpy as np
import pyrealsense2 as rs
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation
import time
import traceback
import json

# ====== Configuration ======
TAG_SIZE = 0.0625  # meters (6.25 cm)
FPS = 30
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
TARGET_TAG_IDS = [0, 1, 2]  # Three target tags
MAP_ORIGIN_TAG_ID = 0  # Tag 0 as Map coordinate origin

# ====== Initialize RealSense ======
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# Get camera intrinsics
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                          [0, intr.fy, intr.ppy],
                          [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

print(f"📷 Camera Intrinsics:\n   fx: {intr.fx:.2f}, fy: {intr.fy:.2f}, cx: {intr.ppx:.2f}, cy: {intr.ppy:.2f}")

# ====== Initialize AprilTag Detector ======
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=2.0,      # Reduced for better accuracy (was 3.0)
    quad_sigma=0.8,         # Reduced blur for sharper detection (was 1.0)
    refine_edges=1,
    decode_sharpening=0.25  # Increased sharpening (was 0.1)
)
print("✅ AprilTag detector initialized! Press Q to quit")
print(f"🗺️  Map Origin: Tag {MAP_ORIGIN_TAG_ID}")

# ====== Helper Functions ======
def draw_pose_axes(frame, rvec, tvec, camera_matrix, dist_coeffs, length=0.05):
    """Draw 3D coordinate axes"""
    axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = imgpts.astype(int)
    origin = tuple(imgpts[0].ravel())
    frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)  # X-axis Red
    frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)  # Y-axis Green
    frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3)  # Z-axis Blue
    return frame

def transform_to_map_frame(tag_pose_R, tag_pose_t, map_origin_R, map_origin_t):
    """
    Transform Tag pose from camera frame to Map frame
    
    Args:
        tag_pose_R: Target tag rotation matrix (relative to camera)
        tag_pose_t: Target tag translation vector (relative to camera)
        map_origin_R: Map origin tag rotation matrix (relative to camera)
        map_origin_t: Map origin tag translation vector (relative to camera)
    
    Returns:
        (R_map, t_map): Target tag pose relative to Map origin
    """
    # Build transform matrix: Camera -> Map origin tag
    T_cam_to_map_origin = np.eye(4)
    T_cam_to_map_origin[:3, :3] = map_origin_R
    T_cam_to_map_origin[:3, 3] = map_origin_t.flatten()
    
    # Invert: Map origin -> Camera
    T_map_origin_to_cam = np.linalg.inv(T_cam_to_map_origin)
    
    # Build transform matrix: Camera -> Target tag
    T_cam_to_tag = np.eye(4)
    T_cam_to_tag[:3, :3] = tag_pose_R
    T_cam_to_tag[:3, 3] = tag_pose_t.flatten()
    
    # Compute: Map origin -> Target tag
    T_map_origin_to_tag = T_map_origin_to_cam @ T_cam_to_tag
    
    # Extract rotation and translation
    R_map = T_map_origin_to_tag[:3, :3]
    t_map = T_map_origin_to_tag[:3, 3].reshape(3, 1)
    
    return R_map, t_map

def draw_map_info(frame, tag_data, map_origin_id):
    """Draw Map information overlay on frame"""
    y_offset = 30
    
    # Title with background for better visibility
    title = f"=== Map Frame (Origin: Tag {map_origin_id}) ==="
    cv2.putText(frame, title, (10, y_offset), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)  # Black shadow
    cv2.putText(frame, title, (10, y_offset), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)  # Cyan text
    y_offset += 30
    
    # Display each tag's position in Map frame
    for tag_id in sorted(tag_data.keys()):
        data = tag_data[tag_id]
        
        if tag_id == map_origin_id:
            text = f"Tag {tag_id}: [ORIGIN] X=0.000 Y=0.000 Z=0.000 m"
            color = (0, 255, 0)  # Green
        else:
            pos = data['map_position']
            text = f"Tag {tag_id}: X={pos[0]:6.3f} Y={pos[1]:6.3f} Z={pos[2]:6.3f} m"
            color = (255, 255, 255)  # White
        
        # Draw with shadow for readability
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 3)
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y_offset += 25
    
    return frame

def validate_pose(pose_R, pose_t):
    """
    Validate if detected pose is reasonable
    
    Args:
        pose_R: Rotation matrix (3x3)
        pose_t: Translation vector (3x1)
    
    Returns:
        bool: True if valid, False otherwise
    """
    if pose_R is None or pose_t is None:
        return False
    
    # Check if rotation matrix is valid (determinant should be close to 1)
    det = np.linalg.det(pose_R)
    if abs(det - 1.0) > 0.1:
        return False
    
    # Check if translation is reasonable (not NaN or too large)
    if np.any(np.isnan(pose_t)) or np.any(np.isinf(pose_t)):
        return False
    
    distance = np.linalg.norm(pose_t)
    if distance < 0.05 or distance > 5.0:  # Between 5cm and 5m
        return False
    
    return True

# ====== Temporal Filtering for Stability ======
class TemporalFilter:
    """Simple moving average filter for pose smoothing"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = {}
    
    def update(self, tag_id, pose_R, pose_t):
        """Add new pose and return filtered pose"""
        if tag_id not in self.buffer:
            self.buffer[tag_id] = {'R': [], 't': []}
        
        # Add to buffer
        self.buffer[tag_id]['R'].append(pose_R)
        self.buffer[tag_id]['t'].append(pose_t)
        
        # Keep only last N frames
        if len(self.buffer[tag_id]['R']) > self.window_size:
            self.buffer[tag_id]['R'].pop(0)
            self.buffer[tag_id]['t'].pop(0)
        
        # Compute average
        avg_t = np.mean(self.buffer[tag_id]['t'], axis=0)
        
        # For rotation, average and re-orthogonalize
        avg_R = np.mean(self.buffer[tag_id]['R'], axis=0)
        U, _, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        return avg_R, avg_t
    
    def clear(self, tag_id=None):
        """Clear buffer for specific tag or all tags"""
        if tag_id is not None:
            if tag_id in self.buffer:
                del self.buffer[tag_id]
        else:
            self.buffer.clear()

# Initialize temporal filter
pose_filter = TemporalFilter(window_size=5)

# ====== Storage for Tag data ======
tag_data_storage = {}

# ====== Main Loop ======
try:
    frame_count = 0
    last_print_time = time.time()
    
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Image preprocessing (improved detection stability)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        # AprilTag Detection with error handling
        try:
            detections = detector.detect(gray, estimate_tag_pose=True,
                                         camera_params=camera_params, tag_size=TAG_SIZE)
        except Exception as e:
            if frame_count % 30 == 0:  # Print error once per second
                print(f"⚠️ Detection error: {e}")
            detections = []
            cv2.putText(frame, "Detection Error!", (10, FRAME_HEIGHT - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # Temporary storage for current frame tags
        current_frame_tags = {}
        map_origin_data = None

        # First pass: Collect all tag data (relative to camera)
        for det in detections:
            if det.tag_id not in TARGET_TAG_IDS:
                continue
            
            # Validate pose before using
            if not validate_pose(det.pose_R, det.pose_t):
                continue
            
            # Apply temporal filtering for stability
            filtered_R, filtered_t = pose_filter.update(det.tag_id, det.pose_R, det.pose_t)
            
            # Draw tag border
            corners = det.corners.astype(int)
            color = (0, 255, 0) if det.tag_id == MAP_ORIGIN_TAG_ID else (255, 0, 0)
            cv2.polylines(frame, [corners.reshape((-1,1,2))], True, color, 2)
            
            # Draw Tag ID
            center = tuple(det.center.astype(int))
            cv2.putText(frame, f"ID:{det.tag_id}", (center[0]-20, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            
            # Get depth
            depth = depth_frame.get_distance(int(center[0]), int(center[1]))
            
            # Draw camera frame coordinate axes
            rvec, _ = cv2.Rodrigues(filtered_R)
            frame = draw_pose_axes(frame, rvec, filtered_t, camera_matrix, dist_coeffs)
            
            # Store tag data
            current_frame_tags[det.tag_id] = {
                'pose_R': filtered_R,
                'pose_t': filtered_t,
                'depth': depth,
                'center': center
            }
            
            # Record map origin
            if det.tag_id == MAP_ORIGIN_TAG_ID:
                map_origin_data = current_frame_tags[det.tag_id]

        # Second pass: If map origin found, compute other tags' positions in Map frame
        if map_origin_data is not None:
            map_origin_R = map_origin_data['pose_R']
            map_origin_t = map_origin_data['pose_t']
            
            for tag_id, data in current_frame_tags.items():
                if tag_id == MAP_ORIGIN_TAG_ID:
                    # Origin position in Map is (0, 0, 0)
                    data['map_position'] = np.array([0.0, 0.0, 0.0])
                    data['map_rotation'] = np.eye(3)
                else:
                    # Transform to Map coordinate system
                    R_map, t_map = transform_to_map_frame(
                        data['pose_R'], data['pose_t'],
                        map_origin_R, map_origin_t
                    )
                    data['map_position'] = t_map.flatten()
                    data['map_rotation'] = R_map
                
                # Update storage
                tag_data_storage[tag_id] = data
            
            # Draw Map info overlay
            frame = draw_map_info(frame, tag_data_storage, MAP_ORIGIN_TAG_ID)
            
            # Terminal output (every 2 seconds to reduce spam)
            current_time = time.time()
            if current_time - last_print_time >= 2.0:
                print(f"\n{'='*60}")
                print(f"🗺️  Map Frame Status (Origin: Tag {MAP_ORIGIN_TAG_ID})")
                print(f"{'='*60}")
                for tag_id in sorted(tag_data_storage.keys()):
                    data = tag_data_storage[tag_id]
                    pos = data['map_position']
                    euler = Rotation.from_matrix(data['map_rotation']).as_euler('xyz', degrees=True)
                    
                    if tag_id == MAP_ORIGIN_TAG_ID:
                        print(f"Tag {tag_id} [ORIGIN]: (0.000, 0.000, 0.000) m")
                    else:
                        print(f"Tag {tag_id}: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) m")
                        print(f"         Rotation: R={euler[0]:6.1f}° P={euler[1]:6.1f}° Y={euler[2]:6.1f}°")
                
                last_print_time = current_time
        else:
            # Map origin not found
            cv2.putText(frame, f"Searching for Map Origin (Tag {MAP_ORIGIN_TAG_ID})...", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)

        # Display number of detected tags
        status_text = f"Detected: {len(current_frame_tags)}/{len(TARGET_TAG_IDS)} tags | FPS: {FPS}"
        cv2.putText(frame, status_text, (10, FRAME_HEIGHT - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # Display results
        cv2.imshow("AprilTag Map Frame - Press Q to quit", frame)

        # Break on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1

except KeyboardInterrupt:
    print("\n⚠️ Interrupted by user (Ctrl+C)")

except Exception as e:
    print("\n❌ Error occurred:")
    traceback.print_exc()

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n🛑 Program terminated")
    
    # Final output and save Map data
    if tag_data_storage:
        print(f"\n{'='*60}")
        print("📊 Final Map Data")
        print(f"{'='*60}")
        
        # Prepare data for saving
        map_data = {
            'metadata': {
                'map_origin_tag_id': MAP_ORIGIN_TAG_ID,
                'tag_size': TAG_SIZE,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'camera_intrinsics': {
                    'fx': intr.fx,
                    'fy': intr.fy,
                    'cx': intr.ppx,
                    'cy': intr.ppy
                }
            },
            'tags': {}
        }
        
        for tag_id in sorted(tag_data_storage.keys()):
            data = tag_data_storage[tag_id]
            pos = data['map_position']
            rot = data['map_rotation']
            euler = Rotation.from_matrix(rot).as_euler('xyz', degrees=True)
            
            print(f"Tag {tag_id}: Position = [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
            print(f"         Rotation = [R={euler[0]:.2f}°, P={euler[1]:.2f}°, Y={euler[2]:.2f}°]")
            
            # Store in dictionary
            map_data['tags'][str(tag_id)] = {
                'position': pos.tolist(),
                'rotation_matrix': rot.tolist(),
                'rotation_euler_deg': euler.tolist(),
                'is_origin': (tag_id == MAP_ORIGIN_TAG_ID)
            }
        
        # Save as JSON file
        try:
            with open('apriltag_map.json', 'w') as f:
                json.dump(map_data, f, indent=2)
            print("\n💾 Map data saved to: apriltag_map.json")
        except Exception as e:
            print(f"\n⚠️ Failed to save map data: {e}")
    else:
        print("\n⚠️ No tag data collected")