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

# ====== Expected Tag Positions (in meters) ======
# Tag 0 position
x0, y0, z0 = 0.0, 0.0, 0.0

# Tag 1 position (measured from your actual setup)
x1, y1, z1 = 0.197, -0.001, -0.021

# Tag 2 position (measured from your actual setup)
x2, y2, z2 = -0.001, -0.101, -0.004

# Expected tag positions in map frame
EXPECTED_TAG_POSITIONS = {
    0: np.array([x0, y0, z0]),
    1: np.array([x1, y1, z1]),
    2: np.array([x2, y2, z2])
}

# Expected camera position relative to Tag 0 (map origin)
# Based on your measurement: [-0.0881, 0.0598, -0.5885]
EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])

# Tolerance for position validation (meters)
POSITION_TOLERANCE = 0.015  # 1.5cm tolerance (increased for real-world variance)

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
    quad_decimate=2.0,
    quad_sigma=0.8,
    refine_edges=1,
    decode_sharpening=0.25
)
print("✅ AprilTag detector initialized! Press Q to quit")
print(f"🗺️  Map Origin: Tag {MAP_ORIGIN_TAG_ID}")
print(f"🎯 Expected Camera Position: {EXPECTED_CAMERA_POSITION} m")
print("\n" + "="*70)
print("📐 EXPECTED PHYSICAL SETUP:")
print("="*70)
print(f"Tag 0 (Origin): X={x0:6.3f}m, Y={y0:6.3f}m, Z={z0:6.3f}m")
print(f"Tag 1:          X={x1:6.3f}m, Y={y1:6.3f}m, Z={z1:6.3f}m")
print(f"Tag 2:          X={x2:6.3f}m, Y={y2:6.3f}m, Z={z2:6.3f}m")
print("\nCoordinate System (when looking at Tag 0):")
print("  +X axis: Points to the RIGHT")
print("  +Y axis: Points UPWARD")
print("  +Z axis: Points OUT toward camera")
print("="*70 + "\n")

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
    """Transform Tag pose from camera frame to Map frame"""
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

def compute_camera_position_in_map(map_origin_R, map_origin_t):
    """
    Compute camera position in map frame
    Camera is at origin in camera frame, transform to map frame
    """
    T_cam_to_map_origin = np.eye(4)
    T_cam_to_map_origin[:3, :3] = map_origin_R
    T_cam_to_map_origin[:3, 3] = map_origin_t.flatten()
    
    T_map_origin_to_cam = np.linalg.inv(T_cam_to_map_origin)
    
    # Camera position in map frame is translation of the inverse transform
    camera_pos_map = T_map_origin_to_cam[:3, 3]
    
    return camera_pos_map

def validate_position(actual_pos, expected_pos, tolerance):
    """Check if actual position is within tolerance of expected position"""
    error = np.linalg.norm(actual_pos - expected_pos)
    return error <= tolerance, error

def draw_validation_overlay(frame, tag_data, camera_pos_map, map_origin_id):
    """Draw validation information overlay on frame"""
    y_offset = 30
    
    # Title
    title = f"=== Camera Position Validator (Origin: Tag {map_origin_id}) ==="
    cv2.putText(frame, title, (10, y_offset), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
    cv2.putText(frame, title, (10, y_offset), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    y_offset += 35
    
    # Camera position validation
    if camera_pos_map is not None:
        cam_valid, cam_error = validate_position(camera_pos_map, EXPECTED_CAMERA_POSITION, POSITION_TOLERANCE)
        cam_status = "✓ OK" if cam_valid else "✗ ERROR"
        cam_color = (0, 255, 0) if cam_valid else (0, 0, 255)
        
        text = f"Camera: [{camera_pos_map[0]:7.4f}, {camera_pos_map[1]:7.4f}, {camera_pos_map[2]:7.4f}] m {cam_status}"
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 3)
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cam_color, 2)
        y_offset += 25
        
        text = f"Expected: [{EXPECTED_CAMERA_POSITION[0]:7.4f}, {EXPECTED_CAMERA_POSITION[1]:7.4f}, {EXPECTED_CAMERA_POSITION[2]:7.4f}] m"
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 25
        
        text = f"Error: {cam_error*1000:.2f} mm"
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cam_color, 1)
        y_offset += 35
    
    # Tag positions validation
    for tag_id in sorted(tag_data.keys()):
        data = tag_data[tag_id]
        pos = data['map_position']
        expected = EXPECTED_TAG_POSITIONS.get(tag_id)
        
        if expected is not None:
            is_valid, error = validate_position(pos, expected, POSITION_TOLERANCE)
            status = "✓" if is_valid else "✗"
            color = (0, 255, 0) if is_valid else (0, 0, 255)
            
            if tag_id == map_origin_id:
                text = f"Tag {tag_id}: [ORIGIN] {status}"
            else:
                text = f"Tag {tag_id}: [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}] m {status} (Err: {error*1000:.1f}mm)"
            
            cv2.putText(frame, text, (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 3)
            cv2.putText(frame, text, (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_offset += 25
    
    return frame

def validate_pose(pose_R, pose_t):
    """Validate if detected pose is reasonable"""
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

pose_filter = TemporalFilter(window_size=5)
tag_data_storage = {}
camera_position_storage = None

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
        
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        try:
            detections = detector.detect(gray, estimate_tag_pose=True,
                                         camera_params=camera_params, tag_size=TAG_SIZE)
        except Exception as e:
            if frame_count % 30 == 0:
                print(f"⚠️ Detection error: {e}")
            detections = []

        current_frame_tags = {}
        map_origin_data = None

        # First pass: Collect all tag data
        for det in detections:
            if det.tag_id not in TARGET_TAG_IDS:
                continue
            
            if not validate_pose(det.pose_R, det.pose_t):
                continue
            
            filtered_R, filtered_t = pose_filter.update(det.tag_id, det.pose_R, det.pose_t)
            
            corners = det.corners.astype(int)
            color = (0, 255, 0) if det.tag_id == MAP_ORIGIN_TAG_ID else (255, 0, 0)
            cv2.polylines(frame, [corners.reshape((-1,1,2))], True, color, 2)
            
            center = tuple(det.center.astype(int))
            cv2.putText(frame, f"ID:{det.tag_id}", (center[0]-20, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            
            depth = depth_frame.get_distance(int(center[0]), int(center[1]))
            
            rvec, _ = cv2.Rodrigues(filtered_R)
            frame = draw_pose_axes(frame, rvec, filtered_t, camera_matrix, dist_coeffs)
            
            current_frame_tags[det.tag_id] = {
                'pose_R': filtered_R,
                'pose_t': filtered_t,
                'depth': depth,
                'center': center
            }
            
            if det.tag_id == MAP_ORIGIN_TAG_ID:
                map_origin_data = current_frame_tags[det.tag_id]

        # Second pass: Compute positions in map frame
        if map_origin_data is not None:
            map_origin_R = map_origin_data['pose_R']
            map_origin_t = map_origin_data['pose_t']
            
            # Compute camera position in map frame
            camera_pos_map = compute_camera_position_in_map(map_origin_R, map_origin_t)
            camera_position_storage = camera_pos_map
            
            for tag_id, data in current_frame_tags.items():
                if tag_id == MAP_ORIGIN_TAG_ID:
                    data['map_position'] = np.array([0.0, 0.0, 0.0])
                    data['map_rotation'] = np.eye(3)
                else:
                    R_map, t_map = transform_to_map_frame(
                        data['pose_R'], data['pose_t'],
                        map_origin_R, map_origin_t
                    )
                    data['map_position'] = t_map.flatten()
                    data['map_rotation'] = R_map
                
                tag_data_storage[tag_id] = data
            
            # Draw validation overlay
            frame = draw_validation_overlay(frame, tag_data_storage, camera_pos_map, MAP_ORIGIN_TAG_ID)
            
            # Terminal output
            current_time = time.time()
            if current_time - last_print_time >= 2.0:
                print(f"\n{'='*70}")
                print(f"🎯 Camera Position Validation")
                print(f"{'='*70}")
                
                # Camera validation
                cam_valid, cam_error = validate_position(camera_pos_map, EXPECTED_CAMERA_POSITION, POSITION_TOLERANCE)
                print(f"Camera Position: [{camera_pos_map[0]:7.4f}, {camera_pos_map[1]:7.4f}, {camera_pos_map[2]:7.4f}] m")
                print(f"Expected:        [{EXPECTED_CAMERA_POSITION[0]:7.4f}, {EXPECTED_CAMERA_POSITION[1]:7.4f}, {EXPECTED_CAMERA_POSITION[2]:7.4f}] m")
                print(f"Error: {cam_error*1000:.2f} mm - {'✓ PASS' if cam_valid else '✗ FAIL'}")
                
                print(f"\n{'='*70}")
                print(f"Tag Position Validation")
                print(f"{'='*70}")
                
                all_valid = cam_valid
                for tag_id in sorted(tag_data_storage.keys()):
                    data = tag_data_storage[tag_id]
                    pos = data['map_position']
                    expected = EXPECTED_TAG_POSITIONS[tag_id]
                    
                    is_valid, error = validate_position(pos, expected, POSITION_TOLERANCE)
                    all_valid = all_valid and is_valid
                    
                    print(f"Tag {tag_id}:")
                    print(f"  Actual:   [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}] m")
                    print(f"  Expected: [{expected[0]:7.4f}, {expected[1]:7.4f}, {expected[2]:7.4f}] m")
                    print(f"  Error: {error*1000:.2f} mm - {'✓ PASS' if is_valid else '✗ FAIL'}")
                
                print(f"\n{'='*70}")
                print(f"Overall: {'✓✓✓ ALL CHECKS PASSED ✓✓✓' if all_valid else '✗✗✗ VALIDATION FAILED ✗✗✗'}")
                print(f"{'='*70}")
                
                last_print_time = current_time
        else:
            cv2.putText(frame, f"Searching for Map Origin (Tag {MAP_ORIGIN_TAG_ID})...", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)

        status_text = f"Detected: {len(current_frame_tags)}/{len(TARGET_TAG_IDS)} tags | FPS: {FPS}"
        cv2.putText(frame, status_text, (10, FRAME_HEIGHT - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        cv2.imshow("AprilTag Camera Position Validator - Press Q to quit", frame)

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
    
    if tag_data_storage and camera_position_storage is not None:
        print(f"\n{'='*70}")
        print("📊 Final Validation Report")
        print(f"{'='*70}")
        
        # Camera validation
        cam_valid, cam_error = validate_position(camera_position_storage, EXPECTED_CAMERA_POSITION, POSITION_TOLERANCE)
        print(f"\n🎥 Camera Position:")
        print(f"  Measured: [{camera_position_storage[0]:7.4f}, {camera_position_storage[1]:7.4f}, {camera_position_storage[2]:7.4f}] m")
        print(f"  Expected: [{EXPECTED_CAMERA_POSITION[0]:7.4f}, {EXPECTED_CAMERA_POSITION[1]:7.4f}, {EXPECTED_CAMERA_POSITION[2]:7.4f}] m")
        print(f"  Error: {cam_error*1000:.2f} mm - {'✓ PASS' if cam_valid else '✗ FAIL'}")
        
        print(f"\n🏷️  Tag Positions:")
        all_valid = cam_valid
        for tag_id in sorted(tag_data_storage.keys()):
            data = tag_data_storage[tag_id]
            pos = data['map_position']
            expected = EXPECTED_TAG_POSITIONS[tag_id]
            
            is_valid, error = validate_position(pos, expected, POSITION_TOLERANCE)
            all_valid = all_valid and is_valid
            
            print(f"\nTag {tag_id}:")
            print(f"  Measured: [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}] m")
            print(f"  Expected: [{expected[0]:7.4f}, {expected[1]:7.4f}, {expected[2]:7.4f}] m")
            print(f"  Error: {error*1000:.2f} mm - {'✓ PASS' if is_valid else '✗ FAIL'}")
        
        print(f"\n{'='*70}")
        if all_valid:
            print("✓✓✓ ALL VALIDATION CHECKS PASSED ✓✓✓")
            print("Camera and tags are correctly positioned!")
        else:
            print("✗✗✗ VALIDATION FAILED ✗✗✗")
            print("Please adjust camera or tag positions!")
        print(f"{'='*70}")
    else:
        print("\n⚠️ Insufficient data for validation")