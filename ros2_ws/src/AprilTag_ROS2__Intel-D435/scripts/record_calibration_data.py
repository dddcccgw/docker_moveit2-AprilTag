import numpy as np
import cv2
import json
from scipy.spatial.transform import Rotation
from my_camera_apriltag import detector, camera_params, TAG_SIZE  # reuse your config

# ---- user provides or loads robot poses ----
# Example: robot pose list from JSON or ROS listener
robot_poses = []  # each is 4x4 np.array (base->ee)

# ---- detect tag pose (camera->tag) ----
def detect_tag_pose():
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray, estimate_tag_pose=True,
                                         camera_params=camera_params, tag_size=TAG_SIZE)
            if detections:
                det = detections[0]  # assume single tag target
                pose_R, pose_t = det.pose_R, det.pose_t
                if pose_R is not None and pose_t is not None:
                    T_cam_tag = np.eye(4)
                    T_cam_tag[:3, :3] = pose_R
                    T_cam_tag[:3, 3] = pose_t.flatten()
                    return T_cam_tag
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# ---- collect data ----
camera_poses = []
print("Press ENTER to capture sample, or 'q' to quit")

while True:
    key = input("Capture sample? [Enter/q]: ")
    if key.lower() == 'q':
        break
    T_cam_tag = detect_tag_pose()
    # get robot pose here (replace with your robot API)
    T_base_ee = np.eye(4)  # placeholder
    camera_poses.append(T_cam_tag)
    robot_poses.append(T_base_ee)
    print("✅ Captured one pose pair")

# ---- save ----
import os

os.makedirs("data", exist_ok=True)
np.save("data/camera_poses.npy", np.array(camera_poses))
np.save("data/robot_poses.npy", np.array(robot_poses))
print("💾 Saved all calibration data!")
