#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class AprilTagTFConverter(Node):
    def __init__(self):
        super().__init__('apriltag_tf_converter')

        # Subscribe to tag detections
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )

        # Broadcaster to publish camera pose in world
        self.br = TransformBroadcaster(self)

        # Define your tag's world position (tag_id : [x, y, z, qx, qy, qz, qw])
        self.world_tags = {
            0: [1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0],  # example: tag 0 at (1,0.5,0)
            1: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]   # example: tag 1 at origin
        }

    def tag_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) == 0:
            return

        # For simplicity, take first detection
        det = msg.detections[0]
        tag_id = det.id[0]

        if tag_id not in self.world_tags:
            self.get_logger().warn(f"Tag {tag_id} not defined in world frame")
            return

        # Pose of tag relative to camera
        tag_pose_cam = det.pose.pose.pose  # geometry_msgs/Pose

        # Convert tag pose to transformation matrix
        trans = [tag_pose_cam.position.x,
                 tag_pose_cam.position.y,
                 tag_pose_cam.position.z]
        rot = [tag_pose_cam.orientation.x,
               tag_pose_cam.orientation.y,
               tag_pose_cam.orientation.z,
               tag_pose_cam.orientation.w]

        T_camera_to_tag = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(trans),
            tf_transformations.quaternion_matrix(rot)
        )

        # Invert to get tag -> camera
        T_tag_to_camera = tf_transformations.inverse_matrix(T_camera_to_tag)

        # Tag in world
        w = self.world_tags[tag_id]
        T_world_to_tag = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(w[:3]),
            tf_transformations.quaternion_matrix(w[3:])
        )

        # Compute camera in world
        T_world_to_camera = tf_transformations.concatenate_matrices(
            T_world_to_tag,
            T_tag_to_camera
        )

        # Extract translation and rotation
        t = tf_transformations.translation_from_matrix(T_world_to_camera)
        q = tf_transformations.quaternion_from_matrix(T_world_to_camera)

        # Publish as TransformStamped
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = "world"
        t_msg.child_frame_id = "camera_in_world"
        t_msg.transform.translation.x = t[0]
        t_msg.transform.translation.y = t[1]
        t_msg.transform.translation.z = t[2]
        t_msg.transform.rotation.x = q[0]
        t_msg.transform.rotation.y = q[1]
        t_msg.transform.rotation.z = q[2]
        t_msg.transform.rotation.w = q[3]

        self.br.sendTransform(t_msg)
        self.get_logger().info(f"Published camera pose in world using tag {tag_id}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTFConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
