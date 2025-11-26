#!/usr/bin/env python3
"""
MoveIt2 Interface for ViperX-300S

This module provides a Python interface to MoveIt2 for motion planning
with collision avoidance and trajectory execution.

Features:
- MoveGroup interface for planning
- Cartesian path planning
- Collision-aware motion planning
- Joint and pose goal setting
- Scene object management
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
from scipy.spatial.transform import Rotation
import time
from typing import Optional, List, Tuple

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotTrajectory,
    MoveItErrorCodes,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Try to import moveit_commander (Python interface)
try:
    import moveit_commander
    HAS_MOVEIT_COMMANDER = True
except ImportError:
    HAS_MOVEIT_COMMANDER = False


class MoveIt2Interface:
    """
    High-level interface to MoveIt2 for the ViperX-300S robot arm.
    
    This class provides methods for:
    - Planning to joint goals
    - Planning to Cartesian pose goals
    - Planning Cartesian paths
    - Executing planned trajectories
    - Managing collision objects
    """
    
    def __init__(self, node: Node, 
                 group_name: str = "interbotix_arm",
                 base_link: str = "vx300s/base_link",
                 ee_link: str = "vx300s/ee_gripper_link"):
        """
        Initialize MoveIt2 interface.
        
        Args:
            node: ROS2 node to attach to
            group_name: MoveIt planning group name
            base_link: Base link frame name
            ee_link: End-effector link frame name
        """
        self.node = node
        self.group_name = group_name
        self.base_link = base_link
        self.ee_link = ee_link
        
        self.logger = node.get_logger()
        
        # Initialize moveit_commander if available
        self.moveit_commander_available = HAS_MOVEIT_COMMANDER
        if HAS_MOVEIT_COMMANDER:
            try:
                self.logger.info("Initializing MoveIt Commander...")
                moveit_commander.roscpp_initialize([])
                self.robot = moveit_commander.RobotCommander()
                self.scene = moveit_commander.PlanningSceneInterface()
                self.move_group = moveit_commander.MoveGroupCommander(group_name)
                
                # Configure planning
                self.move_group.set_planning_time(5.0)
                self.move_group.set_num_planning_attempts(10)
                self.move_group.set_max_velocity_scaling_factor(0.5)
                self.move_group.set_max_acceleration_scaling_factor(0.5)
                self.move_group.allow_replanning(True)
                
                self.logger.info(f"MoveIt Commander initialized for group: {group_name}")
                self.logger.info(f"Planning frame: {self.move_group.get_planning_frame()}")
                self.logger.info(f"End effector: {self.move_group.get_end_effector_link()}")
                
            except Exception as e:
                self.logger.error(f"Failed to initialize MoveIt Commander: {e}")
                self.moveit_commander_available = False
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Current robot state
        self.current_joint_state = None
        self.current_pose = None
        
        # Subscribe to joint states
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Service clients (fallback if commander not available)
        self.get_motion_plan_client = node.create_client(
            GetMotionPlan,
            '/plan_kinematic_path',
            callback_group=self.callback_group
        )
        
        # Action clients
        self.move_action_client = ActionClient(
            node,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )
        
        self.execute_trajectory_client = ActionClient(
            node,
            ExecuteTrajectory,
            '/execute_trajectory',
            callback_group=self.callback_group
        )
        
        self.logger.info("MoveIt2 Interface initialized")
    
    def _joint_state_callback(self, msg: JointState):
        """Update current joint state."""
        self.current_joint_state = msg
    
    def get_current_joint_values(self) -> Optional[List[float]]:
        """Get current joint values for the planning group."""
        if self.moveit_commander_available:
            try:
                return self.move_group.get_current_joint_values()
            except:
                pass
        
        # Fallback: parse from joint_states
        if self.current_joint_state is not None:
            # Filter joints belonging to the arm
            arm_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
            values = []
            for joint in arm_joints:
                try:
                    idx = self.current_joint_state.name.index(joint)
                    values.append(self.current_joint_state.position[idx])
                except ValueError:
                    pass
            if len(values) == 5:
                return values
        
        return None
    
    def get_current_pose(self) -> Optional[PoseStamped]:
        """Get current end-effector pose."""
        if self.moveit_commander_available:
            try:
                return self.move_group.get_current_pose()
            except:
                pass
        return self.current_pose
    
    def plan_to_joint_goal(self, joint_values: List[float]) -> Optional[RobotTrajectory]:
        """
        Plan motion to specified joint values.
        
        Args:
            joint_values: Target joint positions (5 values for ViperX-300S arm)
            
        Returns:
            Planned trajectory or None if planning failed
        """
        if not self.moveit_commander_available:
            self.logger.error("MoveIt Commander not available")
            return None
        
        try:
            self.logger.info(f"Planning to joint goal: {joint_values}")
            self.move_group.set_joint_value_target(joint_values)
            plan = self.move_group.plan()
            
            # plan() returns (success, trajectory, planning_time, error_code) in newer versions
            if isinstance(plan, tuple):
                success, trajectory, _, _ = plan
                if success:
                    self.logger.info("Joint planning succeeded")
                    return trajectory
            else:
                # Older API returns just trajectory
                if plan.joint_trajectory.points:
                    self.logger.info("Joint planning succeeded")
                    return plan
            
            self.logger.warn("Joint planning failed")
            return None
            
        except Exception as e:
            self.logger.error(f"Joint planning error: {e}")
            return None
    
    def plan_to_pose_goal(self, target_pose: Pose, 
                          frame_id: str = "") -> Optional[RobotTrajectory]:
        """
        Plan motion to specified Cartesian pose.
        
        Args:
            target_pose: Target pose (position + orientation)
            frame_id: Reference frame (default: planning frame)
            
        Returns:
            Planned trajectory or None if planning failed
        """
        if not self.moveit_commander_available:
            self.logger.error("MoveIt Commander not available")
            return None
        
        try:
            self.logger.info(f"Planning to pose goal: "
                           f"pos=({target_pose.position.x:.3f}, "
                           f"{target_pose.position.y:.3f}, "
                           f"{target_pose.position.z:.3f})")
            
            # Create PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id or self.move_group.get_planning_frame()
            pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
            pose_stamped.pose = target_pose
            
            self.move_group.set_pose_target(pose_stamped)
            plan = self.move_group.plan()
            
            # Handle different plan() return formats
            if isinstance(plan, tuple):
                success, trajectory, _, _ = plan
                if success:
                    self.logger.info("Pose planning succeeded")
                    return trajectory
            else:
                if plan.joint_trajectory.points:
                    self.logger.info("Pose planning succeeded")
                    return plan
            
            self.logger.warn("Pose planning failed")
            return None
            
        except Exception as e:
            self.logger.error(f"Pose planning error: {e}")
            return None
    
    def plan_cartesian_path(self, waypoints: List[Pose], 
                           eef_step: float = 0.01) -> Optional[Tuple[RobotTrajectory, float]]:
        """
        Plan a Cartesian path through a series of waypoints.
        
        Args:
            waypoints: List of poses to pass through
            eef_step: Step size for Cartesian interpolation (meters)
            
        Returns:
            Tuple of (trajectory, fraction_achieved) or None if planning failed
        """
        if not self.moveit_commander_available:
            self.logger.error("MoveIt Commander not available")
            return None
        
        try:
            self.logger.info(f"Planning Cartesian path through {len(waypoints)} waypoints")
            
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,
                eef_step,
                0.0  # jump_threshold (0.0 = disabled)
            )
            
            if fraction > 0.9:  # At least 90% of path achieved
                self.logger.info(f"Cartesian planning succeeded ({fraction*100:.1f}%)")
                return (plan, fraction)
            else:
                self.logger.warn(f"Cartesian planning incomplete ({fraction*100:.1f}%)")
                return None
                
        except Exception as e:
            self.logger.error(f"Cartesian planning error: {e}")
            return None
    
    def execute_trajectory(self, trajectory: RobotTrajectory, 
                          wait: bool = True) -> bool:
        """
        Execute a planned trajectory.
        
        Args:
            trajectory: Trajectory to execute
            wait: Whether to wait for completion
            
        Returns:
            True if execution succeeded
        """
        if not self.moveit_commander_available:
            self.logger.error("MoveIt Commander not available")
            return False
        
        try:
            self.logger.info("Executing trajectory...")
            success = self.move_group.execute(trajectory, wait=wait)
            
            if success:
                self.logger.info("Trajectory execution succeeded")
            else:
                self.logger.warn("Trajectory execution failed")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Trajectory execution error: {e}")
            return False
    
    def plan_and_execute_to_pose(self, target_pose: Pose, 
                                 frame_id: str = "") -> bool:
        """
        Plan and execute motion to a Cartesian pose.
        
        Args:
            target_pose: Target pose
            frame_id: Reference frame
            
        Returns:
            True if successful
        """
        trajectory = self.plan_to_pose_goal(target_pose, frame_id)
        if trajectory is None:
            return False
        
        return self.execute_trajectory(trajectory)
    
    def plan_and_execute_to_joints(self, joint_values: List[float]) -> bool:
        """
        Plan and execute motion to joint values.
        
        Args:
            joint_values: Target joint positions
            
        Returns:
            True if successful
        """
        trajectory = self.plan_to_joint_goal(joint_values)
        if trajectory is None:
            return False
        
        return self.execute_trajectory(trajectory)
    
    def go_to_named_target(self, target_name: str) -> bool:
        """
        Move to a named target (e.g., "home", "sleep").
        
        Args:
            target_name: Name of the target configuration
            
        Returns:
            True if successful
        """
        if not self.moveit_commander_available:
            self.logger.error("MoveIt Commander not available")
            return False
        
        try:
            self.logger.info(f"Moving to named target: {target_name}")
            self.move_group.set_named_target(target_name)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if success:
                self.logger.info(f"Reached target: {target_name}")
            else:
                self.logger.warn(f"Failed to reach target: {target_name}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Named target error: {e}")
            return False
    
    def stop(self):
        """Stop current motion."""
        if self.moveit_commander_available:
            try:
                self.move_group.stop()
                self.logger.info("Motion stopped")
            except Exception as e:
                self.logger.error(f"Stop error: {e}")
    
    def add_box_to_scene(self, name: str, pose: PoseStamped, 
                         size: Tuple[float, float, float]):
        """
        Add a box obstacle to the planning scene.
        
        Args:
            name: Name of the box
            pose: Pose of the box center
            size: (length, width, height) in meters
        """
        if self.moveit_commander_available:
            try:
                self.scene.add_box(name, pose, size)
                self.logger.info(f"Added box '{name}' to scene")
            except Exception as e:
                self.logger.error(f"Failed to add box: {e}")
    
    def remove_from_scene(self, name: str):
        """Remove an object from the planning scene."""
        if self.moveit_commander_available:
            try:
                self.scene.remove_world_object(name)
                self.logger.info(f"Removed '{name}' from scene")
            except Exception as e:
                self.logger.error(f"Failed to remove object: {e}")
    
    def clear_scene(self):
        """Clear all objects from the planning scene."""
        if self.moveit_commander_available:
            try:
                self.scene.clear()
                self.logger.info("Cleared planning scene")
            except Exception as e:
                self.logger.error(f"Failed to clear scene: {e}")
    
    def create_pose(self, x: float, y: float, z: float,
                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Pose:
        """
        Create a Pose from position and Euler angles.
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
            
        Returns:
            Pose message
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Convert Euler to quaternion
        rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rot.as_quat()  # [x, y, z, w]
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def shutdown(self):
        """Clean up resources."""
        if self.moveit_commander_available:
            try:
                moveit_commander.roscpp_shutdown()
                self.logger.info("MoveIt Commander shutdown")
            except:
                pass


def main():
    """Test the MoveIt2 interface."""
    rclpy.init()
    node = Node('moveit2_interface_test')
    
    # Create interface
    moveit_interface = MoveIt2Interface(node)
    
    # Test getting current state
    node.get_logger().info("Current joint values:")
    joints = moveit_interface.get_current_joint_values()
    if joints:
        node.get_logger().info(f"  {joints}")
    
    # Test planning to home
    node.get_logger().info("Planning to home...")
    success = moveit_interface.go_to_named_target("home")
    node.get_logger().info(f"Result: {success}")
    
    moveit_interface.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
