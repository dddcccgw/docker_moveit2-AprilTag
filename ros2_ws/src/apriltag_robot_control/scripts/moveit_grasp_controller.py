#!/usr/bin/env python3
"""
MoveIt Grasp Controller Node

This node controls the ViperX-300S robot arm to grasp objects detected by AprilTags.
It uses MoveIt2 for collision-aware motion planning and the Interbotix API for gripper control.

Subscribes to:
  - /grasp_pose (geometry_msgs/PoseStamped): Target grasp position
  - /target_detected (std_msgs/Bool): Target detection status

Services:
  - /execute_grasp (std_srvs/Trigger): Trigger grasp execution
  - /go_home (std_srvs/Trigger): Move robot to home position
  - /release_object (std_srvs/Trigger): Open gripper to release object
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
from scipy.spatial.transform import Rotation
import time
import sys
import os

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo

# Import our MoveIt2 interface
sys.path.insert(0, os.path.dirname(__file__))
from moveit2_interface import MoveIt2Interface


class MoveItGraspController(Node):
    """
    ROS2 Node for robot arm grasp control using Interbotix API.
    """
    
    def __init__(self):
        super().__init__('moveit_grasp_controller')
        
        # Declare parameters
        self.declare_parameter('robot_model', 'vx300s')
        self.declare_parameter('robot_name', 'vx300s')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('planning_group', 'interbotix_arm')
        self.declare_parameter('use_moveit', True)  # Use MoveIt2 for motion planning
        self.declare_parameter('approach_height', 0.15)  # Height above target for approach
        self.declare_parameter('grasp_height', 0.02)     # Final grasp height above target
        self.declare_parameter('retreat_height', 0.15)   # Height for retreat after grasp
        self.declare_parameter('moving_time', 2.0)
        self.declare_parameter('accel_time', 0.5)
        self.declare_parameter('gripper_pressure', 0.5)
        
        # Get parameters
        self.robot_model = self.get_parameter('robot_model').value
        self.robot_name = self.get_parameter('robot_name').value
        self.world_frame = self.get_parameter('world_frame').value
        self.planning_group = self.get_parameter('planning_group').value
        self.use_moveit = self.get_parameter('use_moveit').value
        self.approach_height = self.get_parameter('approach_height').value
        self.grasp_height = self.get_parameter('grasp_height').value
        self.retreat_height = self.get_parameter('retreat_height').value
        self.moving_time = self.get_parameter('moving_time').value
        self.accel_time = self.get_parameter('accel_time').value
        self.gripper_pressure = self.get_parameter('gripper_pressure').value
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize robot (will be done after arm drivers are ready)
        self.robot = None
        self.robot_initialized = False
        
        # Initialize MoveIt2 interface (if enabled)
        self.moveit_interface = None
        self.moveit_initialized = False
        if self.use_moveit:
            try:
                self.get_logger().info('Initializing MoveIt2 interface...')
                self.moveit_interface = MoveIt2Interface(
                    node=self,
                    group_name=self.planning_group,
                    base_link=f'{self.robot_name}/base_link',
                    ee_link=f'{self.robot_name}/ee_gripper_link'
                )
                self.moveit_initialized = True
                self.get_logger().info('MoveIt2 interface initialized!')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize MoveIt2: {e}')
                self.get_logger().warn('Falling back to Interbotix API only')
                self.use_moveit = False
        
        # State
        self.current_grasp_pose = None
        self.target_detected = False
        self.is_executing = False
        
        # Subscribers
        self.grasp_pose_sub = self.create_subscription(
            PoseStamped, 'grasp_pose', self.grasp_pose_callback, 10,
            callback_group=self.callback_group
        )
        self.target_detected_sub = self.create_subscription(
            Bool, 'target_detected', self.target_detected_callback, 10,
            callback_group=self.callback_group
        )
        
        # Services
        self.execute_grasp_srv = self.create_service(
            Trigger, 'execute_grasp', self.execute_grasp_callback,
            callback_group=self.callback_group
        )
        self.go_home_srv = self.create_service(
            Trigger, 'go_home', self.go_home_callback,
            callback_group=self.callback_group
        )
        self.release_object_srv = self.create_service(
            Trigger, 'release_object', self.release_object_callback,
            callback_group=self.callback_group
        )
        
        # Publishers for direct joint control
        self.pub_group = self.create_publisher(
            JointGroupCommand, f'/{self.robot_name}/commands/joint_group', 10
        )
        self.pub_single = self.create_publisher(
            JointSingleCommand, f'/{self.robot_name}/commands/joint_single', 10
        )
        
        # Publisher for status
        self.status_pub = self.create_publisher(Bool, 'grasp_controller_ready', 10)
        
        # Timer to initialize robot
        self.init_timer = self.create_timer(2.0, self.try_init_robot)
        
        self.get_logger().info(f'MoveIt Grasp Controller Node initialized')
        self.get_logger().info(f'Robot: {self.robot_model}, Name: {self.robot_name}')
    
    def try_init_robot(self):
        """Try to initialize the robot arm."""
        if self.robot_initialized:
            # Publish ready status
            msg = Bool()
            msg.data = True
            self.status_pub.publish(msg)
            return
        
        try:
            self.get_logger().info('Attempting to initialize robot arm...')
            
            # Initialize the Interbotix robot
            self.robot = InterbotixManipulatorXS(
                robot_model=self.robot_model,
                robot_name=self.robot_name,
                moving_time=self.moving_time,
                accel_time=self.accel_time,
                gripper_pressure=self.gripper_pressure,
                node=self
            )
            
            self.robot_initialized = True
            self.get_logger().info('Robot arm initialized successfully!')
            
            # Move to home position
            self.go_to_home()
            
            # Cancel init timer
            self.init_timer.cancel()
            
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize robot: {e}')
            self.get_logger().info('Will retry in 2 seconds...')
    
    def grasp_pose_callback(self, msg: PoseStamped):
        """Handle new grasp pose."""
        self.current_grasp_pose = msg
    
    def target_detected_callback(self, msg: Bool):
        """Handle target detection status."""
        self.target_detected = msg.data
    
    def execute_grasp_callback(self, request, response):
        """Execute grasp sequence."""
        if not self.robot_initialized:
            response.success = False
            response.message = "Robot not initialized"
            return response
        
        if self.is_executing:
            response.success = False
            response.message = "Already executing a grasp"
            return response
        
        if not self.target_detected or self.current_grasp_pose is None:
            response.success = False
            response.message = "No target detected"
            return response
        
        self.is_executing = True
        
        try:
            success = self.execute_grasp_sequence()
            response.success = success
            response.message = "Grasp executed successfully" if success else "Grasp failed"
        except Exception as e:
            self.get_logger().error(f'Grasp execution error: {e}')
            response.success = False
            response.message = str(e)
        finally:
            self.is_executing = False
        
        return response
    
    def go_home_callback(self, request, response):
        """Move robot to home position."""
        if not self.robot_initialized:
            response.success = False
            response.message = "Robot not initialized"
            return response
        
        try:
            self.go_to_home()
            response.success = True
            response.message = "Moved to home position"
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def release_object_callback(self, request, response):
        """Release object by opening gripper."""
        if not self.robot_initialized:
            response.success = False
            response.message = "Robot not initialized"
            return response
        
        try:
            self.robot.gripper.release()
            response.success = True
            response.message = "Gripper opened"
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def go_to_home(self):
        """Move robot to home position."""
        self.get_logger().info('Moving to home position...')
        self.robot.arm.go_to_home_pose()
        self.robot.gripper.release()
        self.get_logger().info('Reached home position')
    
    def go_to_sleep(self):
        """Move robot to sleep position."""
        self.get_logger().info('Moving to sleep position...')
        self.robot.gripper.release()
        self.robot.arm.go_to_sleep_pose()
        self.get_logger().info('Reached sleep position')
    
    def execute_grasp_sequence(self):
        """
        Execute complete grasp sequence:
        1. Open gripper
        2. Move to approach position (above target)
        3. Move down to grasp position
        4. Close gripper
        5. Retreat upward
        6. Return to home
        """
        if self.use_moveit and self.moveit_initialized:
            return self._execute_grasp_with_moveit()
        else:
            return self._execute_grasp_with_interbotix()
    
    def _execute_grasp_with_moveit(self):
        """Execute grasp using MoveIt2 for collision-aware planning."""
        pose = self.current_grasp_pose.pose
        target_x = pose.position.x
        target_y = pose.position.y
        target_z = pose.position.z
        
        self.get_logger().info('='*60)
        self.get_logger().info('EXECUTING GRASP WITH MOVEIT2')
        self.get_logger().info(f'Target: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
        self.get_logger().info('='*60)
        
        # Step 1: Open gripper
        self.get_logger().info('[1/6] Opening gripper...')
        self.robot.gripper.release()
        time.sleep(0.5)
        
        # Step 2: Plan and move to approach position
        approach_z = target_z + self.approach_height
        self.get_logger().info(f'[2/6] Planning to approach (z={approach_z:.3f})...')
        
        approach_pose = self.moveit_interface.create_pose(
            target_x, target_y, approach_z,
            pitch=np.pi/2  # Gripper pointing down
        )
        
        success = self.moveit_interface.plan_and_execute_to_pose(approach_pose)
        if not success:
            self.get_logger().error('Failed to reach approach position with MoveIt2')
            self.get_logger().warn('Falling back to Interbotix API...')
            return self._execute_grasp_with_interbotix()
        
        time.sleep(0.5)
        
        # Step 3: Plan Cartesian path down to grasp position
        grasp_z = target_z + self.grasp_height
        self.get_logger().info(f'[3/6] Planning Cartesian descent to z={grasp_z:.3f}...')
        
        grasp_pose = self.moveit_interface.create_pose(
            target_x, target_y, grasp_z,
            pitch=np.pi/2
        )
        
        # Create waypoints for Cartesian path (approach -> grasp)
        waypoints = [approach_pose, grasp_pose]
        result = self.moveit_interface.plan_cartesian_path(waypoints, eef_step=0.005)
        
        if result:
            trajectory, fraction = result
            self.get_logger().info(f'Cartesian path planned ({fraction*100:.1f}% achieved)')
            success = self.moveit_interface.execute_trajectory(trajectory)
        else:
            self.get_logger().warn('Cartesian planning failed, using pose goal...')
            success = self.moveit_interface.plan_and_execute_to_pose(grasp_pose)
        
        if not success:
            self.get_logger().error('Failed to descend to grasp position')
            return False
        
        time.sleep(0.3)
        
        # Step 4: Close gripper
        self.get_logger().info('[4/6] Closing gripper...')
        self.robot.gripper.grasp()
        time.sleep(1.0)
        
        # Step 5: Plan retreat upward
        retreat_z = target_z + self.retreat_height
        self.get_logger().info(f'[5/6] Planning retreat to z={retreat_z:.3f}...')
        
        retreat_pose = self.moveit_interface.create_pose(
            target_x, target_y, retreat_z,
            pitch=np.pi/2
        )
        
        success = self.moveit_interface.plan_and_execute_to_pose(retreat_pose)
        if not success:
            self.get_logger().warn('MoveIt2 retreat failed, using Interbotix...')
            self.robot.arm.set_ee_cartesian_trajectory(z=self.retreat_height, moving_time=1.5)
        
        time.sleep(0.5)
        
        # Step 6: Return to home
        self.get_logger().info('[6/6] Returning to home...')
        success = self.moveit_interface.go_to_named_target("Home")
        if not success:
            self.get_logger().warn('MoveIt2 home failed, using Interbotix...')
            self.robot.arm.go_to_home_pose(moving_time=2.0)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✓ GRASP SEQUENCE COMPLETED!')
        self.get_logger().info('='*60)
        
        return True
    
    def _execute_grasp_with_interbotix(self):
        """Execute grasp using Interbotix API (fallback or direct mode)."""
        pose = self.current_grasp_pose.pose
        target_x = pose.position.x
        target_y = pose.position.y
        target_z = pose.position.z
        
        self.get_logger().info(f'Executing grasp at: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
        
        # Step 1: Open gripper
        self.get_logger().info('Step 1: Opening gripper...')
        self.robot.gripper.release()
        time.sleep(0.5)
        
        # Step 2: Move to approach position (above target)
        approach_z = target_z + self.approach_height
        self.get_logger().info(f'Step 2: Moving to approach position at z={approach_z:.3f}...')
        
        # Use set_ee_pose_components for Cartesian control
        # For ViperX-300S, we typically approach from above
        success, _ = self.robot.arm.set_ee_pose_components(
            x=target_x,
            y=target_y,
            z=approach_z,
            pitch=np.pi/2,  # Gripper pointing down
            moving_time=self.moving_time
        )
        
        if not success:
            self.get_logger().error('Failed to reach approach position')
            # Try alternative approach
            success = self._move_to_position_ik(target_x, target_y, approach_z)
            if not success:
                return False
        
        time.sleep(0.5)
        
        # Step 3: Move down to grasp position
        grasp_z = target_z + self.grasp_height
        self.get_logger().info(f'Step 3: Moving down to grasp position at z={grasp_z:.3f}...')
        
        # Use Cartesian trajectory for smooth descent
        success = self.robot.arm.set_ee_cartesian_trajectory(
            z=-(approach_z - grasp_z),
            moving_time=1.5
        )
        
        if not success:
            self.get_logger().warn('Cartesian descent failed, trying direct IK...')
            success, _ = self.robot.arm.set_ee_pose_components(
                x=target_x,
                y=target_y,
                z=grasp_z,
                pitch=np.pi/2,
                moving_time=1.5
            )
        
        time.sleep(0.3)
        
        # Step 4: Close gripper
        self.get_logger().info('Step 4: Closing gripper...')
        self.robot.gripper.grasp()
        time.sleep(1.0)  # Wait for gripper to close
        
        # Step 5: Retreat upward
        self.get_logger().info(f'Step 5: Retreating to z={approach_z:.3f}...')
        self.robot.arm.set_ee_cartesian_trajectory(
            z=self.retreat_height,
            moving_time=1.5
        )
        time.sleep(0.5)
        
        # Step 6: Return to home
        self.get_logger().info('Step 6: Returning to home position...')
        self.robot.arm.go_to_home_pose(moving_time=2.0)
        
        self.get_logger().info('Grasp sequence completed successfully!')
        return True
    
    def _move_to_position_ik(self, x, y, z, pitch=np.pi/2):
        """
        Alternative method to move to position using IK.
        """
        # Build transformation matrix
        T_sd = np.eye(4)
        
        # Rotation: gripper pointing down
        rot = Rotation.from_euler('y', pitch)
        T_sd[:3, :3] = rot.as_matrix()
        T_sd[:3, 3] = [x, y, z]
        
        theta_list, success = self.robot.arm.set_ee_pose_matrix(
            T_sd,
            moving_time=self.moving_time
        )
        
        return success
    
    def destroy_node(self):
        """Clean up resources."""
        if self.robot_initialized:
            try:
                self.go_to_sleep()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItGraspController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
